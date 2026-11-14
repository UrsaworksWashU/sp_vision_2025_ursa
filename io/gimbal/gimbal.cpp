#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    auto timeout = serial::Timeout::simpleTimeout(100);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  // 导航帧转发(可选键, 缺省关闭). 在等第一帧四元数之前就启动:
  // 下位机还没发姿态时导航照样能收发.
  if (yaml["nav_relay"] && yaml["nav_relay"].as<bool>()) {
    auto rx_port = yaml["nav_relay_rx_port"] ? yaml["nav_relay_rx_port"].as<int>() : 8101;
    auto tx_port = yaml["nav_relay_tx_port"] ? yaml["nav_relay_tx_port"].as<int>() : 8102;
    try {
      nav_relay_ = std::make_unique<NavRelay>(rx_port, tx_port);
    } catch (const std::exception & e) {
      tools::logger()->error("{}", e.what());
      exit(1);
    }
    tools::logger()->info(
      "[Gimbal] nav relay enabled: nav -> MCU on udp 127.0.0.1:{}, MCU -> nav to udp 127.0.0.1:{}",
      rx_port, tx_port);
    nav_thread_ = std::thread(&Gimbal::nav_thread, this);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  if (nav_thread_.joinable()) nav_thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    // 逐字节滑动同步帧头, 步长为1.
    // 不能"一次读2字节当帧头"+失配丢2字节: 那样步长恒为偶数, 一旦相对真实帧边界
    // 错位奇数字节, 真帧头'S''P'会永远跨在两次读取之间而无法匹配; 而当payload里恰好
    // 出现0x53 0x50(偶数偏移)时, CRC失败后跳过的44字节又与真实帧周期同相位, 会把
    // 假帧头锁死成无限CRC fail. 逐字节滑动可在<=44字节内从任意错位/假帧头中恢复.
    uint8_t byte;
    if (!read(&byte, 1)) {
      error_count++;
      continue;
    }
    // 导航帧('U'开头)与自瞄包混在同一个字节流里: 整帧读出后原样转给导航程序.
    if (byte == NavRelay::HEAD && nav_relay_) {
      if (read_nav_frame()) error_count = 0;
      continue;
    }
    if (byte != 'S') continue;
    if (!read(&byte, 1)) {
      error_count++;
      continue;
    }
    if (byte != 'P') continue;

    rx_data_.head[0] = 'S';
    rx_data_.head[1] = 'P';

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;
    state_.self_color = rx_data_.self_color;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

// 帧头 'U' 已经读走. 读出剩余部分, 校验通过就转发. 校验失败(比如失步时把自瞄包里的
// 0x55 当成了帧头)直接丢弃, 外层循环继续逐字节找下一个帧头.
bool Gimbal::read_nav_frame()
{
  uint8_t frame[NavRelay::MAX_FRAME];
  frame[0] = NavRelay::HEAD;
  if (!read(frame + 1, 2)) return false;  // id, len
  size_t len = frame[2];
  if (len > NavRelay::MAX_PAYLOAD) return false;
  if (!read(frame + 3, len + 3)) return false;  // payload, crc16, 'N'
  size_t size = len + NavRelay::OVERHEAD;
  if (!NavRelay::valid(frame, size)) {
    tools::logger()->debug("[Gimbal] nav frame check failed.");
    return false;
  }
  nav_relay_->to_nav(frame, size);
  return true;
}

// 导航程序发来的帧校验后原样写串口. serial_.write 内部有写锁, 不会和自瞄自己的包交叉.
void Gimbal::nav_thread()
{
  tools::logger()->info("[Gimbal] nav_thread started.");
  uint8_t buffer[256];
  bool first = true;

  while (!quit_) {
    auto size = nav_relay_->from_nav(buffer, sizeof(buffer), 100);
    if (size == 0) continue;
    if (!NavRelay::valid(buffer, size)) {
      tools::logger()->warn("[Gimbal] Invalid frame from nav ({} bytes), dropped.", size);
      continue;
    }
    try {
      serial_.write(buffer, size);
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to write nav frame: {}", e.what());
      continue;
    }
    if (first) {
      tools::logger()->info("[Gimbal] First nav frame forwarded to MCU.");
      first = false;
    }
  }

  tools::logger()->info("[Gimbal] nav_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io