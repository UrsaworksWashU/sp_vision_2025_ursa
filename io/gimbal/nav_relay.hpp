#ifndef IO__NAV_RELAY_HPP
#define IO__NAV_RELAY_HPP

#include <cstddef>
#include <cstdint>

namespace io
{
// 导航帧转发: 下位机和导航程序之间的帧经由自瞄占用的同一个串口收发, 自瞄不解析内容.
//
// 导航帧格式(下位机定义, 与自瞄的 'S' 'P' 包混在同一个字节流里):
//   'U' | id(u8) | len(u8) | payload[len] | crc16(u16, 小端) | 'N'
//   len <= 32; crc16 与自瞄相同(tools::get_crc16), 校验范围从 'U' 到 payload 末尾.
//
// 与导航程序之间走本机 UDP, 一个数据报就是一整帧:
//   下位机 -> 导航: 串口读到的导航帧原样发到 127.0.0.1:tx_port
//   导航 -> 下位机: 监听 127.0.0.1:rx_port, 收到的帧校验后原样写串口
class NavRelay
{
public:
  static constexpr uint8_t HEAD = 'U';
  static constexpr uint8_t TAIL = 'N';
  static constexpr size_t MAX_PAYLOAD = 32;
  static constexpr size_t OVERHEAD = 6;  // 'U', id, len, crc16, 'N'
  static constexpr size_t MAX_FRAME = MAX_PAYLOAD + OVERHEAD;

  NavRelay(int rx_port, int tx_port);
  ~NavRelay();

  NavRelay(const NavRelay &) = delete;
  NavRelay & operator=(const NavRelay &) = delete;

  // size 个字节是否恰好是一帧完整且校验正确的导航帧
  static bool valid(const uint8_t * frame, size_t size);

  // 把一帧发给导航程序. 导航程序没在运行时数据报被丢弃, 不报错.
  void to_nav(const uint8_t * frame, size_t size);

  // 等待导航程序发来的一个数据报, 最多 timeout_ms. 返回字节数, 超时或出错返回 0.
  size_t from_nav(uint8_t * buffer, size_t capacity, int timeout_ms);

private:
  int rx_fd_ = -1;
  int tx_fd_ = -1;
  int tx_port_;
};

}  // namespace io

#endif  // IO__NAV_RELAY_HPP
