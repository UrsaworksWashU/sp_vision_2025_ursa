#include "nav_relay.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include "tools/crc.hpp"

namespace io
{
namespace
{
sockaddr_in loopback(int port)
{
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(static_cast<uint16_t>(port));
  return addr;
}
}  // namespace

NavRelay::NavRelay(int rx_port, int tx_port) : tx_port_(tx_port)
{
  rx_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  tx_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (rx_fd_ < 0 || tx_fd_ < 0)
    throw std::runtime_error(std::string("[NavRelay] socket: ") + std::strerror(errno));

  int reuse = 1;
  setsockopt(rx_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  auto addr = loopback(rx_port);
  if (bind(rx_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    throw std::runtime_error(
      "[NavRelay] bind 127.0.0.1:" + std::to_string(rx_port) + ": " + std::strerror(errno));
}

NavRelay::~NavRelay()
{
  if (rx_fd_ >= 0) close(rx_fd_);
  if (tx_fd_ >= 0) close(tx_fd_);
}

bool NavRelay::valid(const uint8_t * frame, size_t size)
{
  if (size < OVERHEAD || size > MAX_FRAME) return false;
  size_t len = frame[2];
  if (frame[0] != HEAD || len > MAX_PAYLOAD || size != len + OVERHEAD) return false;
  if (frame[size - 1] != TAIL) return false;
  uint16_t crc = static_cast<uint16_t>(frame[3 + len] | (frame[4 + len] << 8));
  return tools::get_crc16(frame, static_cast<uint32_t>(3 + len)) == crc;
}

void NavRelay::to_nav(const uint8_t * frame, size_t size)
{
  auto addr = loopback(tx_port_);
  sendto(tx_fd_, frame, size, MSG_DONTWAIT, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
}

size_t NavRelay::from_nav(uint8_t * buffer, size_t capacity, int timeout_ms)
{
  pollfd pfd{rx_fd_, POLLIN, 0};
  if (poll(&pfd, 1, timeout_ms) <= 0) return 0;
  auto n = recv(rx_fd_, buffer, capacity, MSG_DONTWAIT);
  return n > 0 ? static_cast<size_t>(n) : 0;
}

}  // namespace io
