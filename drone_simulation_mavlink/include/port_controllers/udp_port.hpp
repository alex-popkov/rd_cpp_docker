#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <netinet/in.h>

class UdpPort {
public:
  explicit UdpPort(const std::string& host, uint16_t port);
  ~UdpPort();

  auto send(const uint8_t* buf, size_t byteCount) -> void;
  auto recv(uint8_t* buf, size_t max) -> int;

private:
  int fd;
  sockaddr_in dest{};
};