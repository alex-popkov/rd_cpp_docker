#include <sys/socket.h>
#include <arpa/inet.h>  // inet_pton, htons
#include <fcntl.h>      // fcntl, O_NONBLOCK
#include <unistd.h>     // close
#include <stdexcept>
#include "port_controllers/udp_port.hpp"

UDPPort::UDPPort(const std::string& host, uint16_t port)
{
  this->fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (this->fd < 0) {
    throw std::runtime_error("Cannot create UDP socket\n");
  }

  this->dest.sin_family = AF_INET;
  this->dest.sin_port = htons(port);
  if (inet_pton(AF_INET, host.c_str(), &this->dest.sin_addr) != 1) {
    close(this->fd);
    throw std::runtime_error("Bad UDP host: " + host + "\n");
  }

  // Неблокуючий режим — recvfrom() не вішає головний цикл.
  int flags = fcntl(this->fd, F_GETFL, 0);
  fcntl(this->fd, F_SETFL, flags | O_NONBLOCK);
}

UdpPort::~UdpPort()
{
  if (this->fd >= 0) {
    close(this->fd);
  }
}

auto UDPPort::send(const uint8_t* buf, size_t byteCount) -> void
{
  ::sendto(this->fd, buf, byteCount, 0, (const sockaddr*)&this->dest, sizeof(this->dest));
}

auto UDPPort::recv(uint8_t* buf, size_t maxByteCount) -> int
{
  return ::recvfrom(this->fd, buf, maxByteCount, 0, nullptr, nullptr);
}