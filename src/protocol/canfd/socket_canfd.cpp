// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

#include "socket_canfd.hpp"

#include <fcntl.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

IMUSocketCANFD::IMUSocketCANFD(const std::string& interface,
                               CanFdCbkFunc callback)
    : interface_(interface), callback_(std::move(callback)) {
  init();
}

IMUSocketCANFD::~IMUSocketCANFD() { close(); }

std::shared_ptr<IMUSocketCANFD> IMUSocketCANFD::open(
    const std::string& interface, CanFdCbkFunc callback) {
  return std::shared_ptr<IMUSocketCANFD>(
      new IMUSocketCANFD(interface, std::move(callback)));
}

void IMUSocketCANFD::init() {
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    throw std::runtime_error("Failed to create CAN FD socket");
  }

  int enable_canfd = 1;
  if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                 sizeof(enable_canfd)) < 0) {
    close();
    throw std::runtime_error("Failed to enable CAN FD frames");
  }

  ifreq request{};
  std::strncpy(request.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0) {
    close();
    throw std::runtime_error("CAN FD interface not found: " + interface_);
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&address),
           sizeof(address)) < 0) {
    close();
    throw std::runtime_error("Failed to bind CAN FD interface: " + interface_);
  }

  const int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    close();
    throw std::runtime_error("Failed to set CAN FD socket non-blocking");
  }

  receiving_ = true;
  try {
    receiver_thread_ = std::thread(&IMUSocketCANFD::receive_loop, this);
  } catch (...) {
    receiving_ = false;
    close();
    throw;
  }
}

void IMUSocketCANFD::close() {
  receiving_ = false;
  if (receiver_thread_.joinable()) receiver_thread_.join();
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

void IMUSocketCANFD::receive_loop() {
  while (receiving_) {
    fd_set descriptors;
    FD_ZERO(&descriptors);
    FD_SET(socket_fd_, &descriptors);

    timeval timeout{};
    timeout.tv_usec = 100000;
    const int ready = select(socket_fd_ + 1, &descriptors, nullptr, nullptr,
                             &timeout);
    if (ready < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (ready == 0 || !receiving_) continue;

    while (receiving_) {
      canfd_frame frame{};
      const ssize_t length = read(socket_fd_, &frame, sizeof(frame));
      if (length < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (length < 0) return;
      if (length == sizeof(frame) && callback_) callback_(frame);
    }
  }
}
