// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

#include "socket_canfd.hpp"

#include <fcntl.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

std::shared_ptr<spdlog::logger> IMUSocketCANFD::logger_ = nullptr;

IMUSocketCANFD::IMUSocketCANFD(const std::string& interface,
                               CanFdCbkFunc callback)
    : interface_(interface), callback_(std::move(callback)) {
  init();
}

IMUSocketCANFD::~IMUSocketCANFD() { close(); }

std::shared_ptr<IMUSocketCANFD> IMUSocketCANFD::open(
    const std::string& interface, CanFdCbkFunc callback) {
  if (!logger_) {
    logger_ = spdlog::get("imu");
    if (!logger_) {
      std::vector<spdlog::sink_ptr> sinks;
      sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
      logger_ = std::make_shared<spdlog::logger>(
          "imu", std::begin(sinks), std::end(sinks));
      spdlog::register_logger(logger_);
    }
  }
  return std::shared_ptr<IMUSocketCANFD>(
      new IMUSocketCANFD(interface, std::move(callback)));
}

void IMUSocketCANFD::init() {
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    logger_->error("Failed to create CAN FD socket");
    throw std::runtime_error("Failed to create CAN FD socket");
  }

  int enable_canfd = 1;
  if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                 sizeof(enable_canfd)) < 0) {
    logger_->error("Failed to enable CAN FD support");
    close();
    throw std::runtime_error("Failed to enable CAN FD frames");
  }

  ifreq request{};
  std::strncpy(request.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0) {
    logger_->error("Unable to detect CAN FD interface {}", interface_);
    close();
    throw std::runtime_error("CAN FD interface not found: " + interface_);
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&address),
           sizeof(address)) < 0) {
    logger_->error("Failed to bind socket to network interface {}", interface_);
    close();
    throw std::runtime_error("Failed to bind CAN FD interface: " + interface_);
  }

  const int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0) {
    logger_->error("Failed to get socket flags");
    close();
    throw std::runtime_error("Failed to get CAN FD socket flags");
  }
  if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    logger_->error("Failed to set socket to non-blocking");
    close();
    throw std::runtime_error("Failed to set CAN FD socket non-blocking");
  }

  receiving_ = true;
  try {
    receiver_thread_ = std::thread([this]() {
      pthread_setname_np(pthread_self(), "canfd_rx");
      struct sched_param sp{};
      sp.sched_priority = 48;
      if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        logger_->error(
            "Failed to set realtime priority for IMU CAN FD RX thread");
      }
      receive_loop();
    });
  } catch (...) {
    receiving_ = false;
    logger_->error("Failed to start CAN FD RX thread");
    close();
    throw;
  }
}

void IMUSocketCANFD::close() {
  receiving_ = false;
  if (receiver_thread_.joinable()) receiver_thread_.join();

  if (socket_fd_ != -1) {
    if (::close(socket_fd_) < 0) {
      logger_->warn("Failed to close socket {}: {}", interface_,
                    std::strerror(errno));
    } else {
      logger_->info("CAN FD interface {} closed successfully.", interface_);
    }
  }
  socket_fd_ = -1;
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
      logger_->error("CAN FD select error: {}", std::strerror(errno));
      break;
    }
    if (ready == 0 || !receiving_) continue;

    while (receiving_) {
      canfd_frame frame{};
      const ssize_t length = read(socket_fd_, &frame, sizeof(frame));
      if (length < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (length < 0) {
        logger_->warn("CAN FD read error: {}", std::strerror(errno));
        return;
      }
      if (length == 0) break;
      if (length == sizeof(frame) && callback_) callback_(frame);
    }
  }
}
