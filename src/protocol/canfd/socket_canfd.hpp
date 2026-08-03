// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

/**
 * @file socket_canfd.hpp
 * @brief Asynchronous Linux SocketCAN-FD transport for IMU drivers.
 */

#pragma once

#include <linux/can.h>

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

/**
 * @brief Owns one CAN-FD socket and delivers received frames to one driver.
 *
 * The transport manages only Linux socket resources and receive scheduling.
 * Device-specific frame validation and decoding remain in the IMU driver.
 */
class IMUSocketCANFD {
 public:
  using CanFdCbkFunc = std::function<void(const canfd_frame&)>;

  IMUSocketCANFD(const IMUSocketCANFD&) = delete;
  IMUSocketCANFD& operator=(const IMUSocketCANFD&) = delete;
  ~IMUSocketCANFD();

  /**
   * @brief Open a CAN-FD interface and start asynchronous reception.
   * @param[in] interface Linux SocketCAN interface name, for example `can0`.
   * @param[in] callback Function invoked for every complete CAN-FD frame.
   * @return Owning shared pointer to the opened transport.
   * @throws std::runtime_error If the socket cannot be configured or opened.
   */
  static std::shared_ptr<IMUSocketCANFD> open(
      const std::string& interface, CanFdCbkFunc callback);

  /**
   * @brief Stop reception and close the CAN-FD socket.
   * @note This operation is idempotent and waits for the receive thread.
   */
  void close();

 private:
  IMUSocketCANFD(const std::string& interface, CanFdCbkFunc callback);

  void init();
  void receive_loop();

  std::string interface_;
  CanFdCbkFunc callback_;
  int socket_fd_{-1};
  std::atomic<bool> receiving_{false};
  std::thread receiver_thread_;
};
