// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

#pragma once

extern "C" {
#include "mct7123_dec.h"
}

#include <memory>
#include <shared_mutex>
#include <string>

#include "imu_driver.hpp"
#include "protocol/canfd/socket_canfd.hpp"
#include "protocol/serial/serial_port.hpp"

/**
 * @brief MCT7123 driver supporting serial and dedicated Linux CAN FD input.
 *
 * Both transports feed the same payload parser and sensor-data cache. Public
 * getters are thread-safe.
 */
class Mct7123IMUDriver : public IMUDriver {
 public:
  /**
   * @brief Open an MCT7123 transport and start receiving data.
   * @param[in] imu_id Low seven bits of the first MCT7123 CAN data-frame ID.
   * @param[in] interface_type Transport name: `serial` or `canfd`.
   * @param[in] interface Serial device path or Linux SocketCAN interface.
   * @param[in] baudrate Serial baud rate; ignored for CAN FD.
   * @throws std::runtime_error If the transport is unsupported or cannot open.
   */
  Mct7123IMUDriver(uint16_t imu_id, const std::string& interface_type,
                   const std::string& interface, int baudrate = 0);

  /** @brief Stop reception and release transport resources. */
  ~Mct7123IMUDriver() override;

  std::vector<float> get_ang_vel() override;
  std::vector<float> get_quat() override;
  std::vector<float> get_lin_acc() override;
  float get_temperature() override;

 private:
  void open_serial();
  void open_canfd();
  void close_transport();
  void serial_rx_cbk(const uint8_t* data, size_t length);
  void can_rx_cbk(const canfd_frame& frame);
  void parse_payload(uint8_t message_id, const uint8_t payload[64]);

  struct SensorData {
    float quat_w{};
    float quat_x{};
    float quat_y{};
    float quat_z{};
    float gyr_x{};
    float gyr_y{};
    float gyr_z{};
    float acc_x{};
    float acc_y{};
    float acc_z{};
    float mag_x{};
    float mag_y{};
    float mag_z{};
    float roll{};
    float pitch{};
    float yaw{};
    float temperature{};
    uint64_t timestamp_us{};
    uint8_t running_status[6]{};
    uint8_t cycle{};
  };

  int baudrate_{};
  std::string interface_type_;
  std::string interface_;
  mutable std::shared_mutex imu_mutex_;
  std::shared_ptr<IMUSerialPort> serial_;
  std::shared_ptr<IMUSocketCANFD> canfd_;
  mct7123_raw_t raw_{};
  SensorData sensor_data_{};
};
