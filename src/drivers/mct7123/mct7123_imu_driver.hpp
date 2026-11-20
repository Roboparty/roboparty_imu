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

  /** @brief Configuration reported by message 0x83 (dbc.txt 6.2.4). */
  struct ConfigData {
    uint8_t sentence_ctrl{};
    uint8_t output_rate{};
    uint8_t orientation{};
    uint8_t fusion_mode{};
    double latitude{};
    double longitude{};
    float height{};
    bool valid{false};
  };

  std::vector<float> get_ang_vel() override;
  std::vector<float> get_quat() override;
  std::vector<float> get_lin_acc() override;
  float get_temperature() override;

  /** @brief Magnetic field {x, y, z} as reported by the device. */
  std::vector<float> get_mag();
  /** @brief Euler angles {roll, pitch, yaw}, FRD->FLU converted. */
  std::vector<float> get_euler();
  /** @brief Device timestamp in milliseconds (systimer; wire unit is ms). */
  uint64_t get_timestamp_ms();
  /** @brief Rolling cycle counter 0-255. */
  uint8_t get_cycle();
  /** @brief Six running-status bytes (0x81 offset 48 / 0x82 offset 42). */
  std::vector<uint8_t> get_running_status();
  /** @brief Fusion status word (0x82 offset 38). */
  uint32_t get_fusion_status();
  /** @brief Last configuration reported by message 0x83. */
  ConfigData get_config();

  /**
   * @brief Mirror every received CAN FD frame out of a serial port as a UART
   *        frame (0xEB 0x90 | msg_id | 0x40 | payload | 0xED).
   * @param[in] device Serial device path to transmit on.
   * @param[in] baudrate Serial baud rate (same set as IMUSerialPort).
   */
  void enable_uart_forward(const std::string& device, int baudrate);

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
    uint32_t fusion_status{};
    ConfigData config{};
  };

  int baudrate_{};
  std::string interface_type_;
  std::string interface_;
  mutable std::shared_mutex imu_mutex_;
  std::shared_ptr<IMUSerialPort> serial_;
  std::shared_ptr<IMUSerialPort> forward_serial_;
  std::shared_ptr<IMUSocketCANFD> canfd_;
  mct7123_raw_t raw_{};
  SensorData sensor_data_{};
};
