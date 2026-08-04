// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

#include "mct7123_imu_driver.hpp"

#include <linux/can.h>

#include <stdexcept>

namespace {
constexpr float kDegreesToRadians = 0.01745329F;
}

Mct7123IMUDriver::Mct7123IMUDriver(uint16_t imu_id,
                                   const std::string& interface_type,
                                   const std::string& interface, int baudrate)
    : baudrate_(baudrate),
      interface_type_(interface_type),
      interface_(interface) {
  imu_id_ = imu_id;
  if (interface_type_ == "serial") {
    open_serial();
  } else if (interface_type_ == "canfd") {
    open_canfd();
  } else {
    throw std::runtime_error(
        "MCT7123 driver only supports serial and canfd interfaces");
  }
}

Mct7123IMUDriver::~Mct7123IMUDriver() { close_transport(); }

void Mct7123IMUDriver::open_serial() {
  serial_ = IMUSerialPort::open(interface_, baudrate_);
  serial_->set_serial_callback(
      std::bind(&Mct7123IMUDriver::serial_rx_cbk, this,
                std::placeholders::_1, std::placeholders::_2));
}

void Mct7123IMUDriver::open_canfd() {
  canfd_ = IMUSocketCANFD::open(
      interface_, std::bind(&Mct7123IMUDriver::can_rx_cbk, this,
                            std::placeholders::_1));
}

void Mct7123IMUDriver::close_transport() {
  if (serial_) serial_->close();
  if (canfd_) canfd_->close();
}

void Mct7123IMUDriver::serial_rx_cbk(const uint8_t* data, size_t length) {
  for (size_t index = 0; index < length; ++index) {
    if (mct7123_input(&raw_, data[index]) == 1) {
      parse_payload(raw_.msg_id, raw_.payload);
    }
  }
}

void Mct7123IMUDriver::can_rx_cbk(const canfd_frame& frame) {
  if (frame.len != MCT7123_PAYLOAD_LEN) return;

  const uint16_t expected_crc = static_cast<uint16_t>(frame.data[62]) |
                                (static_cast<uint16_t>(frame.data[63]) << 8U);
  if (mct7123_crc16(frame.data, 62) != expected_crc) return;

  const uint32_t frame_id = frame.can_id & CAN_SFF_MASK;
  const uint32_t first_frame_id = 0x180U + imu_id_;
  if (frame_id < first_frame_id || frame_id >= first_frame_id + 3U) return;

  const auto message_id = static_cast<uint8_t>(
      MCT7123_MSG_IMU_DATA + frame_id - first_frame_id);
  parse_payload(message_id, frame.data);
}

void Mct7123IMUDriver::parse_payload(uint8_t message_id,
                                     const uint8_t payload[64]) {
  std::unique_lock<std::shared_mutex> lock(imu_mutex_);

  if (message_id == MCT7123_MSG_IMU_DATA) {
    float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
    float mag_x, mag_y, mag_z, temperature;
    uint64_t timestamp_us;
    uint8_t cycle;
    mct7123_parse_imu(payload, &gyro_x, &gyro_y, &gyro_z, &acc_x, &acc_y,
                      &acc_z, &mag_x, &mag_y, &mag_z, &temperature,
                      &timestamp_us, &cycle);
    sensor_data_.gyr_x = gyro_x * kDegreesToRadians;
    sensor_data_.gyr_y = gyro_y * kDegreesToRadians;
    sensor_data_.gyr_z = gyro_z * kDegreesToRadians;
    sensor_data_.acc_x = acc_x;
    sensor_data_.acc_y = acc_y;
    sensor_data_.acc_z = acc_z;
    sensor_data_.mag_x = mag_x;
    sensor_data_.mag_y = mag_y;
    sensor_data_.mag_z = mag_z;
    sensor_data_.temperature = temperature;
    sensor_data_.timestamp_us = timestamp_us;
    sensor_data_.cycle = cycle;
  } else if (message_id == MCT7123_MSG_ATT_DATA) {
    float roll, pitch, yaw, quat_w, quat_x, quat_y, quat_z, temperature;
    uint32_t fusion_status;
    uint64_t timestamp_us;
    uint8_t cycle;
    mct7123_parse_att(payload, &roll, &pitch, &yaw, &quat_w, &quat_x, &quat_y,
                      &quat_z, &temperature, sensor_data_.running_status,
                      &fusion_status, &timestamp_us, &cycle);
    sensor_data_.quat_w = quat_w;
    sensor_data_.quat_x = quat_x;
    sensor_data_.quat_y = quat_y;
    sensor_data_.quat_z = quat_z;
    sensor_data_.roll = roll;
    sensor_data_.pitch = pitch;
    sensor_data_.yaw = yaw;
    sensor_data_.temperature = temperature;
    sensor_data_.timestamp_us = timestamp_us;
    sensor_data_.cycle = cycle;
  } else if (message_id == MCT7123_MSG_CFG_DATA) {
    mct7123_parse_cfg(payload, nullptr, nullptr, nullptr, nullptr, nullptr,
                      nullptr, nullptr);
  }
}

std::vector<float> Mct7123IMUDriver::get_ang_vel() {
  std::shared_lock<std::shared_mutex> lock(imu_mutex_);
  return {sensor_data_.gyr_x, sensor_data_.gyr_y, sensor_data_.gyr_z};
}

std::vector<float> Mct7123IMUDriver::get_quat() {
  std::shared_lock<std::shared_mutex> lock(imu_mutex_);
  return {sensor_data_.quat_w, sensor_data_.quat_x, sensor_data_.quat_y,
          sensor_data_.quat_z};
}

std::vector<float> Mct7123IMUDriver::get_lin_acc() {
  std::shared_lock<std::shared_mutex> lock(imu_mutex_);
  return {sensor_data_.acc_x, sensor_data_.acc_y, sensor_data_.acc_z};
}

float Mct7123IMUDriver::get_temperature() {
  std::shared_lock<std::shared_mutex> lock(imu_mutex_);
  return sensor_data_.temperature;
}
