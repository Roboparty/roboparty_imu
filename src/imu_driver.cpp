// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

/**
 * @file imu_driver.cpp
 * @brief Factory implementation for creating IMU driver instances.
 * @details Provides IMUDriver::create_imu() to instantiate the appropriate
 *          driver backend based on configuration (e.g., HiPNUC).
 */

#include "imu_driver.hpp"

#include "drivers/hipnuc/hipnuc_imu_driver.hpp"
#include "drivers/mct7123/mct7123_serial_driver.hpp"
#include "drivers/mct7123/mct7123_can_driver.hpp"

std::shared_ptr<IMUDriver> IMUDriver::create_imu(uint16_t imu_id, const std::string& interface_type, const std::string& interface,
                                                const std::string& imu_type, const int baudrate) {
    if (imu_type == "HIPNUC") {
        return std::make_shared<HipnucIMUDriver>(imu_id, interface_type, interface, baudrate);
    } else if (imu_type == "MCT7123") {
        if (interface_type == "serial") {
            return std::make_shared<Mct7123SerialDriver>(imu_id, interface, baudrate);
        } else if (interface_type == "canfd") {
            return std::make_shared<Mct7123CanDriver>(imu_id, interface);
        } else {
            throw std::runtime_error("MCT7123: interface_type must be 'serial' or 'canfd'");
        }
    } else {
        throw std::runtime_error("IMU type not supported");
    }
}
