#pragma once

extern "C" {
#include "mct7123_dec.h"
}

#include <memory>
#include <string>
#include <shared_mutex>

#include "imu_driver.hpp"
#include "protocol/serial/serial_port.hpp"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD  (0.01745329f)
#endif

class Mct7123SerialDriver : public IMUDriver {
   public:
    Mct7123SerialDriver(uint16_t imu_id, const std::string& interface, int baudrate);
    ~Mct7123SerialDriver();

    void serial_rx_cbk(const uint8_t* data, size_t length);

    std::vector<float> get_ang_vel() override;
    std::vector<float> get_quat() override;
    std::vector<float> get_lin_acc() override;
    std::vector<float> get_mag() override;
    std::vector<float> get_euler() override;
    uint64_t get_timestamp() override;
    float get_temperature() override;
    uint8_t get_cycle() override;

   private:
    int baudrate_;
    std::string interface_;
    mutable std::shared_mutex imu_mutex_;
    std::shared_ptr<IMUSerialPort> serial_;

    mct7123_raw_t raw_;
    imu_sensor_data_t sensor_data_;
};
