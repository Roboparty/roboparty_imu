#pragma once

extern "C" {
#include "mct7123_dec.h"
}

#include <memory>
#include <string>
#include <shared_mutex>

#include "imu_driver.hpp"
#include "protocol/can/socket_can.hpp"
#include "protocol/serial/serial_port.hpp"
#include "drivers/hipnuc/hipnuc_can_common.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD  (0.01745329f)
#endif

class Mct7123IMUDriver : public IMUDriver {
   public:
    Mct7123IMUDriver(uint16_t imu_id, const std::string& interface_type,
                     const std::string& interface, const int baudrate=0);
    ~Mct7123IMUDriver();

    void serial_rx_cbk(const uint8_t* data, size_t length);
    void can_rx_cbk(const canfd_frame& rx_frame);

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
    std::string interface_type_;
    std::string interface_;
    mutable std::shared_mutex imu_mutex_;
    std::shared_ptr<IMUSerialPort> serial_;
    std::shared_ptr<IMUSocketCAN> can_;

    mct7123_raw_t raw_;
    can_sensor_data_t sensor_data_;
};
