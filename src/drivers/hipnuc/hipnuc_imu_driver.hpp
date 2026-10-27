#pragma once

extern "C" {
#include "hipnuc_dec.h"
// #include "nmea_decode.h"
#include "hipnuc_sample.h"
// #include "canopen_parser.h"
}

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <shared_mutex>
#include <algorithm>

#include "imu_driver.hpp"
#include "protocol/can/socket_can.hpp"
#include "protocol/canfd/socket_canfd.hpp"
#include "protocol/serial/serial_port.hpp"

class HipnucIMUDriver : public IMUDriver {
   public:
    HipnucIMUDriver(uint16_t imu_id, const std::string& interface_type, const std::string& interface, const int baudrate=0);
    ~HipnucIMUDriver();

    void can_rx_cbk(const can_frame& rx_frame);
    void canfd_rx_cbk(const canfd_frame& rx_frame);
    void serial_rx_cbk(const uint8_t* data, size_t length);
    std::vector<float> get_ang_vel() override;
    std::vector<float> get_quat() override;
    std::vector<float> get_lin_acc() override;
    float get_temperature() override;
    hipnuc_sample_t get_sample() const;

   private:
    int baudrate_;
    std::string interface_type_;
    std::string interface_;
    mutable std::shared_mutex imu_mutex_;
    std::shared_ptr<IMUSocketCAN> can_;
    std::shared_ptr<IMUSocketCANFD> canfd_;
    std::shared_ptr<IMUSerialPort> serial_;
    hipnuc_sample_t sample_;
    hipnuc_raw_t raw_;
};
