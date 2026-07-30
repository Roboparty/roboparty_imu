#pragma once

extern "C" {
#include "mct7123_dec.h"
}

#include <memory>
#include <array>
#include <string>
#include <shared_mutex>

#include "imu_driver.hpp"
#include "protocol/can/socket_can.hpp"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD  (0.01745329f)
#endif

class Mct7123CanDriver : public IMUDriver {
   public:
    Mct7123CanDriver(uint16_t imu_id, const std::string& interface);
    ~Mct7123CanDriver();

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
    static constexpr size_t kFrameTypeCount = 3;

    std::string interface_;
    std::array<CanCbkId, kFrameTypeCount> callback_ids_;
    mutable std::shared_mutex imu_mutex_;
    std::shared_ptr<IMUSocketCAN> can_;

    imu_sensor_data_t sensor_data_;
};
