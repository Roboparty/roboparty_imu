/* SPDX-License-Identifier: GPL-3.0 */
/* Copyright (C) 2026 Luo1imasi */

/**
 * @file imu_sensor_data.h
 * @brief Unified IMU sensor data struct — C/C++ compatible.
 *        Single buffer all transports (serial, CAN, CANFD) write to.
 */

#ifndef IMU_SENSOR_DATA_H
#define IMU_SENSOR_DATA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  node_id;
    uint64_t hw_ts_us;
    float    acc_x, acc_y, acc_z;
    float    gyr_x, gyr_y, gyr_z;
    float    mag_x, mag_y, mag_z;
    float    quat_w, quat_x, quat_y, quat_z;
    float    roll, pitch, imu_yaw;
    float    incli_x, incli_y;
    float    temperature;
    float    pressure;
    uint8_t  utc_year, utc_month, utc_day;
    uint8_t  hours, minutes, seconds;
    uint16_t milliseconds;
    uint32_t timestamp_ms;
    double   ins_lat, ins_lon, ins_msl;
    float    undulation, diff_age_s;
    float    ins_vel_e, ins_vel_n, ins_vel_u, ins_speed;
    uint8_t  solq_pos, solq_heading, nv_pos, nv_heading, ins_status;
    uint8_t  running_status[6];
    uint8_t  cycle;
} imu_sensor_data_t;

#ifdef __cplusplus
}
#endif

#endif /* IMU_SENSOR_DATA_H */
