#include "mct7123_can_driver.hpp"
#include <cstring>

Mct7123CanDriver::Mct7123CanDriver(uint16_t imu_id, const std::string& interface)
    : IMUDriver(), interface_(interface)
{
    imu_id_ = imu_id;
    memset(&sensor_data_, 0, sizeof(sensor_data_));

    can_ = IMUSocketCAN::get_instance(interface_);
    CanCbkFunc can_cb = std::bind(&Mct7123CanDriver::can_rx_cbk, this, std::placeholders::_1);
    can_->add_can_callback(can_cb, imu_id_);
    can_->set_key_extractor([](const canfd_frame &frame) -> CanCbkId {
        return frame.can_id & 0x7F;
    });
}

Mct7123CanDriver::~Mct7123CanDriver()
{
    if (can_) can_->remove_can_callback(imu_id_);
}

void Mct7123CanDriver::can_rx_cbk(const canfd_frame& rx_frame)
{
    /* MD7123 CANFD: full 64-byte payload per frame, frame ID selects type.
     * 0x181 = IMU data, 0x182 = Attitude data, 0x183 = Config.
     * Payload layout identical to UART; CRC16 over bytes 0..61. */
    if (rx_frame.len < 64) return;

    const uint8_t *payload = rx_frame.data;

    /* Validate CRC16 over payload[0..61] */
    uint16_t crc_expected = (uint16_t)payload[62] | ((uint16_t)payload[63] << 8);
    if (mct7123_crc16(payload, 62) != crc_expected) return;

    std::unique_lock<std::shared_mutex> lock(imu_mutex_);

    uint32_t id = rx_frame.can_id & CAN_SFF_MASK;

    switch (id) {
    case 0x181: {
        float gx, gy, gz, ax, ay, az, mx, my, mz, t;
        uint64_t ts_us;
        uint8_t cyc;
        mct7123_parse_imu(payload, &gx, &gy, &gz,
                          &ax, &ay, &az, &mx, &my, &mz, &t, &ts_us, &cyc);
        sensor_data_.gyr_x = gx * DEG_TO_RAD;
        sensor_data_.gyr_y = gy * DEG_TO_RAD;
        sensor_data_.gyr_z = gz * DEG_TO_RAD;
        sensor_data_.acc_x = ax;
        sensor_data_.acc_y = ay;
        sensor_data_.acc_z = az;
        sensor_data_.mag_x = mx;
        sensor_data_.mag_y = my;
        sensor_data_.mag_z = mz;
        sensor_data_.temperature = t;
        sensor_data_.timestamp_ms = (uint32_t)(ts_us / 1000ULL);
        sensor_data_.cycle = cyc;
        break;
    }
    case 0x182: {
        float roll, pitch, yaw, qw, qx, qy, qz, t;
        uint32_t fusion_status;
        uint64_t ts_us;
        uint8_t cyc;
        mct7123_parse_att(payload, &roll, &pitch, &yaw,
                          &qw, &qx, &qy, &qz, &t,
                          sensor_data_.running_status, &fusion_status, &ts_us, &cyc);
        sensor_data_.quat_w = qw;
        sensor_data_.quat_x = qx;
        sensor_data_.quat_y = qy;
        sensor_data_.quat_z = qz;
        sensor_data_.roll  = roll;
        sensor_data_.pitch = pitch;
        sensor_data_.imu_yaw = yaw;
        sensor_data_.temperature = t;
        sensor_data_.timestamp_ms = (uint32_t)(ts_us / 1000ULL);
        sensor_data_.cycle = cyc;
        break;
    }
    case 0x183: {
        mct7123_parse_cfg(payload, nullptr, nullptr, nullptr, nullptr,
                          nullptr, nullptr, nullptr);
        break;
    }
    default:
        break;
    }
}

std::vector<float> Mct7123CanDriver::get_ang_vel()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.gyr_x, sensor_data_.gyr_y, sensor_data_.gyr_z};
}

std::vector<float> Mct7123CanDriver::get_quat()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.quat_w, sensor_data_.quat_x,
            sensor_data_.quat_y, sensor_data_.quat_z};
}

std::vector<float> Mct7123CanDriver::get_lin_acc()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.acc_x, sensor_data_.acc_y, sensor_data_.acc_z};
}

std::vector<float> Mct7123CanDriver::get_mag()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.mag_x, sensor_data_.mag_y, sensor_data_.mag_z};
}

std::vector<float> Mct7123CanDriver::get_euler()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.roll, sensor_data_.pitch, sensor_data_.imu_yaw};
}

uint64_t Mct7123CanDriver::get_timestamp()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return (uint64_t)sensor_data_.timestamp_ms * 1000ULL;
}

float Mct7123CanDriver::get_temperature()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return sensor_data_.temperature;
}

uint8_t Mct7123CanDriver::get_cycle()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return sensor_data_.cycle;
}
