#include "mct7123_imu_driver.hpp"
#include <cstdint>
#include <cstring>

Mct7123IMUDriver::Mct7123IMUDriver(uint16_t imu_id, const std::string& interface_type,
                                   const std::string& interface, const int baudrate)
    : IMUDriver(), interface_type_(interface_type), interface_(interface)
{
    imu_id_ = imu_id;
    memset(&raw_, 0, sizeof(raw_));
    memset(&sensor_data_, 0, sizeof(sensor_data_));

    if (interface_type_ == "serial") {
        baudrate_ = baudrate;
        serial_ = IMUSerialPort::open(interface_, baudrate_);
        IMUSerialPort::SerialCbkFunc cb = std::bind(
            &Mct7123IMUDriver::serial_rx_cbk, this,
            std::placeholders::_1, std::placeholders::_2);
        serial_->set_serial_callback(cb);
    } else if (interface_type_ == "canfd") {
        can_ = IMUSocketCAN::get_instance(interface_);
        CanCbkFunc can_cb = std::bind(&Mct7123IMUDriver::can_rx_cbk, this, std::placeholders::_1);
        can_->add_can_callback(can_cb, imu_id_);
        can_->set_key_extractor([](const canfd_frame &frame) -> CanCbkId {
            return frame.can_id & 0x7F;
        });
    } else {
        throw std::runtime_error("MCT7123 driver supports SERIAL and CANFD interfaces");
    }
}

Mct7123IMUDriver::~Mct7123IMUDriver()
{
    if (serial_) serial_->close();
    if (can_) can_->remove_can_callback(imu_id_);
}

void Mct7123IMUDriver::serial_rx_cbk(const uint8_t* data, size_t length)
{
    std::unique_lock<std::shared_mutex> lock(imu_mutex_);

    for (size_t i = 0; i < length; i++) {
        if (mct7123_input(&raw_, data[i]) == 1) {
            switch (raw_.msg_id) {
            case MCT7123_MSG_IMU_DATA: {
                float gx, gy, gz, ax, ay, az, mx, my, mz, t;
                uint64_t ts_us;
                mct7123_parse_imu(raw_.payload, &gx, &gy, &gz,
                                  &ax, &ay, &az, &mx, &my, &mz, &t, &ts_us);
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
                break;
            }
            case MCT7123_MSG_ATT_DATA: {
                float roll, pitch, yaw, qw, qx, qy, qz, t;
                uint32_t fusion_status;
                uint64_t ts_us;
                mct7123_parse_att(raw_.payload, &roll, &pitch, &yaw,
                                  &qw, &qx, &qy, &qz, &t,
                                  nullptr, &fusion_status, &ts_us);
                sensor_data_.quat_w = qw;
                sensor_data_.quat_x = qx;
                sensor_data_.quat_y = qy;
                sensor_data_.quat_z = qz;
                sensor_data_.roll  = roll;
                sensor_data_.pitch = pitch;
                sensor_data_.imu_yaw = yaw;
                sensor_data_.temperature = t;
                sensor_data_.timestamp_ms = (uint32_t)(ts_us / 1000ULL);
                break;
            }
            case MCT7123_MSG_CFG_DATA: {
                mct7123_parse_cfg(raw_.payload, nullptr, nullptr, nullptr, nullptr,
                                  nullptr, nullptr, nullptr);
                break;
            }
            default:
                break;
            }
        }
    }
}

void Mct7123IMUDriver::can_rx_cbk(const canfd_frame& rx_frame)
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
        mct7123_parse_imu(payload, &gx, &gy, &gz,
                          &ax, &ay, &az, &mx, &my, &mz, &t, &ts_us);
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
        break;
    }
    case 0x182: {
        float roll, pitch, yaw, qw, qx, qy, qz, t;
        uint32_t fusion_status;
        uint64_t ts_us;
        mct7123_parse_att(payload, &roll, &pitch, &yaw,
                          &qw, &qx, &qy, &qz, &t,
                          nullptr, &fusion_status, &ts_us);
        sensor_data_.quat_w = qw;
        sensor_data_.quat_x = qx;
        sensor_data_.quat_y = qy;
        sensor_data_.quat_z = qz;
        sensor_data_.roll  = roll;
        sensor_data_.pitch = pitch;
        sensor_data_.imu_yaw = yaw;
        sensor_data_.temperature = t;
        sensor_data_.timestamp_ms = (uint32_t)(ts_us / 1000ULL);
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

std::vector<float> Mct7123IMUDriver::get_ang_vel()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.gyr_x, sensor_data_.gyr_y, sensor_data_.gyr_z};
}

std::vector<float> Mct7123IMUDriver::get_quat()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.quat_w, sensor_data_.quat_x,
            sensor_data_.quat_y, sensor_data_.quat_z};
}

std::vector<float> Mct7123IMUDriver::get_lin_acc()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.acc_x, sensor_data_.acc_y, sensor_data_.acc_z};
}

std::vector<float> Mct7123IMUDriver::get_mag()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.mag_x, sensor_data_.mag_y, sensor_data_.mag_z};
}

std::vector<float> Mct7123IMUDriver::get_euler()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.roll, sensor_data_.pitch, sensor_data_.imu_yaw};
}

uint64_t Mct7123IMUDriver::get_timestamp()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return (uint64_t)sensor_data_.timestamp_ms * 1000ULL;
}

float Mct7123IMUDriver::get_temperature()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return sensor_data_.temperature;
}
