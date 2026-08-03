#include "mct7123_serial_driver.hpp"
#include <cstring>

Mct7123SerialDriver::Mct7123SerialDriver(uint16_t imu_id, const std::string& interface, int baudrate)
    : IMUDriver(), baudrate_(baudrate), interface_(interface)
{
    imu_id_ = imu_id;
    memset(&raw_, 0, sizeof(raw_));
    memset(&sensor_data_, 0, sizeof(sensor_data_));

    serial_ = IMUSerialPort::open(interface_, baudrate_);
    IMUSerialPort::SerialCbkFunc cb = std::bind(
        &Mct7123SerialDriver::serial_rx_cbk, this,
        std::placeholders::_1, std::placeholders::_2);
    serial_->set_serial_callback(cb);
}

Mct7123SerialDriver::~Mct7123SerialDriver()
{
    if (serial_) serial_->close();
}

void Mct7123SerialDriver::serial_rx_cbk(const uint8_t* data, size_t length)
{
    std::unique_lock<std::shared_mutex> lock(imu_mutex_);

    for (size_t i = 0; i < length; i++) {
        if (mct7123_input(&raw_, data[i]) == 1) {
            switch (raw_.msg_id) {
            case MCT7123_MSG_IMU_DATA: {
                float gx, gy, gz, ax, ay, az, mx, my, mz, t;
                uint64_t ts_us;
                uint8_t cyc;
                mct7123_parse_imu(raw_.payload, &gx, &gy, &gz,
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
            case MCT7123_MSG_ATT_DATA: {
                float roll, pitch, yaw, qw, qx, qy, qz, t;
                uint32_t fusion_status;
                uint64_t ts_us;
                uint8_t cyc;
                mct7123_parse_att(raw_.payload, &roll, &pitch, &yaw,
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

std::vector<float> Mct7123SerialDriver::get_ang_vel()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.gyr_x, sensor_data_.gyr_y, sensor_data_.gyr_z};
}

std::vector<float> Mct7123SerialDriver::get_quat()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.quat_w, sensor_data_.quat_x,
            sensor_data_.quat_y, sensor_data_.quat_z};
}

std::vector<float> Mct7123SerialDriver::get_lin_acc()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.acc_x, sensor_data_.acc_y, sensor_data_.acc_z};
}

std::vector<float> Mct7123SerialDriver::get_mag()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.mag_x, sensor_data_.mag_y, sensor_data_.mag_z};
}

std::vector<float> Mct7123SerialDriver::get_euler()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.roll, sensor_data_.pitch, sensor_data_.imu_yaw};
}

uint64_t Mct7123SerialDriver::get_timestamp()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return (uint64_t)sensor_data_.timestamp_ms * 1000ULL;
}

float Mct7123SerialDriver::get_temperature()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return sensor_data_.temperature;
}

uint8_t Mct7123SerialDriver::get_cycle()
{
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return sensor_data_.cycle;
}
