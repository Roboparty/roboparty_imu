#include "hipnuc_imu_driver.hpp"

HipnucIMUDriver::HipnucIMUDriver(uint16_t imu_id, const std::string& interface_type, const std::string& interface, const int baudrate)
    : IMUDriver(), interface_type_(interface_type), interface_(interface) {
    imu_id_ = imu_id;
    memset(&raw_, 0, sizeof(raw_));
    memset(&sensor_data_, 0, sizeof(sensor_data_));
    if (interface_type_ == "serial") {
        baudrate_ = baudrate;
        serial_ = IMUSerialPort::open(interface_, baudrate_);
        IMUSerialPort::SerialCbkFunc serial_callback = std::bind(&HipnucIMUDriver::serial_rx_cbk, this, std::placeholders::_1, std::placeholders::_2);
        serial_->set_serial_callback(serial_callback);
    } else if (interface_type_ == "can") {
        can_ = IMUSocketCAN::get_instance(interface_);
        CanCbkFunc can_callback = std::bind(&HipnucIMUDriver::can_rx_cbk, this, std::placeholders::_1);
        can_subscription_ =
            CanCallbackSubscription(can_, CAN_CALLBACK_WILDCARD, can_callback);
    } else {
        throw std::runtime_error("Hipnuc driver only support CAN and SERIAL interface");
    }
}

HipnucIMUDriver::~HipnucIMUDriver() {
    if (interface_type_ == "serial" && serial_) {
        serial_->close();
    }
}

void HipnucIMUDriver::can_rx_cbk(const canfd_frame& rx_frame) {
    /* Only process classic CAN frames (len <= 8).
     * CANFD frames on the same bus are handled by other drivers (e.g. MCT7123). */
    if (rx_frame.len > 8 ||
        !(rx_frame.can_id & HIPNUC_CAN_EFF_FLAG) ||
        hipnuc_can_extract_node_id(rx_frame.can_id) != imu_id_) {
        return;
    }

    hipnuc_can_frame_t frame;
    frame.can_id = rx_frame.can_id;
    frame.can_dlc = rx_frame.len;
    memcpy(frame.data, rx_frame.data, 8);

    std::unique_lock<std::shared_mutex> lock(imu_mutex_);
    
    int ret = hipnuc_j1939_parse_frame(&frame, &sensor_data_);
    if (ret == CAN_MSG_ACCEL) {
        sensor_data_.acc_x *= GRA_ACC;
        sensor_data_.acc_y *= GRA_ACC;
        sensor_data_.acc_z *= GRA_ACC;
        return;
    } else if (ret == CAN_MSG_GYRO) {
        sensor_data_.gyr_x *= DEG_TO_RAD;
        sensor_data_.gyr_y *= DEG_TO_RAD;
        sensor_data_.gyr_z *= DEG_TO_RAD;
        return;
    }
}

void HipnucIMUDriver::serial_rx_cbk(const uint8_t* data, size_t length) {
    std::unique_lock<std::shared_mutex> lock(imu_mutex_);
    
    for (size_t i = 0; i < length; i++) {
        if (hipnuc_input(&raw_, data[i])) {
            // Update sensor data from parsed raw data
            sensor_data_.quat_w = raw_.hi91.quat[0];
            sensor_data_.quat_x = raw_.hi91.quat[1];
            sensor_data_.quat_y = raw_.hi91.quat[2];
            sensor_data_.quat_z = raw_.hi91.quat[3];
            
            sensor_data_.gyr_x = raw_.hi91.gyr[0] * DEG_TO_RAD;
            sensor_data_.gyr_y = raw_.hi91.gyr[1] * DEG_TO_RAD;
            sensor_data_.gyr_z = raw_.hi91.gyr[2] * DEG_TO_RAD;
            
            sensor_data_.acc_x = raw_.hi91.acc[0] * GRA_ACC;
            sensor_data_.acc_y = raw_.hi91.acc[1] * GRA_ACC;
            sensor_data_.acc_z = raw_.hi91.acc[2] * GRA_ACC;

            sensor_data_.roll  = raw_.hi91.roll;
            sensor_data_.pitch = raw_.hi91.pitch;
            sensor_data_.imu_yaw = raw_.hi91.yaw;

            sensor_data_.mag_x = raw_.hi91.mag[0];
            sensor_data_.mag_y = raw_.hi91.mag[1];
            sensor_data_.mag_z = raw_.hi91.mag[2];

            sensor_data_.temperature = raw_.hi91.temp;
            sensor_data_.timestamp_ms = raw_.hi91.system_time;
        }
    }
}

std::vector<float> HipnucIMUDriver::get_ang_vel() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.gyr_x, sensor_data_.gyr_y, sensor_data_.gyr_z};
}

std::vector<float> HipnucIMUDriver::get_quat() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.quat_w, sensor_data_.quat_x, sensor_data_.quat_y, sensor_data_.quat_z};
}

std::vector<float> HipnucIMUDriver::get_lin_acc() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.acc_x, sensor_data_.acc_y, sensor_data_.acc_z};
}

float HipnucIMUDriver::get_temperature() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return sensor_data_.temperature;
}

std::vector<float> HipnucIMUDriver::get_euler() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.roll, sensor_data_.pitch, sensor_data_.imu_yaw};
}

std::vector<float> HipnucIMUDriver::get_mag() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sensor_data_.mag_x, sensor_data_.mag_y, sensor_data_.mag_z};
}

uint64_t HipnucIMUDriver::get_timestamp() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return (uint64_t)sensor_data_.timestamp_ms * 1000ULL;
}

uint8_t HipnucIMUDriver::get_cycle() {
    return 0;  /* HiPNUC protocol has no cycle counter */
}
