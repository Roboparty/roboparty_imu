#include "hipnuc_imu_driver.hpp"
#include "hipnuc_j1939.h"

namespace {
void merge_sample(hipnuc_sample_t& dst, const hipnuc_sample_t& src) {
    const uint64_t v = src.valid;
    dst.source = src.source;
    dst.node_id = src.node_id;
    dst.main_status = src.main_status;
    dst.ins_status = src.ins_status;
    dst.valid |= v;
    if (v & HIPNUC_VALID_ACC) std::copy(std::begin(src.acc), std::end(src.acc), std::begin(dst.acc));
    if (v & HIPNUC_VALID_GYR) std::copy(std::begin(src.gyr), std::end(src.gyr), std::begin(dst.gyr));
    if (v & HIPNUC_VALID_MAG) std::copy(std::begin(src.mag), std::end(src.mag), std::begin(dst.mag));
    if (v & HIPNUC_VALID_ROLL_PITCH) { dst.roll = src.roll; dst.pitch = src.pitch; }
    if (v & HIPNUC_VALID_HEADING) dst.heading = src.heading;
    if (v & HIPNUC_VALID_YAW) dst.yaw = src.yaw;
    if (v & HIPNUC_VALID_QUAT) std::copy(std::begin(src.quat), std::end(src.quat), std::begin(dst.quat));
    if (v & HIPNUC_VALID_DEVICE_TIME) dst.device_time_us = src.device_time_us;
    if (v & HIPNUC_VALID_PRESSURE) dst.pressure = src.pressure;
    if (v & HIPNUC_VALID_TEMPERATURE) dst.temperature = src.temperature;
    if (v & HIPNUC_VALID_POSITION) { dst.longitude = src.longitude; dst.latitude = src.latitude; }
    if (v & HIPNUC_VALID_ALTITUDE) dst.altitude_msl = src.altitude_msl;
    if (v & HIPNUC_VALID_VELOCITY_ENU) std::copy(std::begin(src.vel_enu), std::end(src.vel_enu), std::begin(dst.vel_enu));
}
}

HipnucIMUDriver::HipnucIMUDriver(uint16_t imu_id, const std::string& interface_type, const std::string& interface, const int baudrate)
    : IMUDriver(), interface_type_(interface_type), interface_(interface) {
    imu_id_ = imu_id;
    memset(&raw_, 0, sizeof(raw_));
    hipnuc_sample_clear(&sample_);
    if (interface_type_ == "serial") {
        baudrate_ = baudrate;
        serial_ = IMUSerialPort::open(interface_, baudrate_);
        IMUSerialPort::SerialCbkFunc serial_callback = std::bind(&HipnucIMUDriver::serial_rx_cbk, this, std::placeholders::_1, std::placeholders::_2);
        serial_->set_serial_callback(serial_callback);
    } else if (interface_type_ == "can") {
        can_ = IMUSocketCAN::get_instance(interface_);
        CanCbkFunc can_callback = std::bind(&HipnucIMUDriver::can_rx_cbk, this, std::placeholders::_1);
        can_->add_can_callback(can_callback, imu_id_);
        can_->set_key_extractor([](const can_frame &frame) -> CanCbkId {
            return frame.can_id & 0x7F;
        });
    } else if (interface_type_ == "canfd") {
        canfd_ = IMUSocketCANFD::open(interface_, std::bind(&HipnucIMUDriver::canfd_rx_cbk, this, std::placeholders::_1));
    } else {
        throw std::runtime_error("Hipnuc driver only support CAN and SERIAL interface");
    }
}

HipnucIMUDriver::~HipnucIMUDriver() {
    if (interface_type_ == "serial" && serial_) {
        serial_->close();
    } else if (interface_type_ == "can" && can_) {
        can_->remove_can_callback(imu_id_);
    } else if (interface_type_ == "canfd" && canfd_) {
        canfd_->close();
    }
}

void HipnucIMUDriver::can_rx_cbk(const can_frame& rx_frame) {
    std::unique_lock<std::shared_mutex> lock(imu_mutex_);
    hipnuc_sdk_can_frame_t frame{};
    hipnuc_sample_t decoded{};
    frame.id = rx_frame.can_id & CAN_EFF_MASK;
    frame.is_extended = (rx_frame.can_id & CAN_EFF_FLAG) != 0;
    frame.len = rx_frame.can_dlc;
    memcpy(frame.data, rx_frame.data, frame.len);
    if (hipnuc_sdk_j1939_parse(&frame, &decoded, nullptr) <= 0) return;
    if ((decoded.valid & HIPNUC_VALID_NODE_ID) && decoded.node_id != (imu_id_ & 0xFF)) return;
    merge_sample(sample_, decoded);
}

void HipnucIMUDriver::canfd_rx_cbk(const canfd_frame& rx_frame) {
    std::unique_lock<std::shared_mutex> lock(imu_mutex_);
    hipnuc_sdk_can_frame_t frame{};
    hipnuc_sample_t decoded{};
    frame.id = rx_frame.can_id & CAN_EFF_MASK;
    frame.is_extended = (rx_frame.can_id & CAN_EFF_FLAG) != 0;
    frame.len = rx_frame.len;
    memcpy(frame.data, rx_frame.data, frame.len);
    if (hipnuc_sdk_j1939_parse(&frame, &decoded, nullptr) <= 0) return;
    if ((decoded.valid & HIPNUC_VALID_NODE_ID) && decoded.node_id != (imu_id_ & 0xFF)) return;
    merge_sample(sample_, decoded);
}

void HipnucIMUDriver::serial_rx_cbk(const uint8_t* data, size_t length) {
    std::unique_lock<std::shared_mutex> lock(imu_mutex_);
    
    for (size_t i = 0; i < length; i++) {
        if (hipnuc_input(&raw_, data[i])) {
            if (!hipnuc_sample_from_raw(&raw_, &sample_)) continue;
        }
    }
}

std::vector<float> HipnucIMUDriver::get_ang_vel() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sample_.gyr[0], sample_.gyr[1], sample_.gyr[2]};
}

std::vector<float> HipnucIMUDriver::get_quat() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sample_.quat[0], sample_.quat[1], sample_.quat[2], sample_.quat[3]};
}

std::vector<float> HipnucIMUDriver::get_lin_acc() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return {sample_.acc[0], sample_.acc[1], sample_.acc[2]};
}

float HipnucIMUDriver::get_temperature() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return sample_.temperature;
}

hipnuc_sample_t HipnucIMUDriver::get_sample() const {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return sample_;
}
