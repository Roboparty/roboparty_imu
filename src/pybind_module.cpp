// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

/**
 * @file pybind_module.cpp
 * @brief Python bindings for the IMU driver library via pybind11.
 * @details Exposes IMUDriver and all sensor data accessors to Python
 *          as the imu_py module. Each method includes inline docstring
 *          visible via help() in Python.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "imu_driver.hpp"

namespace py = pybind11;

PYBIND11_MODULE(imu_py, m) {
    m.doc() = "IMU Driver Python SDK";

    py::class_<IMUDriver, std::shared_ptr<IMUDriver>>(m, "IMUDriver")
        .def(py::init<>())
        .def_static("create_imu", &IMUDriver::create_imu,
            py::arg("imu_id"),
            py::arg("interface_type"),
            py::arg("interface"),
            py::arg("imu_type"),
            py::arg("baudrate") = 0,
            "工厂方法: create_imu(imu_id, interface_type, interface, imu_type, baudrate=0)\n"
            "  imu_id         int        节点 ID (CAN 模式用于多设备路由)\n"
            "  interface_type \"serial\"|\"can\"|\"canfd\"\n"
            "  interface      str        设备路径 (/dev/ttyUSB0, can0 …)\n"
            "  imu_type       \"MCT7123\"|\"HIPNUC\"\n"
            "  baudrate       int        serial 必须, can/canfd 忽略")
        .def("get_imu_id", &IMUDriver::get_imu_id,
            "IMU 节点 ID — CAN 模式用于多设备路由")
        .def("get_ang_vel", &IMUDriver::get_ang_vel,
            "角速度 [x, y, z]     rad/s  — 陀螺仪输出")
        .def("get_quat", &IMUDriver::get_quat,
            "姿态四元数 [w, x, y, z]       — IMU 姿态解算输出")
        .def("get_lin_acc", &IMUDriver::get_lin_acc,
            "线加速度 [x, y, z]   m/s²   — 加速度计输出")
        .def("get_mag", &IMUDriver::get_mag,
            "磁场强度 [x, y, z]   uT     — 磁力计输出 (9 轴模式下有效)")
        .def("get_euler", &IMUDriver::get_euler,
            "欧拉角 [roll, pitch, yaw] 度 — IMU 硬件直接输出")
        .def("get_timestamp", &IMUDriver::get_timestamp,
            "系统时间戳           us     — IMU 内部时钟 (开机累积)")
        .def("get_temperature", &IMUDriver::get_temperature,
            "传感器温度           °C");
}
