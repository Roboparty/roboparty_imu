// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

/**
 * @file serial_port.hpp
 * @brief Serial port interface for IMU communication.
 * @details Provides a UART serial port abstraction with asynchronous read
 *          callback support for receiving IMU data streams.
 */

#pragma once

#include <string>
#include <memory>
#include <functional>
#include <thread>
#include <atomic>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <iostream>
#include <cerrno>
#include <termios.h>
#include <pthread.h>

#define BUF_SIZE 1024

class IMUSerialPort {
public:
    using SerialCbkFunc = std::function<void(const uint8_t*, size_t)>;

    IMUSerialPort(const IMUSerialPort &) = delete;
    IMUSerialPort &operator=(const IMUSerialPort &) = delete;
    ~IMUSerialPort();

    static void init_logger(std::shared_ptr<spdlog::logger> logger) { logger_ = logger; }
    static std::shared_ptr<IMUSerialPort> open(const std::string& interface, int baudrate);
    void init();

    void set_serial_callback(SerialCbkFunc callback);

    /**
     * @brief Write bytes to the serial port (thread-safe).
     * @param[in] data Bytes to transmit.
     * @param[in] length Number of bytes; short writes are retried internally.
     */
    void send(const uint8_t* data, size_t length);

    void close();

private:
    IMUSerialPort(const std::string& interface, int baudrate);

    std::string interface_;
    int baudrate_;
    int fd_;
    std::atomic<bool> running_;
    std::thread rx_thread_;
    SerialCbkFunc callback_;
    std::mutex write_mutex_;

    static std::shared_ptr<spdlog::logger> logger_;
};
