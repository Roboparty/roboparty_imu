// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

/**
 * @file socket_can.cpp
 * @brief SocketCAN receiver implementation for CAN/CAN FD frames.
 * @details Contains socket initialization, epoll-based frame reception,
 *          and error frame filtering logic via Linux SocketCAN.
 */

#include "socket_can.hpp"

#include <algorithm>

std::shared_ptr<spdlog::logger> IMUSocketCAN::logger_ = nullptr;
std::unordered_map<std::string, std::shared_ptr<IMUSocketCAN>> IMUSocketCAN::instances_;

IMUSocketCAN::IMUSocketCAN(std::string interface)
    : interface_(interface), sockfd_(INIT_FD), receiving_(false) {
    open(interface);
}

IMUSocketCAN::~IMUSocketCAN() { this->close(); }

void IMUSocketCAN::open(std::string interface) {
    sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd_ == INIT_FD) {
        logger_->error("Failed to create CAN socket");
        throw std::runtime_error("Failed to create CAN socket");
    }

    /* Enable CAN FD frame reception (backward compatible with classic CAN) */
    int canfd_on = 1;
    if (setsockopt(sockfd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on)) < 0) {
        logger_->warn("CAN FD not supported on interface {}, falling back to classic CAN", interface);
    }

    strncpy(if_request_.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(sockfd_, SIOCGIFINDEX, &if_request_) == -1) {
        logger_->error("Unable to detect CAN interface {}", interface);

        this->close();
        throw std::runtime_error("Unable to detect CAN interface " + interface);
    }

    // Bind the socket to the network interface
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = if_request_.ifr_ifindex;
    int rc = ::bind(sockfd_, reinterpret_cast<struct sockaddr *>(&addr_), sizeof(addr_));
    if (rc == -1) {
        logger_->error("Failed to bind socket to network interface {}", interface);
        this->close();
        throw std::runtime_error("Failed to bind socket to network interface " + interface);
    }

    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags == -1) {
        logger_->error("Failed to get socket flags");
        this->close();
        throw std::runtime_error("Failed to get socket flags");
    }
    if (fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) == -1) {
        logger_->error("Failed to set socket to non-blocking");
        this->close();
        throw std::runtime_error("Failed to set socket to non-blocking");
    }

    receiving_ = true;
    receiver_thread_ = std::thread([this]() {
        pthread_setname_np(pthread_self(), "can_rx");
        struct sched_param sp{}; sp.sched_priority = 80;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
            logger_->warn("Failed to set realtime priority for IMU CAN RX thread");
        }
        fd_set descriptors;
        int maxfd = sockfd_;
        struct timeval timeout;
        canfd_frame rx_frame;

        while (receiving_) {
            FD_ZERO(&descriptors);
            FD_SET(sockfd_, &descriptors);

            timeout.tv_sec = TIMEOUT_SEC;
            timeout.tv_usec = TIMEOUT_USEC;

            int sel_ret = ::select(maxfd + 1, &descriptors, NULL, NULL, &timeout);
            if (sel_ret < 0) {
                if (errno == EINTR) continue;
                logger_->error("CAN select error: {}", strerror(errno));
                break;
            }
            if (sel_ret == 1) {
                while (true){
                    int len = ::read(sockfd_, &rx_frame, sizeof(canfd_frame));
                    if (len < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break; 
                        }
                        logger_->warn("CAN read error: {}", strerror(errno));
                        break;
                    }
                    if (len == 0){
                        break;
                    }
                    std::vector<CanCbkEntryPtr> callbacks_to_run;
                    {
                        std::lock_guard<std::mutex> lock(can_callback_mutex_);
                        CanCbkId key = key_extractor_(rx_frame);
                        auto it = can_callback_list_.find(key);
                        if (it != can_callback_list_.end()) {
                            callbacks_to_run = it->second;
                        }
                    }
                    for (const auto &entry : callbacks_to_run) {
                        {
                            std::lock_guard<std::mutex> lock(entry->mutex);
                            if (!entry->enabled) continue;
                            ++entry->running_count;
                        }

                        try {
                            entry->callback(rx_frame);
                        } catch (const std::exception &e) {
                            logger_->error("CAN callback error: {}", e.what());
                        } catch (...) {
                            logger_->error("CAN callback error: unknown exception");
                        }

                        {
                            std::lock_guard<std::mutex> lock(entry->mutex);
                            --entry->running_count;
                            if (entry->running_count == 0) entry->cv.notify_all();
                        }
                    }
                }
            }
        }
    });
}

void IMUSocketCAN::close() {
    receiving_ = false;
    if (receiver_thread_.joinable()) receiver_thread_.join();

    if (sockfd_ != INIT_FD) {
        if (::close(sockfd_) < 0) {
            logger_->warn("Failed to close socket {}: {}", interface_, strerror(errno));
        } else {
            logger_->info("CAN interface {} closed successfully.", interface_);
        }
    }
    sockfd_ = INIT_FD;
}

CanCbkToken IMUSocketCAN::add_can_callback(const CanCbkFunc callback, const CanCbkId id) {
    std::lock_guard<std::mutex> lock(can_callback_mutex_);
    const CanCbkToken token = next_callback_token_++;
    auto entry = std::make_shared<CanCbkEntry>();
    entry->token = token;
    entry->callback = callback;
    can_callback_list_[id].push_back(std::move(entry));
    return token;
}

void IMUSocketCAN::remove_can_callback(CanCbkId id, CanCbkToken token) {
    CanCbkEntryPtr removed_entry;
    {
        std::lock_guard<std::mutex> lock(can_callback_mutex_);
        auto it = can_callback_list_.find(id);
        if (it == can_callback_list_.end()) return;

        auto &entries = it->second;
        auto entry_it = std::find_if(
            entries.begin(), entries.end(),
            [token](const CanCbkEntryPtr &entry) { return entry->token == token; });
        if (entry_it == entries.end()) return;

        removed_entry = *entry_it;
        entries.erase(entry_it);
        if (entries.empty()) can_callback_list_.erase(it);
    }

    // Do not call this synchronously from the callback being removed.
    std::unique_lock<std::mutex> lock(removed_entry->mutex);
    removed_entry->enabled = false;
    removed_entry->cv.wait(lock, [&removed_entry] {
        return removed_entry->running_count == 0;
    });
}

void IMUSocketCAN::clear_can_callbacks() {
    std::vector<CanCbkEntryPtr> removed_entries;
    {
        std::lock_guard<std::mutex> lock(can_callback_mutex_);
        for (const auto &item : can_callback_list_) {
            removed_entries.insert(removed_entries.end(),
                                   item.second.begin(), item.second.end());
        }
        can_callback_list_.clear();
    }

    for (const auto &entry : removed_entries) {
        std::unique_lock<std::mutex> lock(entry->mutex);
        entry->enabled = false;
        entry->cv.wait(lock, [&entry] { return entry->running_count == 0; });
    }
}

void IMUSocketCAN::set_key_extractor(CanCbkKeyExtractor extractor) {
    std::lock_guard<std::mutex> lock(can_callback_mutex_);
    key_extractor_ = std::move(extractor);
}

int IMUSocketCAN::send(const canfd_frame &frame) {
    if (sockfd_ == INIT_FD) return -1;
    ssize_t n = ::write(sockfd_, &frame, CANFD_MTU);
    if (n < 0) {
        logger_->warn("CAN send error on {}: {}", interface_, strerror(errno));
        return -1;
    }
    return (int)n;
}
