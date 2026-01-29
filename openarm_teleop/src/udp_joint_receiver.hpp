// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * UDP receiver for joint angles from RoboCap
 * Receives JSON data containing joint angles and gripper position
 */
class UdpJointReceiver {
public:
    UdpJointReceiver(int port = 5678, size_t num_joints = 7)
        : port_(port),
          num_joints_(num_joints),
          socket_fd_(-1),
          running_(false),
          data_ready_(false),
          sequence_number_(0),
          timestamp_(0.0),
          left_gripper_torque_(0.0),
          right_gripper_torque_(0.0) {
        left_joint_angles_.resize(num_joints_, 0.0);
        right_joint_angles_.resize(num_joints_, 0.0);
        pelvis_position_.resize(3, 0.0);

        // Create UDP socket
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            throw std::runtime_error("Failed to create UDP socket");
        }

        // Set socket to non-blocking mode
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
        // Set this can make socket monitored by multiple processes
        // int optval = 1;
        // if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval)) < 0) {
        //     throw std::runtime_error("Failed to set SO_REUSEPORT");
        // }
        // Bind to port
        struct sockaddr_in server_addr;
        std::memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port_);

        if (bind(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            close(socket_fd_);
            throw std::runtime_error("Failed to bind UDP socket to port " + std::to_string(port_));
        }

        // Start receiver thread
        running_ = true;
        receiver_thread_ = std::thread(&UdpJointReceiver::receive_loop, this);

        std::cout << "[UdpJointReceiver] Waiting for UDP data..." << std::endl;
        wait_for_data(5.0);
    }

    ~UdpJointReceiver() {
        running_ = false;
        if (receiver_thread_.joinable()) {
            receiver_thread_.join();
        }
        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }
    }

    bool get_joints_angles(std::vector<double>& l_joints, std::vector<double>& r_joints) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!data_ready_) return false;

        uint64_t current_seq = sequence_number_;
        if (current_seq == last_read_sequence_) {
            return false;  // No new data since last read
        }

        l_joints = left_joint_angles_;
        r_joints = right_joint_angles_;
        last_read_sequence_ = current_seq;
        return true;
    }

    // Get left gripper torque
    double get_left_gripper_torque() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return left_gripper_torque_;
    }

    // Get right gripper torque
    double get_right_gripper_torque() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return right_gripper_torque_;
    }

    // Get timestamp
    double get_timestamp() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return timestamp_;
    }

    // Check if data is ready
    bool is_data_ready() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return data_ready_;
    }

    // Get statistics
    uint64_t get_total_updates() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return sequence_number_;
    }

    // Wait for data with timeout
    bool wait_for_data(double timeout_seconds = 5.0) {
        auto start = std::chrono::steady_clock::now();
        while (!is_data_ready()) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed =
                std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
            if (elapsed >= timeout_seconds) {
                std::cerr << "[UdpJointReceiver] ⚠️  Timeout waiting for UDP data" << std::endl;
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "[UdpJointReceiver] ✅ UDP data received!" << std::endl;
        return true;
    }

private:
    void receive_loop() {
        char buffer[65535];
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        std::cout << "[UdpJointReceiver] Receive loop started" << std::endl;

        while (running_) {
            ssize_t recv_len = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                        (struct sockaddr*)&client_addr, &client_len);

            if (recv_len > 0) {
                buffer[recv_len] = '\0';
                
                // Debug: print first packet info
                static bool first_packet = true;
                if (first_packet) {
                    std::cout << "[UdpJointReceiver] First packet received! " 
                              << recv_len << " bytes from " 
                              << inet_ntoa(client_addr.sin_addr) << ":" 
                              << ntohs(client_addr.sin_port) << std::endl;
                    first_packet = false;
                }
                
                process_data(buffer, recv_len);
            } else if (recv_len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "[UdpJointReceiver] recvfrom error: " << strerror(errno) << std::endl;
            }
            
            // No data available, sleep briefly
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        
        std::cout << "[UdpJointReceiver] Receive loop stopped" << std::endl;
    }

    void process_data(const char* data, size_t length) {
        try {
            // Debug: print raw data
            if (sequence_number_ < 3) {
                std::cout << "[UdpJointReceiver] Raw data (" << length << " bytes): "
                          << std::string(data, std::min(length, size_t(100))) << "..." << std::endl;
            }

            // Parse JSON
            json j = json::parse(std::string(data, length));

            std::lock_guard<std::mutex> lock(data_mutex_);

            // Extract left joint angles
            if (j.contains("left_joints")) {
                std::vector<double> left_joints_array = j["left_joints"].get<std::vector<double>>();
                if (left_joints_array.size() == num_joints_) {
                    left_joint_angles_ = left_joints_array;
                } else {
                    std::cerr << "[UdpJointReceiver] Warning: Expected " << num_joints_
                              << " left joints, got " << left_joints_array.size() << std::endl;
                }
            }

            // Extract right joint angles
            if (j.contains("right_joints")) {
                std::vector<double> right_joints_array = j["right_joints"].get<std::vector<double>>();
                if (right_joints_array.size() == num_joints_) {
                    right_joint_angles_ = right_joints_array;
                } else {
                    std::cerr << "[UdpJointReceiver] Warning: Expected " << num_joints_
                              << " right joints, got " << right_joints_array.size() << std::endl;
                }
            }

            // Extract left gripper torque
            if (j.contains("left_gripper")) {
                left_gripper_torque_ = j["left_gripper"].get<double>();
            }

            // Extract right gripper torque
            if (j.contains("right_gripper")) {
                right_gripper_torque_ = j["right_gripper"].get<double>();
            }

            // Extract timestamp
            if (j.contains("timestamp")) {
                timestamp_ = j["timestamp"].get<double>();
            }

            // Update sequence and mark data as ready
            sequence_number_++;
            data_ready_ = true;

        } catch (const std::exception& e) {
            std::cerr << "[UdpJointReceiver] Parse error: " << e.what() << std::endl;
            std::cerr << "[UdpJointReceiver] Data: " << std::string(data, std::min(length, size_t(200))) << std::endl;
        }
    }

    int port_;
    size_t num_joints_;
    int socket_fd_;
    std::atomic<bool> running_;
    std::thread receiver_thread_;

    // Protected data
    mutable std::mutex data_mutex_;
    bool data_ready_;
    uint64_t sequence_number_;
    uint64_t last_read_sequence_ = 0;
    double timestamp_;
    std::vector<double> left_joint_angles_;
    std::vector<double> right_joint_angles_;
    double left_gripper_torque_;
    double right_gripper_torque_;
    std::vector<double> pelvis_position_;
};
