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
#include <string>
#include <math.h>

// Packet layout (float32) for rebocap: [timestamp, 14 joints, left_gripper, right_gripper]
#pragma pack(push, 1)
struct UdpFloatPacket {
    float timestamp;
    float joints[14];
};
#pragma pack(pop)

// Packet layout (float32) for Manus
#pragma pack(push, 1)
struct UdpManusPacket {
    uint32_t sequence_number;
    uint32_t left_glove_id;
    uint32_t right_glove_id;
    float left_sensor_transforms[5][7];
    float left_wrist_rotation[4];
    float right_sensor_transforms[5][7];
    float right_wrist_rotation[4];
};
#pragma pack(pop)


// UDP receiver for joint angles from RoboCap
class UdpJointReceiver {
public:
        UdpJointReceiver(int port = 5678, size_t num_joints = 7, const std::string &listen_ip = "0.0.0.0")
                : port_(port),
                    num_joints_(num_joints),
                    socket_fd_(-1),
                    running_(false),
                    sequence_number_(0),
                    timestamp_(0.0),
                    left_gripper_pos_(0.0),
                    right_gripper_pos_(0.0) {
        left_joint_angles_.resize(num_joints_, 0.0);
        right_joint_angles_.resize(num_joints_, 0.0);

        // Create UDP socket
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            throw std::runtime_error("Failed to create UDP socket for rebocap");
        }
        manus_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (manus_socket_fd_ < 0) {
            throw std::runtime_error("Failed to create UDP socket for manus");
        }
        // Set socket to non-blocking mode
        int flags = fcntl(manus_socket_fd_, F_GETFL, 0);
        fcntl(manus_socket_fd_, F_SETFL, flags | O_NONBLOCK);

        // Bind to port (optionally to a specific listen IP)
        struct sockaddr_in server_addr;
        std::memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);
        
        struct sockaddr_in manus_server_addr;
        std::memset(&manus_server_addr, 0, sizeof(manus_server_addr));
        manus_server_addr.sin_family = AF_INET;
        constexpr uint16_t MANUS_PORT = 9000;
        manus_server_addr.sin_port = htons(MANUS_PORT);
        manus_server_addr.sin_addr.s_addr = INADDR_ANY;

        if (listen_ip.empty() || listen_ip == "0.0.0.0") {
            server_addr.sin_addr.s_addr = INADDR_ANY;
        } else {
            int p = inet_pton(AF_INET, listen_ip.c_str(), &server_addr.sin_addr);
            if (p == 1) {
                // parsed ok
            } else if (p == 0) {
                close(socket_fd_);
                throw std::runtime_error(std::string("Invalid listen IP address: ") + listen_ip);
            } else {
                close(socket_fd_);
                throw std::runtime_error(std::string("inet_pton error for address: ") + listen_ip + ": " + std::strerror(errno));
            }
        }

        if (bind(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            close(socket_fd_);
            throw std::runtime_error(std::string("Failed to bind UDP socket to ") + listen_ip + ":" + std::to_string(port_));
        }
        if (bind(socket_fd_, (struct sockaddr*)&manus_server_addr, sizeof(manus_server_addr)) < 0) {
            close(manus_socket_fd_);
            throw std::runtime_error(std::string("Failed to bind Manus UDP socket to port ") + std::to_string(MANUS_PORT));
        }
    }

    ~UdpJointReceiver() {
        running_ = false;
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
        if (manus_socket_fd_ >= 0) {
            close(manus_socket_fd_);
            manus_socket_fd_ = -1;
        }
    }

    bool get_joints_angles(std::vector<double>& l_joints, std::vector<double>& r_joints) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
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
            
            // Extract pack data
            process_data(buffer, recv_len);

            l_joints = left_joint_angles_;
            r_joints = right_joint_angles_;
            return true;
        } 
        return false;
    }

    bool get_gripper_pos(double& l_pos, double& r_pos) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        ssize_t recv_len = recvfrom(manus_socket_fd_, buffer, sizeof(buffer) - 1, 0,
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
            
            // Extract pack data
            process_manus_data(buffer, recv_len);

            l_pos = left_gripper_pos_;
            r_pos = right_gripper_pos_;
            return true;
        } 
        return false;
    }

private:
    void process_data(const char* data, size_t length) {
        const size_t expected_bytes = sizeof(UdpFloatPacket);
        if (length < expected_bytes) {
            if (sequence_number_ < 3) {
                std::cerr << "[UdpJointReceiver] Warning: packet too small (" << length
                          << " bytes), expected " << expected_bytes << " bytes" << std::endl;
            }
            return;
        }

        UdpFloatPacket pkt;
        std::memcpy(&pkt, data, expected_bytes);

        for (size_t i = 0; i < num_joints_; ++i) {
            left_joint_angles_[i] = static_cast<double>(pkt.joints[i]);
            right_joint_angles_[i] = static_cast<double>(pkt.joints[num_joints_ + i]);
        }

        timestamp_ = static_cast<double>(pkt.timestamp);
        ++sequence_number_;
    }

    void process_manus_data(const char* data, size_t length) {
        const size_t expected_bytes = sizeof(UdpManusPacket);
        if (length < expected_bytes) {
            if (sequence_number_ < 3) {
                std::cerr << "[UdpJointReceiver] Warning: packet too small (" << length
                          << " bytes), expected " << expected_bytes << " bytes" << std::endl;
            }
            return;
        }

        UdpManusPacket pkt;
        std::memcpy(&pkt, data, expected_bytes);

        if (pkt.left_glove_id != 0) {
            float l_dist = sqrtf(powf(pkt.left_sensor_transforms[0][0]-pkt.left_sensor_transforms[1][0], 2) +
                                    powf(pkt.left_sensor_transforms[0][1]-pkt.left_sensor_transforms[1][1], 2) +
                                    powf(pkt.left_sensor_transforms[0][2]-pkt.left_sensor_transforms[1][2], 2));
            left_gripper_pos_ = finger_dist_to_gripper_joint_pos(l_dist);
        }
        if (pkt.right_glove_id != 0) {
            float r_dist = sqrtf(powf(pkt.right_sensor_transforms[0][0]-pkt.right_sensor_transforms[1][0], 2) +
                                    powf(pkt.right_sensor_transforms[0][1]-pkt.right_sensor_transforms[1][1], 2) +
                                    powf(pkt.right_sensor_transforms[0][2]-pkt.right_sensor_transforms[1][2], 2));
            right_gripper_pos_ = finger_dist_to_gripper_joint_pos(r_dist);
        }
    }

    float finger_dist_to_gripper_joint_pos(float dist) {
        constexpr float offset = 3.651 / 100.0; // 3.651cm 
        constexpr float max_hand_dist = 11.873 / 100.0; // 11.873cm
        dist = std::max(0.f, dist-offset);
        return -std::min(1.f, dist/max_hand_dist);
    }

    int port_;
    size_t num_joints_;
    int socket_fd_;
    int manus_socket_fd_;
    std::atomic<bool> running_;

    // Protected data
    char buffer[65535];
    uint64_t sequence_number_;
    double timestamp_;
    std::vector<double> left_joint_angles_;
    std::vector<double> right_joint_angles_;
    double left_gripper_pos_;
    double right_gripper_pos_;
};
