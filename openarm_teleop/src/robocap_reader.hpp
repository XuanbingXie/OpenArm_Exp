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

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "robocap_shared_data.hpp"

class RoboCapReader {
public:
    RoboCapReader(const char* shm_name = ROBOCAP_SHM_NAME) : data_(nullptr), last_sequence_(0) {
        // Open shared memory
        int fd = shm_open(shm_name, O_RDONLY, 0666);
        if (fd == -1) {
            throw std::runtime_error("Failed to open shared memory: " + std::string(shm_name) +
                                     ". Make sure Python RoboCap writer is running first.");
        }

        // Map shared memory to process address space
        data_ = static_cast<RoboCapSharedData*>(
            mmap(nullptr, ROBOCAP_SHM_SIZE, PROT_READ, MAP_SHARED, fd, 0));

        if (data_ == MAP_FAILED) {
            close(fd);
            throw std::runtime_error("Failed to map shared memory");
        }

        close(fd);  // File descriptor can be closed after mmap

        std::cout << "[RoboCapReader] ✅ Connected to shared memory: " << shm_name << std::endl;
        std::cout << "[RoboCapReader] Waiting for RoboCap data..." << std::endl;

        // Wait for initial data
        wait_for_data(5.0);
    }

    ~RoboCapReader() {
        if (data_ != nullptr && data_ != MAP_FAILED) {
            munmap(data_, ROBOCAP_SHM_SIZE);
        }
    }

    // Get joint angles (returns true if new data is available)
    bool get_joint_angles(std::vector<double>& joints) {
        if (!is_data_ready()) {
            return false;
        }

        uint64_t current_seq = data_->sequence_number;
        if (current_seq == last_sequence_) {
            return false;  // No new data
        }

        // Read joint angles
        joints.resize(7);
        std::memcpy(joints.data(), data_->right_arm_joints, 7 * sizeof(double));

        last_sequence_ = current_seq;
        return true;
    }

    // Get gripper position
    double get_gripper_position() const { return data_->right_gripper; }

    // Get timestamp
    double get_timestamp() const { return data_->timestamp; }

    // Get pelvis position
    void get_pelvis_position(double pos[3]) const {
        std::memcpy(pos, data_->pelvis_position, 3 * sizeof(double));
    }

    // Get raw quaternions (for debugging)
    void get_raw_quaternions(RoboCapSharedData::Quaternion& shoulder,
                             RoboCapSharedData::Quaternion& elbow,
                             RoboCapSharedData::Quaternion& wrist,
                             RoboCapSharedData::Quaternion& hand) const {
        shoulder = data_->r_shoulder;
        elbow = data_->r_elbow;
        wrist = data_->r_wrist;
        hand = data_->r_hand;
    }

    // Check if data is ready
    bool is_data_ready() const { return data_ != nullptr && data_->data_ready == 1; }

    // Get statistics
    uint64_t get_total_updates() const { return data_->total_updates; }

    double get_last_update_time() const { return data_->last_update_time; }

    // Wait for data with timeout
    bool wait_for_data(double timeout_seconds = 5.0) {
        auto start = std::chrono::steady_clock::now();
        while (!is_data_ready()) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed =
                std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
            if (elapsed >= timeout_seconds) {
                std::cerr << "[RoboCapReader] ⚠️  Timeout waiting for RoboCap data" << std::endl;
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "[RoboCapReader] ✅ RoboCap data received!" << std::endl;
        return true;
    }

private:
    RoboCapSharedData* data_;
    uint64_t last_sequence_;
};
