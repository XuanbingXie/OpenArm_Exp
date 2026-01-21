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

#include <cstdint>

// Shared memory data structure for RoboCap teleoperation
// This structure is shared between Python (RoboCap SDK) and C++ (OpenArm control)
struct RoboCapSharedData {
    // Synchronization
    uint64_t sequence_number;  // Incremented on each update
    uint8_t data_ready;        // 1 if data is valid, 0 otherwise
    uint8_t padding[7];        // Alignment padding

    // Timestamp
    double timestamp;  // RoboCap timestamp

    // Right arm joint angles (7 DOF)
    // Joint order: [shoulder_yaw, shoulder_pitch, shoulder_roll, elbow, wrist_yaw, wrist_pitch,
    // wrist_roll]
    double right_arm_joints[7];

    // Right gripper position (0.0 = closed, 1.0 = open)
    double right_gripper;

    // Raw quaternions from RoboCap (for debugging and advanced mapping)
    struct Quaternion {
        double w, x, y, z;
    };
    Quaternion r_shoulder;  // Right shoulder quaternion
    Quaternion r_elbow;     // Right elbow quaternion
    Quaternion r_wrist;     // Right wrist quaternion
    Quaternion r_hand;      // Right hand quaternion

    // Pelvis position (global position from RoboCap)
    double pelvis_position[3];  // [x, y, z]

    // Statistics (for monitoring)
    uint64_t total_updates;  // Total number of updates
    double last_update_time; // Last update timestamp
};

// Shared memory configuration
constexpr const char* ROBOCAP_SHM_NAME = "/robocap_openarm_data";
constexpr size_t ROBOCAP_SHM_SIZE = sizeof(RoboCapSharedData);
