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

#include <atomic>
#include <chrono>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <periodic_timer_thread.hpp>
#include <robot_state.hpp>
#include <robocap_reader.hpp>
#include <thread>
#include <yamlloader.hpp>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Exiting loop..." << std::endl;
        keep_running = false;
    }
}

// Thread to read RoboCap data from shared memory
class RoboCapThread : public PeriodicTimerThread {
public:
    RoboCapThread(std::shared_ptr<RobotSystemState> robot_state, RoboCapReader* reader,
                  double hz = 500.0)
        : PeriodicTimerThread(hz), robot_state_(robot_state), reader_(reader), update_count_(0) {}

protected:
    void before_start() override {
        std::cout << "[RoboCapThread] Starting RoboCap data reader thread at " << get_frequency()
                  << " Hz" << std::endl;
    }

    void after_stop() override {
        std::cout << "[RoboCapThread] Stopped. Total updates: " << update_count_ << std::endl;
    }

    void on_timer() override {
        std::vector<double> joint_angles;

        // Try to get new joint angles from shared memory
        if (reader_->get_joint_angles(joint_angles)) {
            // New data available - update robot state
            robot_state_->arm_state().set_all_references(joint_angles);

            // Update gripper
            double gripper_pos = reader_->get_gripper_position();
            std::vector<double> gripper_ref = {gripper_pos};
            robot_state_->hand_state().set_all_references(gripper_ref);

            update_count_++;

            // Print status every 500 updates (~1 second at 500Hz)
            if (update_count_ % 500 == 0) {
                std::cout << "[RoboCapThread] Updates: " << update_count_
                          << " | Timestamp: " << reader_->get_timestamp()
                          << " | Gripper: " << gripper_pos << std::endl;
            }
        }
        // If no new data, the previous reference values will be used (smooth continuation)
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    RoboCapReader* reader_;
    uint64_t update_count_;
};

// Thread to control the follower arm
class FollowerArmThread : public PeriodicTimerThread {
public:
    FollowerArmThread(std::shared_ptr<RobotSystemState> robot_state, Control* control_f,
                      double hz = 500.0)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_f_(control_f) {}

protected:
    void before_start() override {
        std::cout << "[FollowerArmThread] Starting follower control thread at " << get_frequency()
                  << " Hz" << std::endl;
    }

    void after_stop() override { std::cout << "[FollowerArmThread] Stopped" << std::endl; }

    void on_timer() override {
        // Execute one control step
        control_f_->unilateral_step();
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control* control_f_;
};

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, signal_handler);

        // Parse command line arguments
        std::string arm_side = "right_arm";
        std::string urdf_path;
        std::string can_interface = "can0";

        if (argc < 2) {
            std::cerr << "Usage: " << argv[0] << " <urdf_path> [arm_side] [can_interface]"
                      << std::endl;
            std::cerr << "Example: " << argv[0]
                      << " /path/to/openarm.urdf right_arm can0" << std::endl;
            return 1;
        }

        // Required: URDF path
        urdf_path = argv[1];

        // Optional: arm side
        if (argc >= 3) {
            arm_side = argv[2];
            if (arm_side != "left_arm" && arm_side != "right_arm") {
                std::cerr << "[ERROR] Invalid arm_side: " << arm_side
                          << ". Must be 'left_arm' or 'right_arm'." << std::endl;
                return 1;
            }
        }

        // Optional: CAN interface
        if (argc >= 4) {
            can_interface = argv[3];
        }

        // Check URDF file exists
        if (!std::filesystem::exists(urdf_path)) {
            std::cerr << "[ERROR] URDF file not found: " << urdf_path << std::endl;
            return 1;
        }

        // Print configuration
        std::cout << "========================================" << std::endl;
        std::cout << "  RoboCap → OpenArm Teleoperation" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Arm side       : " << arm_side << std::endl;
        std::cout << "CAN interface  : " << can_interface << std::endl;
        std::cout << "URDF path      : " << urdf_path << std::endl;
        std::cout << "Control freq   : " << FREQUENCY << " Hz" << std::endl;
        std::cout << "========================================\n" << std::endl;

        // Setup dynamics
        std::string root_link = "openarm_body_link0";
        std::string leaf_link =
            (arm_side == "left_arm") ? "openarm_left_hand" : "openarm_right_hand";

        std::cout << "[INFO] Initializing dynamics model..." << std::endl;
        Dynamics* arm_dynamics = new Dynamics(urdf_path, root_link, leaf_link);
        arm_dynamics->Init();
        std::cout << "[INFO] ✅ Dynamics model initialized" << std::endl;

        // Initialize RoboCap reader
        std::cout << "\n[INFO] Connecting to RoboCap shared memory..." << std::endl;
        std::cout << "[INFO] Make sure Python RoboCap writer is running!" << std::endl;
        RoboCapReader robocap_reader;

        // Initialize OpenArm hardware
        std::cout << "\n[INFO] Initializing OpenArm hardware on " << can_interface << "..."
                  << std::endl;
        openarm::can::socket::OpenArm* openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(can_interface, true);

        size_t arm_motor_num = openarm->get_arm().get_motors().size();
        size_t hand_motor_num = openarm->get_gripper().get_motors().size();

        std::cout << "[INFO] Arm motors  : " << arm_motor_num << std::endl;
        std::cout << "[INFO] Hand motors : " << hand_motor_num << std::endl;

        // Create robot state
        std::shared_ptr<RobotSystemState> robot_state =
            std::make_shared<RobotSystemState>(arm_motor_num, hand_motor_num);

        // Create control instance
        Control* control = new Control(openarm, arm_dynamics, arm_dynamics, robot_state,
                                       1.0 / FREQUENCY, ROLE_FOLLOWER, arm_side, arm_motor_num,
                                       hand_motor_num);

        // Load control parameters from YAML
        std::cout << "\n[INFO] Loading control parameters..." << std::endl;
        YamlLoader loader("config/follower.yaml");

        std::vector<double> kp = loader.get_vector("FollowerArmParam", "Kp");
        std::vector<double> kd = loader.get_vector("FollowerArmParam", "Kd");
        std::vector<double> Fc = loader.get_vector("FollowerArmParam", "Fc");
        std::vector<double> k = loader.get_vector("FollowerArmParam", "k");
        std::vector<double> Fv = loader.get_vector("FollowerArmParam", "Fv");
        std::vector<double> Fo = loader.get_vector("FollowerArmParam", "Fo");

        control->SetParameter(kp, kd, Fc, k, Fv, Fo);
        std::cout << "[INFO] ✅ Control parameters loaded" << std::endl;

        // Move to home position
        std::cout << "\n[INFO] Moving to home position..." << std::endl;
        control->AdjustPosition();
        std::cout << "[INFO] ✅ Home position reached" << std::endl;

        // Create and start control threads
        std::cout << "\n[INFO] Starting control threads..." << std::endl;
        RoboCapThread robocap_thread(robot_state, &robocap_reader, FREQUENCY);
        FollowerArmThread follower_thread(robot_state, control, FREQUENCY);

        robocap_thread.start_thread();
        follower_thread.start_thread();

        std::cout << "\n🚀 RoboCap teleoperation is now ACTIVE!" << std::endl;
        std::cout << "   Move your arm to control the robot" << std::endl;
        std::cout << "   Press Ctrl+C to stop\n" << std::endl;

        // Main loop - just wait for interrupt
        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Shutdown sequence
        std::cout << "\n[INFO] Shutting down..." << std::endl;
        robocap_thread.stop_thread();
        follower_thread.stop_thread();

        std::cout << "[INFO] Disabling motors..." << std::endl;
        openarm->disable_all();

        std::cout << "[INFO] ✅ Shutdown complete" << std::endl;

        // Cleanup
        delete control;
        delete arm_dynamics;

    } catch (const std::exception& e) {
        std::cerr << "\n❌ Fatal error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
