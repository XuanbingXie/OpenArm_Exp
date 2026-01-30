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
#include <udp_joint_receiver.hpp>
#include <thread>
#include <yamlloader.hpp>
#include <manus_interop.hpp>

// Low-pass filter class for smoothing joint angles
class LowPassFilter {
public:
    LowPassFilter(double alpha) : alpha_(alpha), prev_output_(0.0) {}
    double update(double input) {
        double output = alpha_ * input + (1.0 - alpha_) * prev_output_;
        prev_output_ = output;
        return output;
    }
private:
    double alpha_;
    double prev_output_;
};

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Exiting loop..." << std::endl;
        keep_running = false;
        keep_manus_running = false;
    }
}


// Thread to read joint data from UDP for dual arms
class UdpReceiverThread : public PeriodicTimerThread {
public:
    UdpReceiverThread(std::shared_ptr<RobotSystemState> left_robot_state,
                      std::shared_ptr<RobotSystemState> right_robot_state,
                      UdpJointReceiver* receiver,
                      double hz = 100.0, std::vector<double> filter_alphas = std::vector<double>(7, 0.1))
        : PeriodicTimerThread(hz), left_robot_state_(left_robot_state), right_robot_state_(right_robot_state),
          receiver_(receiver), update_count_(0), hz_(hz) {
        // Initialize filters with individual alphas for each joint
        for (size_t i = 0; i < filter_alphas.size(); ++i) {
            left_filters_.push_back(LowPassFilter(filter_alphas[i]));
            right_filters_.push_back(LowPassFilter(filter_alphas[i]));
        }
    }

protected:
    void before_start() override {
        std::cout << "[UdpReceiverThread] Starting UDP data reader thread at "
                  << hz_ << " Hz" << std::endl;
    }

    void after_stop() override {
        std::cout << "[UdpReceiverThread] Stopped. Total updates: " << update_count_ << std::endl;
    }

    void on_timer() override {
        std::vector<double> left_joint_angles;
        std::vector<double> right_joint_angles;

        // Try to get new joint angles from UDP
        if (receiver_->get_joints_angles(left_joint_angles, right_joint_angles)) {
            // ----------------------Left-------------------------
            if (left_robot_state_ && !left_joint_angles.empty()) {
                // Apply low-pass filter to left joint angles
                for (size_t i = 0; i < left_joint_angles.size(); ++i) {
                    left_joint_angles[i] = left_filters_[i].update(left_joint_angles[i]);
                }

                // New data available - convert to JointState and update left robot state
                std::vector<JointState> left_joint_states(left_joint_angles.size());
                for (size_t i = 0; i < left_joint_angles.size(); ++i) {
                    left_joint_states[i].position = left_joint_angles[i];
                    left_joint_states[i].velocity = 0.0;
                    left_joint_states[i].effort = 0.0;
                }
                left_robot_state_->arm_state().set_all_references(left_joint_states);

                // Update left gripper (for gui, torque control)
                double left_gripper_tor = receiver_->get_left_gripper_torque();
                std::vector<JointState> left_gripper_states(1);
                left_gripper_states[0].position = 0;
                left_gripper_states[0].velocity = 0.0;
                left_gripper_states[0].effort = left_gripper_tor;
                left_robot_state_->hand_state().set_all_references(left_gripper_states);
            }

            // ----------------------Right-------------------------
            if (right_robot_state_ && !right_joint_angles.empty()) {
                // Apply low-pass filter to right joint angles
                for (size_t i = 0; i < right_joint_angles.size(); ++i) {
                    right_joint_angles[i] = right_filters_[i].update(right_joint_angles[i]);
                }

                // New data available - convert to JointState and update right robot state
                std::vector<JointState> right_joint_states(right_joint_angles.size());
                for (size_t i = 0; i < right_joint_angles.size(); ++i) {
                    right_joint_states[i].position = right_joint_angles[i];
                    right_joint_states[i].velocity = 0.0;
                    right_joint_states[i].effort = 0.0;
                }
                right_robot_state_->arm_state().set_all_references(right_joint_states);

                // Update right gripper (for gui, torque control)
                double right_gripper_tor = receiver_->get_right_gripper_torque();
                std::vector<JointState> right_gripper_states(1);
                right_gripper_states[0].position = 0;
                right_gripper_states[0].velocity = 0.0;
                right_gripper_states[0].effort = right_gripper_tor;
                right_robot_state_->hand_state().set_all_references(right_gripper_states);
            }

            // // Print status every 500 updates (~1 second at 500Hz)
            // if (++update_count_ % 500 == 0) {
            //     std::cout << "[UdpReceiverThread] Updates: " << update_count_
            //               << " | Timestamp: " << receiver_->get_timestamp();
            //     if (left_robot_state_) {
            //         std::cout << " | Left Gripper Torque: " << receiver_->get_left_gripper_torque();
            //     }
            //     if (right_robot_state_) {
            //         std::cout << " | Right Gripper Torque: " << receiver_->get_right_gripper_torque();
            //     }
            //     std::cout << std::endl;
            // }
        }

        // Set gripper states (for Manus, position control)
        if (left_robot_state_) {
            std::vector<JointState> left_gripper_states(1);
            left_gripper_states[0].position = thumb_dist_to_gripper_joint_pos(shared_thumb_index_distance_left);
            left_gripper_states[0].velocity = 0.0;
            left_gripper_states[0].effort = 0.0;
            left_robot_state_->hand_state().set_all_references(left_gripper_states);
        }

        if (right_robot_state_) {
            std::vector<JointState> right_gripper_states(1);
            right_gripper_states[0].position = thumb_dist_to_gripper_joint_pos(shared_thumb_index_distance_right);
            right_gripper_states[0].velocity = 0.0;
            right_gripper_states[0].effort = 0.0;
            right_robot_state_->hand_state().set_all_references(right_gripper_states);
        }

        // // For debug
        // if (left_robot_state_) {
        //     static std::vector<JointState> debug_left_joint_angles{
        //         {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        //         {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        //         {0.0, 0.0, 0.0}};
        //     left_robot_state_->arm_state().set_all_references(debug_left_joint_angles);
        // }

        // if (right_robot_state_) {
        //     static std::vector<JointState> debug_right_joint_angles{
        //         {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        //         {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        //         {0.0, 0.0, 0.0}};
        //     right_robot_state_->arm_state().set_all_references(debug_right_joint_angles);
        // }
    }

private:
    std::shared_ptr<RobotSystemState> left_robot_state_;
    std::shared_ptr<RobotSystemState> right_robot_state_;
    UdpJointReceiver* receiver_;
    uint64_t update_count_;
    double hz_;
    std::vector<LowPassFilter> left_filters_;
    std::vector<LowPassFilter> right_filters_;
};

// Thread to control the follower arm
class FollowerArmThread : public PeriodicTimerThread {
public:
    FollowerArmThread(std::shared_ptr<RobotSystemState> robot_state, Control* control_f,
                      double hz = 1000.0)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_f_(control_f), hz_(hz) {}

protected:
    void before_start() override {
        std::cout << "[FollowerArmThread] Starting follower control thread at " 
                  << hz_ << " Hz" << std::endl;
    }

    void after_stop() override { std::cout << "[FollowerArmThread] Stopped" << std::endl; }

    void on_timer() override {
        // Execute one control step
        control_f_->unilateral_step();
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control* control_f_;
    double hz_;
};

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, signal_handler);


        // Parse command line arguments
        std::string urdf_path;
        std::string arm_mode = "dual";
        std::string can_interface1 = "can1";
        std::string can_interface2 = "can0";
        int udp_port = 5678;

        if (argc < 2) {
            std::cerr << "Usage: " << argv[0] << " <urdf_path> [arm_mode] [can_interface1] [can_interface2] [udp_port]"
                      << std::endl;
            std::cerr << "arm_mode: dual (default), single_left, single_right" << std::endl;
            std::cerr << "Examples:" << std::endl;
            std::cerr << "  Dual arm: " << argv[0] << " /path/to/openarm.urdf dual can1 can0 5678" << std::endl;
            std::cerr << "  Single left: " << argv[0] << " /path/to/openarm.urdf single_left can1 5678" << std::endl;
            std::cerr << "  Single right: " << argv[0] << " /path/to/openarm.urdf single_right can0 5678" << std::endl;
            return 1;
        }

        // Required: URDF path
        urdf_path = argv[1];

        // Optional: arm mode
        if (argc >= 3) {
            arm_mode = argv[2];
            if (arm_mode != "dual" && arm_mode != "single_left" && arm_mode != "single_right") {
                std::cerr << "[ERROR] Invalid arm_mode: " << arm_mode
                          << ". Must be 'dual', 'single_left', or 'single_right'." << std::endl;
                return 1;
            }
        }

        // Optional: CAN interfaces
        if (arm_mode == "dual") {
            if (argc >= 4) {
                can_interface1 = argv[3];
            }
            if (argc >= 5) {
                can_interface2 = argv[4];
            }
            if (argc >= 6) {
                udp_port = std::stoi(argv[5]);
            }
        } else {
            // Single arm mode
            if (argc >= 4) {
                can_interface1 = argv[3];
            }
            if (argc >= 5) {
                udp_port = std::stoi(argv[4]);
            }
        }

        // Check URDF file exists
        if (!std::filesystem::exists(urdf_path)) {
            std::cerr << "[ERROR] URDF file not found: " << urdf_path << std::endl;
            return 1;
        }

        // Set up Manus
        initialize_manus_sdk();

        // Determine which arms to initialize
        bool use_left_arm = (arm_mode == "dual" || arm_mode == "single_left");
        bool use_right_arm = (arm_mode == "dual" || arm_mode == "single_right");

        // Setup dynamics for left arm if needed
        Dynamics* left_arm_dynamics = nullptr;
        if (use_left_arm) {
            std::string root_link = "openarm_body_link0";
            std::string left_leaf_link = "openarm_left_hand";

            std::cout << "[INFO] Initializing dynamics model for left arm..." << std::endl;
            left_arm_dynamics = new Dynamics(urdf_path, root_link, left_leaf_link);
            if (!left_arm_dynamics->Init()) {
                std::cerr << "[ERROR] Failed to initialize dynamics model for left arm" << std::endl;
                return 1;
            }
            std::cout << "[INFO] Left arm dynamics model initialized" << std::endl;
        }

        // Setup dynamics for right arm if needed
        Dynamics* right_arm_dynamics = nullptr;
        if (use_right_arm) {
            std::string root_link = "openarm_body_link0";
            std::string right_leaf_link = "openarm_right_hand";

            std::cout << "[INFO] Initializing dynamics model for right arm..." << std::endl;
            right_arm_dynamics = new Dynamics(urdf_path, root_link, right_leaf_link);
            if (!right_arm_dynamics->Init()) {
                std::cerr << "[ERROR] Failed to initialize dynamics model for right arm" << std::endl;
                return 1;
            }
            std::cout << "[INFO] Right arm dynamics model initialized" << std::endl;
        }

        // Initialize UDP receiver
        std::cout << "\n[INFO] Starting UDP receiver on port " << udp_port << "..." << std::endl;
        std::cout << "[INFO] Make sure joint data sender is running!" << std::endl;
        UdpJointReceiver udp_receiver(udp_port, 7);  // 7 joints for OpenArm

        // Initialize OpenArm hardware
        openarm::can::socket::OpenArm* left_openarm = nullptr;
        openarm::can::socket::OpenArm* right_openarm = nullptr;
        size_t left_arm_motor_num = 0, left_hand_motor_num = 0;
        size_t right_arm_motor_num = 0, right_hand_motor_num = 0;

        if (use_left_arm) {
            std::cout << "\n[INFO] Initializing OpenArm hardware for left arm on " << can_interface1 << "..."
                      << std::endl;
            left_openarm = openarm_init::OpenArmInitializer::initialize_openarm(can_interface1, true);
            left_arm_motor_num = left_openarm->get_arm().get_motors().size();
            left_hand_motor_num = left_openarm->get_gripper().get_motors().size();
            std::cout << "[INFO] Left arm motors  : " << left_arm_motor_num << std::endl;
            std::cout << "[INFO] Left hand motors : " << left_hand_motor_num << std::endl;
        }

        if (use_right_arm) {
            std::string right_can = (arm_mode == "dual") ? can_interface2 : can_interface1;
            std::cout << "\n[INFO] Initializing OpenArm hardware for right arm on " << right_can << "..."
                      << std::endl;
            right_openarm = openarm_init::OpenArmInitializer::initialize_openarm(right_can, true);
            right_arm_motor_num = right_openarm->get_arm().get_motors().size();
            right_hand_motor_num = right_openarm->get_gripper().get_motors().size();
            std::cout << "[INFO] Right arm motors  : " << right_arm_motor_num << std::endl;
            std::cout << "[INFO] Right hand motors : " << right_hand_motor_num << std::endl;
        }

        // Create robot states
        std::shared_ptr<RobotSystemState> left_robot_state = nullptr;
        std::shared_ptr<RobotSystemState> right_robot_state = nullptr;

        if (use_left_arm) {
            left_robot_state = std::make_shared<RobotSystemState>(left_arm_motor_num, left_hand_motor_num);
        }
        if (use_right_arm) {
            right_robot_state = std::make_shared<RobotSystemState>(right_arm_motor_num, right_hand_motor_num);
        }

        // Create control instances
        Control* left_control = nullptr;
        Control* right_control = nullptr;

        if (use_left_arm) {
            left_control = new Control(left_openarm, left_arm_dynamics, left_arm_dynamics, left_robot_state,
                                       1.0 / FOLLOW_FREQUENCY, ROLE_FOLLOWER, "left_arm", left_arm_motor_num,
                                       left_hand_motor_num);
        }
        if (use_right_arm) {
            right_control = new Control(right_openarm, right_arm_dynamics, right_arm_dynamics, right_robot_state,
                                        1.0 / FOLLOW_FREQUENCY, ROLE_FOLLOWER, "right_arm", right_arm_motor_num,
                                        right_hand_motor_num);
        }

        // Load control parameters from YAML and set to controllers
        std::cout << "\n[INFO] Loading control parameters..." << std::endl;
        YamlLoader loader("config/follower.yaml");
        std::vector<double> l_kp = loader.get_vector_by_two_levels("FollowerArmParam", "Left", "Kp");
        std::vector<double> l_kd = loader.get_vector_by_two_levels("FollowerArmParam", "Left", "Kd");
        std::vector<double> l_Fc = loader.get_vector_by_two_levels("FollowerArmParam", "Left", "Fc");
        std::vector<double> l_k = loader.get_vector_by_two_levels("FollowerArmParam", "Left", "k");
        std::vector<double> l_Fv = loader.get_vector_by_two_levels("FollowerArmParam", "Left", "Fv");
        std::vector<double> l_Fo = loader.get_vector_by_two_levels("FollowerArmParam", "Left", "Fo");
        std::vector<double> r_kp = loader.get_vector_by_two_levels("FollowerArmParam", "Right", "Kp");
        std::vector<double> r_kd = loader.get_vector_by_two_levels("FollowerArmParam", "Right", "Kd");
        std::vector<double> r_Fc = loader.get_vector_by_two_levels("FollowerArmParam", "Right", "Fc");
        std::vector<double> r_k = loader.get_vector_by_two_levels("FollowerArmParam", "Right", "k");
        std::vector<double> r_Fv = loader.get_vector_by_two_levels("FollowerArmParam", "Right", "Fv");
        std::vector<double> r_Fo = loader.get_vector_by_two_levels("FollowerArmParam", "Right", "Fo");
        if (left_control) {
            left_control->SetParameter(l_kp, l_kd, l_Fc, l_k, l_Fv, l_Fo);
        }
        if (right_control) {
            right_control->SetParameter(r_kp, r_kd, r_Fc, r_k, r_Fv, r_Fo);
        }
        std::cout << "[INFO] Control parameters loaded" << std::endl;

        // Load UDP receiver filter parameters
        std::vector<double> filter_alphas = loader.get_vector("UdpReceiverFilter", "FilterAlphas");
        std::cout << "[INFO] UDP receiver filter parameters loaded" << std::endl;
        UdpReceiverThread udp_thread(left_robot_state, right_robot_state, &udp_receiver, UPD_RECEIVER_FREQUENCY, filter_alphas);
        udp_thread.start_thread();
        std::cout << "Receiving joint angles via UDP on port " << udp_port << std::endl;

        // Insure initial position is received and static before starting control
        std::cout << "\n[INFO] Start initialize home position, please don't move arms in 6 secs........" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Initialize home position
        std::cout << "\n[INFO] Moving to home position..." << std::endl;
        if (left_control) {
            left_control->AdjustPosition();
        }
        if (right_control) {
            right_control->AdjustPosition();
        }
        std::cout << "[INFO] Home positions reached" << std::endl;


        // Create and start control threads
        std::cout << "\n[INFO] Starting control threads..." << std::endl;
        FollowerArmThread* left_follower_thread = nullptr;
        FollowerArmThread* right_follower_thread = nullptr;
        if (left_control) {
            left_follower_thread = new FollowerArmThread(left_robot_state, left_control, FOLLOW_FREQUENCY);
            left_follower_thread->start_thread();
        }
        if (right_control) {
            right_follower_thread = new FollowerArmThread(right_robot_state, right_control, FOLLOW_FREQUENCY);
            right_follower_thread->start_thread();
        }


        // Main loop - just wait for interrupt
        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Shutdown sequence
        udp_thread.stop_thread();
        if (left_follower_thread) {
            left_follower_thread->stop_thread();
            delete left_follower_thread;
        }
        if (right_follower_thread) {
            right_follower_thread->stop_thread();
            delete right_follower_thread;
        }
        if (left_openarm) {
            left_openarm->disable_all();
        }
        if (right_openarm) {
            right_openarm->disable_all();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        shutdown_manus_sdk();
        std::cout << "[INFO] Shutdown complete" << std::endl;

        // Cleanup
        delete left_control;
        delete right_control;
        delete left_arm_dynamics;
        delete right_arm_dynamics;

    } catch (const std::exception& e) {
        std::cerr << "\n❌ Fatal error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
