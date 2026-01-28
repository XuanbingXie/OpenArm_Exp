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
    }
}


// Thread to read joint data from UDP
class UdpReceiverThread : public PeriodicTimerThread {
public:
    UdpReceiverThread(std::shared_ptr<RobotSystemState> robot_state, UdpJointReceiver* receiver,
                      double hz = 100.0, std::vector<double> filter_alphas = std::vector<double>(7, 0.1))
        : PeriodicTimerThread(hz), robot_state_(robot_state), receiver_(receiver),
          update_count_(0), hz_(hz) {
        // Initialize filters with individual alphas for each joint
        for (size_t i = 0; i < filter_alphas.size(); ++i) {
            filters_.push_back(LowPassFilter(filter_alphas[i]));
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
        std::vector<double> joint_angles;

        // Try to get new joint angles from UDP
        if (receiver_->get_joint_angles(joint_angles)) {
            // Apply low-pass filter to joint angles
            for (size_t i = 0; i < joint_angles.size(); ++i) {
                joint_angles[i] = filters_[i].update(joint_angles[i]);
            }

            // New data available - convert to JointState and update robot state
            std::vector<JointState> joint_states(joint_angles.size());
            for (size_t i = 0; i < joint_angles.size(); ++i) {
                joint_states[i].position = joint_angles[i];
                joint_states[i].velocity = 0.0;
                joint_states[i].effort = 0.0;
            }
            robot_state_->arm_state().set_all_references(joint_states);

            // Update gripper
            double gripper_tor = receiver_->get_gripper_torque();
            std::vector<JointState> gripper_states(1);
            gripper_states[0].position = 0;
            gripper_states[0].velocity = 0.0;
            gripper_states[0].effort = gripper_tor;
            robot_state_->hand_state().set_all_references(gripper_states);

            update_count_++;

            // Print status every 500 updates (~1 second at 500Hz)
            if (update_count_ % 500 == 0) {
                std::cout << "[UdpReceiverThread] Updates: " << update_count_
                          << " | Timestamp: " << receiver_->get_timestamp()
                          << " | Gripper Torque: " << gripper_tor << std::endl;
            }
        }

        // For debug
        // static std::vector<JointState> debug_joint_angles{
        //     {-0.6, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        //     {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        //     {0.0, 0.0, 0.0}};
        // robot_state_->arm_state().set_all_references(debug_joint_angles);
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    UdpJointReceiver* receiver_;
    uint64_t update_count_;
    double hz_;
    std::vector<LowPassFilter> filters_;

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
        std::string arm_side = "left_arm";
        std::string urdf_path;
        std::string can_interface = "can1";
        int udp_port = 5678;

        if (argc < 2) {
            std::cerr << "Usage: " << argv[0] << " <urdf_path> [arm_side] [can_interface] [udp_port]"
                      << std::endl;
            std::cerr << "Example: " << argv[0]
                      << " /path/to/openarm.urdf right_arm can0 5678" << std::endl;
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

        // Optional: UDP port
        if (argc >= 5) {
            udp_port = std::stoi(argv[4]);
        }

        // Check URDF file exists
        if (!std::filesystem::exists(urdf_path)) {
            std::cerr << "[ERROR] URDF file not found: " << urdf_path << std::endl;
            return 1;
        }

        // Setup dynamics
        std::string root_link = "openarm_body_link0";
        std::string leaf_link =
            (arm_side == "left_arm") ? "openarm_left_hand" : "openarm_right_hand";

        std::cout << "[INFO] Initializing dynamics model..." << std::endl;
        Dynamics* arm_dynamics = new Dynamics(urdf_path, root_link, leaf_link);
        if (!arm_dynamics->Init()) {
            std::cerr << "[ERROR] Failed to initialize dynamics model" << std::endl;
            return 1;
        }
        std::cout << "[INFO] ✅ Dynamics model initialized" << std::endl;

        // Initialize UDP receiver
        std::cout << "\n[INFO] Starting UDP receiver on port " << udp_port << "..." << std::endl;
        std::cout << "[INFO] Make sure joint data sender is running!" << std::endl;
        UdpJointReceiver udp_receiver(udp_port, arm_side, 7);  // 7 joints for OpenArm

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
                                       1.0 / FOLLOW_FREQUENCY, ROLE_FOLLOWER, arm_side, arm_motor_num,
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

        // Load UDP receiver filter parameters
        std::vector<double> filter_alphas = loader.get_vector("UdpReceiverFilter", "FilterAlphas");
        std::cout << "[INFO] ✅ UDP receiver filter parameters loaded" << std::endl;

        // Move to home position
        std::cout << "\n[INFO] Moving to home position..." << std::endl;
        control->AdjustPosition();
        std::cout << "[INFO] ✅ Home position reached" << std::endl;

        // Create and start control threads
        std::cout << "\n[INFO] Starting control threads..." << std::endl;
        UdpReceiverThread udp_thread(robot_state, &udp_receiver, UPD_RECEIVER_FREQUENCY, filter_alphas);
        FollowerArmThread follower_thread(robot_state, control, FOLLOW_FREQUENCY);

        udp_thread.start_thread();
        follower_thread.start_thread();

        std::cout << "   Receiving joint angles via UDP on port " << udp_port << std::endl;

        // Main loop - just wait for interrupt
        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Shutdown sequence
        udp_thread.stop_thread();
        follower_thread.stop_thread();
        openarm->disable_all();

        std::cout << "[INFO] Shutdown complete" << std::endl;

        // Cleanup
        delete control;
        delete arm_dynamics;

    } catch (const std::exception& e) {
        std::cerr << "\n❌ Fatal error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
