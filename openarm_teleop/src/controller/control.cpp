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

#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <thread>

Control::Control(openarm::can::socket::OpenArm* arm, Dynamics* dynamics_l, Dynamics* dynamics_f,
                 std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
                 std::string arm_type, size_t arm_motor_num, size_t hand_motor_num, 
                 std::vector<double> filter_alphas=std::vector<double>(7, 0.1))
    : openarm_(arm),
      dynamics_l_(dynamics_l),
      dynamics_f_(dynamics_f),
      robot_state_(robot_state),
      Ts_(Ts),
      role_(role),
      arm_motor_num_(arm_motor_num),
      hand_motor_num_(hand_motor_num) {
    differentiator_ = new Differentiator(Ts);
    openarmjointconverter_ = new OpenArmJointConverter(arm_motor_num_);
    openarmgripperjointconverter_ = new OpenArmJGripperJointConverter(hand_motor_num_);

    arm_type_ = arm_type;

    // Initialize filters with individual alphas for each joint
    for (size_t i = 0; i < filter_alphas.size(); ++i) {
        filters_.push_back(LowPassFilter(filter_alphas[i]));
    }
    
        // Open joint writer file
    if (arm_type_ == "left_arm") joint_writer_.open("joint_angles_left.txt", std::ios::out);
    else joint_writer_.open("joint_angles_right.txt", std::ios::out);
    if (joint_writer_.is_open()) {
        joint_writer_ << "timestamp,arm_ref_0,arm_ref_1,arm_ref_2,arm_ref_3,arm_ref_4,arm_ref_5,arm_ref_6,"
                      << "arm_cur_0,arm_cur_1,arm_cur_2,arm_cur_3,arm_cur_4,arm_cur_5,arm_cur_6,"
                      << "hand_ref_0,hand_cur_0" << std::endl;
    }
}

Control::~Control() {
    std::cout << "Control destructed " << std::endl;
    if (joint_writer_.is_open()) {
        joint_writer_.close();
    }
    delete openarmjointconverter_;
    delete differentiator_;
}

void Control::Shutdown(void) {
    std::cout << "control shutdown !!!" << std::endl;

    openarm_->disable_all();
}

void Control::SetParameter(const std::vector<double>& Kp, const std::vector<double>& Kd,
                           const std::vector<double>& Fc, const std::vector<double>& k,
                           const std::vector<double>& Fv, const std::vector<double>& Fo) {
    // Real values
    Kp_ = Kp;
    Kd_ = Kd;
    Fc_ = Fc;
    k_ = k;
    Fv_ = Fv;
    Fo_ = Fo;

    // Soft values
    for (double kp: Kp) {
        soft_kp_.push_back(kp * init_ratio_);
    }
    // Calculate increment step
    incre_step_ratio_ = (1.0 - init_ratio_) / (init_time_ / Ts_);             
}

bool Control::unilateral_step() {
    // get motor status
    std::vector<MotorState> arm_motor_states;
    for (const auto& motor : openarm_->get_arm().get_motors()) {
        arm_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<MotorState> gripper_motor_states;
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
        gripper_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    // convert joint to motor
    std::vector<JointState> joint_arm_states =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_gripper_states =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    // set reponse
    robot_state_->arm_state().set_all_responses(joint_arm_states);
    robot_state_->hand_state().set_all_responses(joint_gripper_states);

    size_t arm_dof = robot_state_->arm_state().get_size();
    size_t gripper_dof = robot_state_->hand_state().get_size();

    std::vector<double> joint_arm_positions(arm_dof, 0.0);
    std::vector<double> joint_arm_velocities(arm_dof, 0.0);
    std::vector<double> joint_gripper_positions(gripper_dof, 0.0);
    std::vector<double> joint_gripper_velocities(gripper_dof, 0.0);

    for (size_t i = 0; i < arm_dof; ++i) {
        joint_arm_positions[i] = joint_arm_states[i].position;
        joint_arm_velocities[i] = joint_arm_states[i].velocity;
    }

    for (size_t i = 0; i < gripper_dof; ++i) {
        joint_gripper_positions[i] = joint_gripper_states[i].position;
        joint_gripper_velocities[i] = joint_gripper_states[i].velocity;
    }

    std::vector<double> gravity(arm_dof, 0.0);
    std::vector<double> coriolis(arm_dof, 0.0);
    std::vector<double> friction(arm_dof + gripper_dof, 0.0);

    if (role_ == ROLE_LEADER) {} 
    else if (role_ == ROLE_FOLLOWER) {
        // calc dynamics for gravity and friction compensation
        dynamics_f_->GetGravity(joint_arm_positions.data(), gravity.data());
        dynamics_f_->GetCoriolis(joint_arm_positions.data(), joint_arm_velocities.data(), coriolis.data());

        // Friction (compute joint friction)
        for (size_t i = 0; i < joint_arm_velocities.size(); ++i)
            ComputeFriction(joint_arm_velocities.data(), friction.data(), i);

        std::vector<JointState> joint_arm_states_ref =
            robot_state_->arm_state().get_all_references();
        std::vector<JointState> joint_hand_states_ref =
            robot_state_->hand_state().get_all_references();

        // Apply low-pass filter to joint angles
        for (size_t i = 0; i < joint_arm_states_ref.size(); ++i) {
            joint_arm_states_ref[i].position = filters_[i].update(joint_arm_states_ref[i].position);
        }

        // **Just Test Torque**
        // joint_arm_states_ref = joint_arm_states;
        // joint_hand_states_ref = joint_gripper_states;
        // // set gravity and friction comp joint torque value
        // for (size_t i = 0; i < arm_dof; i++) {
        //     if (arm_type_ == "left_arm") {
        //         if(i < 3) {
        //             joint_arm_states_ref[i].effort = gravity[i] + friction[i];
        //         }
        //     } else {
        //         if (i < 4) {
        //             if (i != 2) joint_arm_states_ref[i].effort = gravity[i] + friction[i];
        //             else joint_arm_states_ref[i].effort = friction[i];
        //         }
        //     }
        // }
        
        // Joint → Motor
        std::vector<MotorState> arm_motor_refs =
        openarmjointconverter_->joint_to_motor(joint_arm_states_ref);
        std::vector<MotorState> hand_motor_refs =
        openarmgripperjointconverter_->joint_to_motor(joint_hand_states_ref);
        

        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        for (size_t i = 0; i < arm_motor_refs.size(); ++i) {
            arm_cmds.emplace_back(openarm::damiao_motor::MITParam{soft_kp_[i], Kd_[i],
                                                                   arm_motor_refs[i].position,
                                                                   arm_motor_refs[i].velocity,
                                                                   arm_motor_refs[i].effort});
        }

        std::vector<openarm::damiao_motor::MITParam> hand_cmds;
        hand_cmds.reserve(hand_motor_refs.size());
        for (size_t i = 0; i < hand_motor_refs.size(); ++i) {
            hand_cmds.emplace_back(openarm::damiao_motor::MITParam{
                soft_kp_[arm_dof+i], Kd_[arm_dof+i], hand_motor_refs[i].position,
                hand_motor_refs[i].velocity, hand_motor_refs[i].effort});
        }

        // Update soft kp for next step
        if (soft_kp_[0] < Kp_[0]) {
            for (size_t i = 0; i < soft_kp_.size(); ++i) {
                soft_kp_[i] += Kp_[i] * incre_step_ratio_;
                if (soft_kp_[i] > Kp_[i]) {
                    soft_kp_[i] = Kp_[i];
                }
            }
        }


        // Write joint angles to file
        if (debug) {
            static int count = 0;
            ++count;
            if (count % 10 == 0) {
                write_joint_angles_to_file(joint_arm_states_ref, joint_arm_states, joint_hand_states_ref, joint_gripper_states);
            }
            if (count % 500 == 0) {
                if (arm_type_  == "left_arm") std::cout << "[Follower Left Arm] " << std::endl;
                else std::cout << "[Follower Right Arm] " << std::endl;
                std::cout << "[Follower] Joint Pos Ref: ";
                for (const auto& joint : joint_arm_states_ref) {
                    std::cout << std::fixed << std::setprecision(2) << joint.position << " ";
                }
                std::cout << std::endl;
                std::cout << "[Follower] Motor Pos Now: ";
                for (const auto& motor : arm_motor_states) {
                    std::cout << std::fixed << std::setprecision(2) << motor.position << " ";
                }
            }
        }

        // openarm_->get_arm().mit_control_all(arm_cmds);
        // openarm_->get_gripper().mit_control_all(hand_cmds);

        openarm_->recv_all(200);

        return true;
    }

    return true;
}

void Control::ComputeFriction(const double* velocity, double* friction, size_t index) {
    if (TANHFRIC) {
        const double amp_tmp = 1.0;
        const double coef_tmp = 0.1;

        const double v = velocity[index];
        const double Fc = Fc_.at(index);
        const double k = k_.at(index);
        const double Fv = Fv_.at(index);
        const double Fo = Fo_.at(index);

        friction[index] = amp_tmp * Fc * std::tanh(coef_tmp * k * v) + Fv * v + Fo;
    } else {
        friction[index] = velocity[index] * Dn_.at(index);
    }
}

bool Control::AdjustPosition(void) {
    int nstep = 400;
    double alpha;

    std::vector<MotorState> arm_motor_states;
    for (const auto& motor : openarm_->get_arm().get_motors()) {
        arm_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<MotorState> gripper_motor_states;
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
        gripper_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<JointState> joint_arm_now =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_hand_now =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    std::vector<JointState> joint_arm_states_ref_init =
        robot_state_->arm_state().get_all_references();
    std::vector<JointState> joint_hand_states_ref_init =
        robot_state_->hand_state().get_all_references();
        
    std::vector<JointState> joint_arm_goal(NMOTORS - 1);
    for (size_t i = 0; i < NMOTORS - 1; ++i) {
        joint_arm_goal[i].position = joint_arm_states_ref_init[i].position;
        joint_arm_goal[i].velocity = 0.0;
        joint_arm_goal[i].effort = 0.0;
    }

    std::vector<JointState> joint_hand_goal(joint_hand_now.size());
    for (size_t i = 0; i < joint_hand_goal.size(); ++i) {
        joint_hand_goal[i].position = joint_hand_states_ref_init[i].position;
        joint_hand_goal[i].velocity = 0.0;
        joint_hand_goal[i].effort = 0.0;
    }

    // std::vector<double> kp_arm_temp = {50, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
    // std::vector<double> kp_arm_temp = {20.0, 30.0, 20.0, 20.0, 5.0, 5.0, 5.0, 3.0};
    std::vector<double> kp_arm_temp;
    for (size_t i = 0; i < NMOTORS - 1; ++i) {
        kp_arm_temp.push_back(soft_kp_[i]);
    }
    std::vector<double> kd_arm_temp = {1.2, 1.2, 1.2, 1.2, 0.3, 0.2, 0.3};

    std::vector<double> kp_hand_temp = {10.0};
    std::vector<double> kd_hand_temp = {0.5};

    for (int step = 0; step < nstep; ++step) {
        alpha = static_cast<double>(step + 1) / nstep;

        std::vector<JointState> joint_arm_interp(NMOTORS - 1);
        for (size_t i = 0; i < NMOTORS - 1; ++i) {
            joint_arm_interp[i].position =
                joint_arm_goal[i].position * alpha + joint_arm_now[i].position * (1.0 - alpha);
            joint_arm_interp[i].velocity = 0.0;
        }

        std::vector<JointState> joint_hand_interp(joint_hand_goal.size());
        for (size_t i = 0; i < joint_hand_interp.size(); ++i) {
            joint_hand_interp[i].position =
                joint_hand_goal[i].position * alpha + joint_hand_now[i].position * (1.0 - alpha);
            joint_hand_interp[i].velocity = 0.0;
        }

        std::vector<MotorState> arm_motor_refs =
            openarmjointconverter_->joint_to_motor(joint_arm_interp);
        std::vector<MotorState> hand_motor_refs =
            openarmgripperjointconverter_->joint_to_motor(joint_hand_interp);

        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(arm_motor_refs.size());
        for (size_t i = 0; i < arm_motor_refs.size(); ++i) {
            arm_cmds.emplace_back(openarm::damiao_motor::MITParam{kp_arm_temp[i], kd_arm_temp[i],
                                                                  arm_motor_refs[i].position,
                                                                  arm_motor_refs[i].velocity, 0.0});
        }
        
        std::vector<openarm::damiao_motor::MITParam> hand_cmds;
        hand_cmds.reserve(hand_motor_refs.size());
        for (size_t i = 0; i < hand_motor_refs.size(); ++i) {
            hand_cmds.emplace_back(openarm::damiao_motor::MITParam{
                kp_hand_temp[i], kd_hand_temp[i], hand_motor_refs[i].position,
                hand_motor_refs[i].velocity, 0.0});
        }

        // openarm_->get_arm().mit_control_all(arm_cmds);
        // openarm_->get_gripper().mit_control_all(hand_cmds);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        openarm_->recv_all();
    }

    std::vector<MotorState> arm_motor_states_final;
    for (const auto& motor : openarm_->get_arm().get_motors()) {
        arm_motor_states_final.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<MotorState> gripper_motor_states_final;
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
        gripper_motor_states_final.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<JointState> joint_arm_final =
        openarmjointconverter_->motor_to_joint(arm_motor_states_final);
    std::vector<JointState> joint_hand_final =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states_final);

    robot_state_->arm_state().set_all_references(joint_arm_final);
    robot_state_->hand_state().set_all_references(joint_hand_final);

    return true;
}

bool Control::DetectVibration(const double* velocity, bool* what_axis) {
    bool vibration_detected = false;

    for (int i = 0; i < NJOINTS; ++i) {
        what_axis[i] = false;

        velocity_buffer_[i].push_back(velocity[i]);
        if (velocity_buffer_[i].size() > VEL_WINDOW_SIZE) velocity_buffer_[i].pop_front();

        if (velocity_buffer_[i].size() < VEL_WINDOW_SIZE) continue;

        double mean = std::accumulate(velocity_buffer_[i].begin(), velocity_buffer_[i].end(), 0.0) /
                      velocity_buffer_[i].size();

        double var = 0.0;
        for (double v : velocity_buffer_[i]) {
            var += (v - mean) * (v - mean);
        }

        double stddev = std::sqrt(var / velocity_buffer_[i].size());

        if (stddev > VIB_THRESHOLD) {
            what_axis[i] = true;
            vibration_detected = true;
            std::cout << "[VIBRATION] Joint " << i << " stddev: " << stddev << std::endl;
        }
    }

    return vibration_detected;
}

void Control::write_joint_angles_to_file(const std::vector<JointState>& arm_ref, const std::vector<JointState>& arm_current,
                                         const std::vector<JointState>& hand_ref, const std::vector<JointState>& hand_current) {
    if (!joint_writer_.is_open()) return;

    static long long timestamp = 0;
    timestamp++;

    joint_writer_ << timestamp;

    // Write arm reference positions
    for (const auto& joint : arm_ref) {
        joint_writer_ << "," << std::fixed << std::setprecision(6) << joint.position;
    }

    // Write arm current positions
    for (const auto& joint : arm_current) {    
        joint_writer_ << "," << std::fixed << std::setprecision(6) << joint.position;
    }

    // // Write hand reference and current positions
    // if (!hand_ref.empty()) {
    //     joint_writer_ << "," << std::fixed << std::setprecision(6) << hand_ref[0].position;
    // } else {
    //     joint_writer_ << ",0.0";
    // }

    // if (!hand_current.empty()) {
    //     joint_writer_ << "," << std::fixed << std::setprecision(6) << hand_current[0].position;
    // } else {
    //     joint_writer_ << ",0.0";
    // }

    joint_writer_ << std::endl;
}
