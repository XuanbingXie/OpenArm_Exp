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

#define LIMIT(x, min, max)                                                     \
  if (x < min) {                                                               \
    x = min;                                                                   \
  } else if (x > max) {                                                        \
    x = max;                                                                   \
  }

// #include <sensor_msgs/msg/joint_state.hpp>
#include <controller/diff.hpp>
#include <controller/dynamics.hpp>
#include <deque>
#include <fstream>
#include <joint_state_converter.hpp>
#include <string>
#include <memory>
#include <numeric>
#include <vector>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_constants.hpp>
#include <robot_state.hpp>
#include <utility>

class PID {
public:
  PID(double min, double max);

  double update(double cur_error);

  double limit(double value) {
    LIMIT(value, min_, max_);
    return value;
  }
  
private:
  double out_{0};
  double error_{0};
  double last_error_{0};
  double error_sum_{0};
  double proportion_{0.4};
  double integral_{0.000000013};
  double differential_{2.1};
  double min_;
  double max_;
};

class Control {
    openarm::can::socket::OpenArm *openarm_;

    double Ts_;
    int role_;

    size_t arm_motor_num_;
    size_t hand_motor_num_;

    Differentiator *differentiator_;
    OpenArmJointConverter *openarmjointconverter_;
    OpenArmJGripperJointConverter *openarmgripperjointconverter_;

    std::shared_ptr<RobotSystemState> robot_state_;

    std::string arm_type_;

    Dynamics *dynamics_f_;
    Dynamics *dynamics_l_;

    double oblique_coordinates_force;
    double oblique_coordinates_position;

    // for easy logging
    // std::vector<std::pair<double, double>> velocity_log_;  // (differ_velocity, motor_velocity)
    // std::string log_file_path_ = "../data/velocity_comparison.csv";
    static constexpr int VEL_WINDOW_SIZE = 10;
    static constexpr double VIB_THRESHOLD = 0.7;  // [rad/s]
    std::deque<double> velocity_buffer_[NJOINTS];
    
public:
    Control(openarm::can::socket::OpenArm *arm, Dynamics *dynamics_l, Dynamics *dynamics_f,
            std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
            size_t arm_joint_num, size_t hand_motor_num);
    Control(openarm::can::socket::OpenArm *arm, Dynamics *dynamics_l, Dynamics *dynamics_f,
            std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
            std::string arm_type, size_t arm_joint_num, size_t hand_motor_num);
    ~Control();

    std::shared_ptr<RobotSystemState> response_;
    std::shared_ptr<RobotSystemState> reference_;

    std::vector<double> Dn_, Kp_, Kd_, Fc_, k_, Fv_, Fo_;

    std::vector<std::unique_ptr<PID>> joint_angle_pids_;

    // bool Setup(void);
    void Setstate(int state);
    void Shutdown(void);

    void SetParameter(const std::vector<double> &Kp, const std::vector<double> &Kd,
                      const std::vector<double> &Fc, const std::vector<double> &k,
                      const std::vector<double> &Fv, const std::vector<double> &Fo);

    bool AdjustPosition(void);

    // Compute torque based on bilateral
    bool bilateral_step();
    bool unilateral_step();

    // NOTE! Control() class operates on "joints", while the underlying
    // classes operates on "actuators". The following functions map
    // joints to actuators.

    void ComputeJointPosition(const double *motor_position, double *joint_position);
    void ComputeJointVelocity(const double *motor_velocity, double *joint_velocity);
    void ComputeMotorTorque(const double *joint_torque, double *motor_torque);

    // void ComputeFriction(const double *velocity, double *friction);
    void ComputeFriction(const double *velocity, double *friction, size_t index);
    void ComputeGravity(const double *position, double *gravity);
    bool DetectVibration(const double *velocity, bool *what_axis);

    // For debug
    // Write joint angles to file
    bool debug_{false};
    void write_joint_angles_to_file(const std::vector<JointState>& arm_ref, const std::vector<JointState>& arm_current,
                                     const std::vector<JointState>& hand_ref, const std::vector<JointState>& hand_current);

    // Joint angle writer
    std::ofstream joint_writer_;

};
