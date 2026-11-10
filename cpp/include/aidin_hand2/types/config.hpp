// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/error.hpp>

namespace aidin_hand2
{

// ------------------------------- Hand config --------------------------------

struct HandConfig {
  HandConfig(std::string interface_name, HandSide hand_side)
  : interface_name(std::move(interface_name)), hand_side(hand_side)
  {
    if (this->interface_name.empty()) {
      throw Exception(ErrorCode::InvalidArgument, "HandConfig: interface_name must not be empty");
    }
    if (hand_side != HandSide::Left && hand_side != HandSide::Right) {
      throw Exception(ErrorCode::InvalidArgument,
                      "HandConfig: hand_side must be HandSide::Left or HandSide::Right");
    }
  }

  // "can0", or "auto" to scan for this hand's channel
  std::string interface_name;
  HandSide    hand_side;

  // Hz
  int control_rate{500};
  bool auto_home{true};

  // -1 = unset
  int rt_cpu_affinity{-1};
  bool auto_reconnect{false};

  // 0 = no limit
  int auto_reconnect_timeout_ms{0};
  bool auto_reconnect_home{false};
  std::vector<int> disabled_actuators{};
};

// ---------------------------- Controller config -----------------------------

// Runtime settable with set_controller_config()
struct ControllerConfig {
  struct JointPositionController {
    bool filter_enabled{true};

    // Hz
    double cutoff_freq{10.0};

    // rad
    double deadband{0.000873};
  };
  struct JointImpedanceController {
    // Per active joint, in the order of JointImpedanceCommand::target
    // 0..1, 1 = the joint's hard stiffness; the damping follows from it at a fixed damping ratio
    std::array<double, kActiveJointCount> stiffness{
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    // Target low-pass, the joint position controller's without the deadband
    bool filter_enabled{true};

    // Hz
    double cutoff_freq{10.0};

    // Per actuator on the motor shaft: rotor, gearhead, encoder, gears and screw
    std::array<double, kActuatorCount> motor_inertia_kgm2{
        3.3e-9, 3.3e-9, 3.3e-9, 3.3e-9, 3.3e-9, 3.3e-9, 3.3e-9, 3.3e-9,
        3.3e-9, 3.3e-9, 3.3e-9, 3.3e-9, 3.3e-9, 3.3e-9, 3.3e-9, 3.3e-9};
    // Per active joint, added to the motor inertia seen at the joint
    std::array<double, kActiveJointCount> link_inertia_kgm2{};
    // Share of the motor torque that reaches the joint, 0 < efficiency <= 1
    std::array<double, kActuatorCount> efficiency{
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  };

  JointPositionController  joint_position_controller{};
  JointImpedanceController joint_impedance_controller{};
};

}  // namespace aidin_hand2
