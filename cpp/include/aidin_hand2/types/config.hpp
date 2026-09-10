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
    std::array<double, kActuatorCount> stiffness{
        0.02, 0.02, 0.02, 0.02, 0.01, 0.01, 0.02, 0.01,
        0.01, 0.02, 0.01, 0.01, 0.02, 0.01, 0.01, 0.02};
    std::array<double, kActuatorCount> damping{
        1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5,
        1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5};
  };

  JointPositionController  joint_position_controller{};
  JointImpedanceController joint_impedance_controller{};
};

}  // namespace aidin_hand2
