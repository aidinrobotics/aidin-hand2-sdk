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
  // Effort is 0.1 % of rated, 1000 = 100%
  // Fields after damping default to plain PD
  struct JointImpedanceController {
    // Effort per count
    std::array<double, kActuatorCount> stiffness{
        0.02, 0.02, 0.02, 0.02, 0.01, 0.01, 0.02, 0.01,
        0.01, 0.02, 0.01, 0.01, 0.02, 0.01, 0.01, 0.02};

    // Effort per count/s
    std::array<double, kActuatorCount> damping{
        1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5,
        1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5};

    // Effort cap on the stiffness term, 0 = no cap
    double p_saturation_effort{0.0};

    // Velocity from velocity_rpm, not the encoder difference
    bool velocity_from_rpm{false};

    // Hz, velocity low-pass, 0 = no filter
    double velocity_cutoff_hz{0.0};

    // Pairs in order: thumb, index, middle, ring, baby actuator1..2
    // Each actuator adds kx times the partner's error, |kx| < stiffness
    // cross_damping does the same with the partner's velocity
    std::array<double, 5> cross_stiffness{};
    std::array<double, 5> cross_damping{};

    // Friction compensation, friction_scale 0 = off
    // c+ resists motion toward +count, c- toward -count
    enum class FrictionModel : int { Coulomb = 0, PositionTable = 1 };
    static constexpr std::size_t kFrictionTableBins = 8;
    FrictionModel friction_model{FrictionModel::Coulomb};

    // Effort, c+ and c- of Coulomb
    std::array<double, kActuatorCount> friction_coulomb_positive{};
    std::array<double, kActuatorCount> friction_coulomb_negative{};

    // Effort per bin, c+ and c- of PositionTable
    // Bins span friction_table_start..end in count
    std::array<std::array<double, kFrictionTableBins>, kActuatorCount> friction_table_positive{};
    std::array<std::array<double, kFrictionTableBins>, kActuatorCount> friction_table_negative{};
    std::array<double, kActuatorCount> friction_table_start{};
    std::array<double, kActuatorCount> friction_table_end{};

    // Effort per count/s
    std::array<double, kActuatorCount> friction_viscous{};

    // count/s, below it the impedance effort sets the direction
    double friction_stick_velocity{300.0};

    // Effort for full compensation at rest, at least 2 * max(c+, c-)
    double friction_stick_ramp{300.0};
    double friction_scale{0.0};

    // Apply the bias (c+ - c-) / 2 at rest too
    bool friction_bias_at_rest{false};

    // count/s, braking starts above it, 0 = no brake
    double velocity_soft_limit{0.0};

    // Effort per count/s over velocity_soft_limit
    double velocity_brake_gain{0.0};
  };

  JointPositionController  joint_position_controller{};
  JointImpedanceController joint_impedance_controller{};
};

}  // namespace aidin_hand2
