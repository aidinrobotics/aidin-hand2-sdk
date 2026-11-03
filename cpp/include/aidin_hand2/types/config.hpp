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
    // damping = 2 * damping_ratio * sqrt(stiffness * inertia), the inertia at the current pose
    std::array<double, kActiveJointCount> stiffness_Nm_per_rad{
        4.9, 26.5, 3.2, 2.5, 0.4, 2.5, 1.35, 0.4,
        2.5, 1.35, 0.4, 2.5, 1.35, 0.4, 2.5, 1.35};
    // 0..1, 1 = critical
    std::array<double, kActiveJointCount> damping_ratio{
        0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7,
        0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};

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

    // Friction models, per actuator
    // Effort is 0.1 % of rated, 1000 = 100%
    // Velocity is the drive's velocity_rpm in count/s
    // positive resists motion toward +count and negative toward -count, in effort
    // viscous is effort per count/s

    // c±
    struct CoulombFriction {
      std::array<double, kActuatorCount> positive{};
      std::array<double, kActuatorCount> negative{};
      std::array<double, kActuatorCount> viscous{};
    };

    // c± per bin, bin centres evenly over start..end in count, edge values held outside
    struct PositionTableFriction {
      static constexpr std::size_t kBins = 8;
      std::array<std::array<double, kBins>, kActuatorCount> positive{};
      std::array<std::array<double, kBins>, kActuatorCount> negative{};
      std::array<double, kActuatorCount> start{};
      std::array<double, kActuatorCount> end{};
      std::array<double, kActuatorCount> viscous{};
    };

    // c + (s - c) * exp(-(|v| / stribeck_velocity)^shape), s± the breakaway at rest
    struct StribeckFriction {
      std::array<double, kActuatorCount> positive{};
      std::array<double, kActuatorCount> negative{};
      std::array<double, kActuatorCount> static_positive{};
      std::array<double, kActuatorCount> static_negative{};
      // count/s
      std::array<double, kActuatorCount> stribeck_velocity{};
      double shape{2.0};
      std::array<double, kActuatorCount> viscous{};
    };

    // Bristle state z with the Stribeck curve g of StribeckFriction's fields
    // dz/dt = v - bristle_stiffness * |v| * z / g(v)
    // effort = bristle_stiffness * z + bristle_damping * dz/dt + viscous * v
    // bristle_damping 0 is the Dahl model
    struct LuGreFriction {
      StribeckFriction curve{};
      // Effort per count
      std::array<double, kActuatorCount> bristle_stiffness{};
      // Effort per count/s
      std::array<double, kActuatorCount> bristle_damping{};
    };

    // Friction compensation with the chosen model, friction_scale 0 = off
    enum class FrictionModel : int { Coulomb = 0, PositionTable = 1, Stribeck = 2, LuGre = 3 };
    FrictionModel friction_model{FrictionModel::Coulomb};
    double friction_scale{0.0};
    CoulombFriction coulomb_friction{};
    PositionTableFriction position_table_friction{};
    StribeckFriction stribeck_friction{};
    LuGreFriction lugre_friction{};
  };

  JointPositionController  joint_position_controller{};
  JointImpedanceController joint_impedance_controller{};
};

// ------------------------------- Friction file -------------------------------

// Reads a friction file ("aidin-hand2-friction/1") into config's Coulomb model and selects it
// friction_scale is left as it is, actuators the file does not list get zero
// Throws InvalidArgument when the file cannot be read or is not a friction file
void load_friction_file(const std::string& path, ControllerConfig::JointImpedanceController& config);

}  // namespace aidin_hand2
