#pragma once

#include <array>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>
#include "hand_core/comms/canfd/protocol.hpp"

namespace aidin_hand2::controllers
{

// Row-major 16 x 16
using JointJacobian = std::array<double, kActiveJointCount * kActuatorCount>;

// N·m per active joint, K * (target - position) - 2 * damping ratio * sqrt(K * inertia) * velocity
// K is the hard stiffness times stiffness (0..1); count_per_joint is count per rad, row actuator
std::array<double, kActiveJointCount> joint_impedance_torque(
    const std::array<double, kActiveJointCount>& target, const std::array<double, kActiveJointCount>& position,
    const std::array<double, kActiveJointCount>& velocity, const std::array<double, kActiveJointCount>& stiffness,
    const JointJacobian& count_per_joint);

// Effort per actuator for joint torques, before the max_effort clamp; joint_per_count is rad per count, row joint
std::array<double, kActuatorCount> joint_torque_to_effort(
    const std::array<double, kActiveJointCount>& torque, const JointJacobian& joint_per_count);

// Joint space impedance: joint_impedance_torque at the FK pose, then joint_torque_to_effort, sent as CST
// The target passes a low-pass and a 250 deg/s limit and keeps off the hard stop; hand type c sends 0
class JointImpedanceController
{
 public:
  explicit JointImpedanceController(long period_nanoseconds);

  // Fills mode_of_operation and target_effort in frames
  void compute(const JointImpedanceCommand& command,
               const ControllerConfig::JointImpedanceController& config, const HandState& state,
               const std::array<double, kActuatorCount>& max_effort, canfd::CommandFrames& frames);

  // Next entry starts the target from the measured pose and the stiffness at its setting
  void reset();

 private:
  // rad, the command target through the low-pass, the velocity limit and the hard stop margin
  std::array<double, kActiveJointCount> filtered_target(
      const std::array<double, kActiveJointCount>& command_target,
      const ControllerConfig::JointImpedanceController& config,
      const std::array<double, kActiveJointCount>& measured);

  const double dt_seconds_;

  // Per cycle share of the way to a new stiffness
  const double stiffness_alpha_;

  // The cutoff alpha_ was derived from, -1 forcing the first derivation
  double cutoff_freq_{-1.0};
  double alpha_{1.0};

  // rad, and a NaN in target_ means unset, so the next compute seeds it and section_ from the measured pose
  std::array<double, kActiveJointCount> target_{};
  std::array<std::array<double, 3>, kActiveJointCount> section_{};

  // 0..1, and a NaN means unset, so the next compute starts at the setting
  std::array<double, kActiveJointCount> stiffness_{};
};

}  // namespace aidin_hand2::controllers
