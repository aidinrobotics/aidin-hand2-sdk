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

// N·m per active joint: stiffness * (target - position) - damping * velocity
// damping = 2 * damping_ratio * sqrt(stiffness * inertia), the inertia reflected through
// count_per_joint (count per rad, row actuator)
std::array<double, kActiveJointCount> joint_impedance_torque(
    const std::array<double, kActiveJointCount>& target, const std::array<double, kActiveJointCount>& position,
    const std::array<double, kActiveJointCount>& velocity, const ControllerConfig::JointImpedanceController& config,
    const JointJacobian& count_per_joint);

// Effort per actuator for joint torques, before friction and the max_effort clamp
// joint_per_count is rad per count, row joint
std::array<double, kActuatorCount> joint_torque_to_effort(
    const std::array<double, kActiveJointCount>& torque, const JointJacobian& joint_per_count,
    const ControllerConfig::JointImpedanceController& config);

// Joint space impedance: joint_impedance_torque at the FK pose, then joint_torque_to_effort
// Friction compensation is added per actuator, and the effort is clamped to max_effort and sent as CST
// Hand type c has no Jacobian and sends 0
class JointImpedanceController
{
 public:
  explicit JointImpedanceController(long period_nanoseconds);

  // Fills mode_of_operation and target_effort in frames
  void compute(const JointImpedanceCommand& command,
               const ControllerConfig::JointImpedanceController& config, const HandState& state,
               const std::array<double, kActuatorCount>& max_effort, canfd::CommandFrames& frames);

  // Effort per actuator added to the impedance effort, which sets the direction at rest
  // The LuGre model advances its bristle state by one period
  std::array<double, kActuatorCount> friction_compensation(
      const ControllerConfig::JointImpedanceController& config, const HandState& state,
      const std::array<double, kActuatorCount>& impedance_effort);

  // Next entry starts the LuGre bristles relaxed
  void reset();

 private:
  const double dt_seconds_;

  // count, the LuGre bristle deflection
  std::array<double, kActuatorCount> bristle_{};
};

}  // namespace aidin_hand2::controllers
