#pragma once

#include <array>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>
#include "hand_core/comms/canfd/protocol.hpp"

namespace aidin_hand2::controllers
{

// PD impedance in encoder space: effort = stiffness * error - damping * velocity
// IK gives the target position, and the resulting effort is clamped to max_effort and sent as CST
// Velocity is the drive's rpm, cross gains and friction compensation come from the config
class JointImpedanceController
{
 public:
  explicit JointImpedanceController(long period_nanoseconds);

  // Fills mode_of_operation and target_effort in frames
  void compute(const JointImpedanceCommand& command,
               const ControllerConfig::JointImpedanceController& config, const HandState& state,
               const std::array<double, kActuatorCount>& max_effort, canfd::CommandFrames& frames);

  // Next entry starts the LuGre bristles relaxed
  void reset();

 private:
  const double dt_seconds_;

  // count, the LuGre bristle deflection
  std::array<double, kActuatorCount> bristle_{};
};

}  // namespace aidin_hand2::controllers
