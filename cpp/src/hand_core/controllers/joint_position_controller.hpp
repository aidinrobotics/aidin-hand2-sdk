#pragma once

#include <array>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>
#include "hand_core/comms/canfd/protocol.hpp"

namespace aidin_hand2::controllers
{

// Active joint target in rad through a deadband and a 3rd-order low-pass, then IK to CSP
// The deadband is send-on-delta, so it drops small changes without leaving a steady offset
// The low-pass has coincident real poles, so it does not overshoot
class JointPositionController
{
 public:
  explicit JointPositionController(long period_nanoseconds);

  void compute(const JointPositionCommand& command,
               const ControllerConfig::JointPositionController& config, const HandState& state,
               canfd::CommandFrames& frames);

  // Next entry starts from the measured pose
  void reset();

 private:
  const double dt_seconds_;

  // The cutoff alpha_ was derived from, -1 forcing the first derivation
  double cutoff_freq_{-1.0};
  double alpha_{1.0};

  // rad, and a NaN in held_ means unset, so the next compute seeds both from the measured pose
  std::array<double, kActiveJointCount> held_{};
  std::array<std::array<double, 3>, kActiveJointCount> section_{};
};

}  // namespace aidin_hand2::controllers
