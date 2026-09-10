#include "hand_core/controllers/joint_impedance_controller.hpp"

#include <algorithm>

#include "hand_core/kinematics/hand_kinematics_core.hpp"

namespace aidin_hand2::controllers
{

JointImpedanceController::JointImpedanceController(long period_nanoseconds)
: dt_seconds_(static_cast<double>(period_nanoseconds) / 1e9)
{
}

void JointImpedanceController::reset()
{
  have_previous_ = false;
}

void JointImpedanceController::compute(const JointImpedanceCommand& command,
                                       const ControllerConfig::JointImpedanceController& config,
                                       const HandState& state,
                                       const std::array<double, kActuatorCount>& max_effort,
                                       canfd::CommandFrames& frames)
{
  // 1) IK: active joint target(rad) → actuator target position (count).
  std::array<int, kActuatorCount> desired_cnt{};
  kinematics::ik_joint_to_actuator(command.target, desired_cnt);

  // 2) PD effort: stiffness * error - damping * velocity. Velocity differentiates the encoder
  //    against the previous cycle, so the first cycle after a reset contributes no damping.
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    const double error    = static_cast<double>(desired_cnt[i]) - state.actuators.position_count[i];
    const double velocity = have_previous_
                                ? (state.actuators.position_count[i] - previous_position_count_[i]) / dt_seconds_
                                : 0.0;
    const double effort = config.stiffness[i] * error - config.damping[i] * velocity;

    // 3) CST command — the drive does not cap current in CST, so clamp to ±max_effort here.
    frames.mode_of_operation[i] = canfd::mode_of_operation::kCST;
    frames.target_effort[i]     = static_cast<std::int16_t>(std::clamp(effort, -max_effort[i], max_effort[i]));
  }

  // 4) Keep this position for the next cycle's velocity term.
  previous_position_count_ = state.actuators.position_count;
  have_previous_ = true;
}

}  // namespace aidin_hand2::controllers
