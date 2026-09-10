#include "hand_core/controllers/joint_position_controller.hpp"

#include <cmath>
#include <limits>

#include "hand_core/kinematics/hand_kinematics_core.hpp"  // ik_joint_to_actuator

namespace aidin_hand2::controllers
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

// 21-joint FK output → the 16 active joints (each digit's last entry is the passive q4).
std::array<double, kActiveJointCount> active_joints_of(
    const std::array<double, kJointCount>& joints)
{
  std::array<double, kActiveJointCount> active{};
  active[0]  = joints[0];   // thumb  q0
  active[1]  = joints[1];   // thumb  q1
  active[2]  = joints[2];   // thumb  q2
  active[3]  = joints[3];   // thumb  q3
  active[4]  = joints[5];   // index  q1
  active[5]  = joints[6];   // index  q2
  active[6]  = joints[7];   // index  q3
  active[7]  = joints[9];   // middle q1
  active[8]  = joints[10];  // middle q2
  active[9]  = joints[11];  // middle q3
  active[10] = joints[13];  // ring   q1
  active[11] = joints[14];  // ring   q2
  active[12] = joints[15];  // ring   q3
  active[13] = joints[17];  // baby   q1
  active[14] = joints[18];  // baby   q2
  active[15] = joints[19];  // baby   q3
  return active;
}

// Matched-pole low-pass coefficient: alpha = 1 - exp(-dt/tau), tau = 1/(2*pi*fc). cutoff <= 0 gives 1.0.
double lowpass_alpha(double cutoff_freq, double dt_seconds)
{
  if (!(cutoff_freq > 0.0)) return 1.0;
  return 1.0 - std::exp(-dt_seconds * 2.0 * kPi * cutoff_freq);
}

}  // namespace

JointPositionController::JointPositionController(long period_nanoseconds)
: dt_seconds_(static_cast<double>(period_nanoseconds) / 1e9)
{
  reset();
}

// Marks the filter unset so the next compute() re-seeds held_ and section_.
void JointPositionController::reset()
{
  held_.fill(std::numeric_limits<double>::quiet_NaN());
}

// Active joint target(rad) → deadband → low-pass → IK → CSP frames.
void JointPositionController::compute(const JointPositionCommand& command,
                                      const ControllerConfig::JointPositionController& config,
                                      const HandState& state, canfd::CommandFrames& frames)
{
  if (config.cutoff_freq != cutoff_freq_) {
    cutoff_freq_ = config.cutoff_freq;
    alpha_       = lowpass_alpha(cutoff_freq_, dt_seconds_);
  }
  // Off is deadband 0 with alpha 1 — the same path, so the state stays on the target.
  const double deadband = config.filter_enabled ? config.deadband : 0.0;
  const double alpha    = config.filter_enabled ? alpha_ : 1.0;

  const std::array<double, kActiveJointCount> measured = active_joints_of(state.joints.position_rad);

  std::array<double, kActiveJointCount> filtered{};
  for (std::size_t i = 0; i < kActiveJointCount; ++i) {
    // Seed from the measured pose on (re-)entry; from the target if FK gave NaN.
    if (std::isnan(held_[i])) {
      const double seed = std::isfinite(measured[i]) ? measured[i] : command.target[i];
      held_[i] = seed;
      section_[i].fill(seed);
    }

    // 1) Send-on-delta deadband: a target within one band of the last accepted value is dropped.
    if (std::abs(command.target[i] - held_[i]) >= deadband) held_[i] = command.target[i];

    // 2) 3rd-order low-pass as first-order sections in cascade; alpha 1 passes the target through.
    if (alpha >= 1.0) {
      section_[i].fill(held_[i]);
    } else {
      section_[i][0] += alpha * (held_[i]       - section_[i][0]);
      section_[i][1] += alpha * (section_[i][0] - section_[i][1]);
      section_[i][2] += alpha * (section_[i][1] - section_[i][2]);
    }
    filtered[i] = section_[i][2];
  }

  // 3) IK: active_joint(rad) → actuator position (count).
  std::array<int, kActuatorCount> actuator_cnt{};
  kinematics::ik_joint_to_actuator(filtered, actuator_cnt);

  // 4) CSP command.
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    frames.mode_of_operation[i] = canfd::mode_of_operation::kCSP;
    frames.target_position[i]   = static_cast<std::int32_t>(actuator_cnt[i]);
  }
}

}  // namespace aidin_hand2::controllers
