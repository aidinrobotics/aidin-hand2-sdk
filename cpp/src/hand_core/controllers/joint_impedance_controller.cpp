#include "hand_core/controllers/joint_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aidin_hand2::controllers
{

namespace
{

using Config = ControllerConfig::JointImpedanceController;

constexpr double kPi = 3.14159265358979323846;

// Nm/rad on the motor shaft at long finger j3 stiffness 1
constexpr double kHardMotorStiffnessJ3 = 3.62e-5;

// Motor rad per long finger j3 rad, the most the j3 hard stiffness follows
constexpr double kMaxMotorPerJ3 = 652.0;

// Damping ratio per active joint
constexpr std::array<double, kActiveJointCount> kDampingRatio{
    0.62, 0.49, 0.49, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};

// s, the time constant a new stiffness is approached with
constexpr double kStiffnessTimeConstant = 0.2;

// rad/s, the most a target moves
constexpr double kMaxTargetVelocity = 250.0 * kPi / 180.0;

// count, the least a target keeps each actuator off its hard stop at 0
constexpr int kMinTargetCount = 1000;

// count, over which the effort toward the hard stop tapers to 0
constexpr double kStopZoneCount = 1000.0;

// Matched-pole low-pass coefficient: alpha = 1 - exp(-dt * 2 * pi * fc), cutoff <= 0 gives 1
double lowpass_alpha(double cutoff_freq, double dt_seconds)
{
  if (!(cutoff_freq > 0.0)) return 1.0;
  return 1.0 - std::exp(-dt_seconds * 2.0 * kPi * cutoff_freq);
}

// Active joints 6, 9, 12 and 15, each driven by the actuator of the same index
bool is_long_finger_j3(std::size_t joint)
{
  return joint >= 4 && (joint - 4) % 3 == 2;
}

// rad of the motor shaft per encoder count
double motor_rad_per_count(const ActuatorConstants& actuator)
{
  return 2.0 * kPi / actuator.count_per_revolution;
}

// Nm on the motor shaft at effort 1000
double rated_motor_torque(const ActuatorConstants& actuator)
{
  return actuator.torque_constant_Nm_per_A * actuator.rated_current_A;
}

// Nm/rad at stiffness 1; the long finger j3 follows the pose at its motor-side stiffness
double joint_hard_stiffness(const std::array<double, kActiveJointCount>& hard_stiffness,
                            const ActuatorConstants& actuator, std::size_t joint,
                            const JointJacobian& count_per_joint)
{
  if (!is_long_finger_j3(joint)) return hard_stiffness[joint];
  const double motor_per_joint = std::min(
      std::abs(count_per_joint[joint * kActiveJointCount + joint]) * motor_rad_per_count(actuator),
      kMaxMotorPerJ3);
  return kHardMotorStiffnessJ3 * motor_per_joint * motor_per_joint;
}

// The 21-joint FK output without each digit's passive q4
std::array<double, kActiveJointCount> active_joints_of(const std::array<double, kJointCount>& joints)
{
  std::array<double, kActiveJointCount> active{};
  for (std::size_t k = 0; k < 4; ++k) active[k] = joints[k];
  for (std::size_t finger = 0; finger < 4; ++finger) {
    for (std::size_t k = 0; k < 3; ++k) active[4 + 3 * finger + k] = joints[5 + 4 * finger + k];
  }
  return active;
}

// Thumb 0 and long fingers 1..4, for an active joint or the actuator of the same index
std::size_t finger_of(std::size_t index)
{
  return index < 4 ? 0 : 1 + (index - 4) / 3;
}

// A finger with an IK count below kMinTargetCount takes the FK pose of its counts raised to it
std::array<double, kActiveJointCount> off_hard_stop(const kinematics::Solver& solver,
                                                     const std::array<double, kActiveJointCount>& target)
{
  std::array<int, kActuatorCount> count{};
  solver.ik_joint_to_actuator(target, count);
  std::array<bool, kFingerCount> raised{};
  bool any_raised = false;
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (count[i] >= kMinTargetCount) continue;
    count[i] = kMinTargetCount;
    raised[finger_of(i)] = true;
    any_raised = true;
  }
  if (!any_raised) return target;

  std::array<double, kJointCount> joints{};
  std::array<double, kTaskCount> task{};
  solver.fk_actuator_to_joint(count, joints, task);
  const std::array<double, kActiveJointCount> lifted = active_joints_of(joints);
  std::array<double, kActiveJointCount> result = target;
  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    if (raised[finger_of(j)] && std::isfinite(lifted[j])) result[j] = lifted[j];
  }
  return result;
}

// rad/s from the drives' velocity_rpm
std::array<double, kActiveJointCount> joint_velocity(const ActuatorConstants& actuator, const HandState& state,
                                                      const JointJacobian& joint_per_count)
{
  std::array<double, kActiveJointCount> velocity{};
  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      const double count_rate = state.actuators.velocity_rpm[i] * actuator.count_per_revolution / 60.0;
      velocity[j] += joint_per_count[j * kActuatorCount + i] * count_rate;
    }
  }
  return velocity;
}

}  // namespace

std::array<double, kActiveJointCount> joint_impedance_torque(
    const std::array<double, kActiveJointCount>& hard_stiffness, const ActuatorConstants& actuator,
    const std::array<double, kActiveJointCount>& target, const std::array<double, kActiveJointCount>& position,
    const std::array<double, kActiveJointCount>& velocity, const std::array<double, kActiveJointCount>& stiffness,
    const JointJacobian& count_per_joint)
{
  std::array<double, kActiveJointCount> torque{};
  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    double inertia = 0.0;
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      const double motor_per_joint = count_per_joint[i * kActiveJointCount + j] * motor_rad_per_count(actuator);
      inertia += motor_per_joint * motor_per_joint * actuator.motor_inertia_kg_m2;
    }
    const double k           = joint_hard_stiffness(hard_stiffness, actuator, j, count_per_joint) * stiffness[j];
    const double coefficient = 2.0 * kDampingRatio[j] * std::sqrt(k * inertia);
    torque[j] = k * (target[j] - position[j]) - coefficient * velocity[j];
  }
  return torque;
}

std::array<double, kActuatorCount> joint_torque_to_effort(
    const ActuatorConstants& actuator, const std::array<double, kActiveJointCount>& torque,
    const JointJacobian& joint_per_count)
{
  std::array<double, kActuatorCount> effort{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    // Motor torque is the joint torques through d(joint)/d(motor rad)
    double motor_torque = 0.0;
    for (std::size_t j = 0; j < kActiveJointCount; ++j) {
      // The long finger j3 drives its own actuator only
      // j2 holds the linkage's reaction by its stiffness
      if (is_long_finger_j3(j) && i != j) continue;
      motor_torque += joint_per_count[j * kActuatorCount + i] / motor_rad_per_count(actuator) * torque[j];
    }
    effort[i] = motor_torque / rated_motor_torque(actuator) * 1000.0;
  }
  return effort;
}

JointImpedanceController::JointImpedanceController(long period_nanoseconds, const kinematics::Solver& solver,
                                                   const std::array<double, kActiveJointCount>& hard_stiffness,
                                                   const ActuatorConstants& actuator)
: solver_(solver),
  hard_stiffness_(hard_stiffness),
  actuator_(actuator),
  dt_seconds_(static_cast<double>(period_nanoseconds) / 1e9),
  stiffness_alpha_(1.0 - std::exp(-dt_seconds_ / kStiffnessTimeConstant))
{
  reset();
}

void JointImpedanceController::reset()
{
  target_.fill(std::numeric_limits<double>::quiet_NaN());
  stiffness_.fill(std::numeric_limits<double>::quiet_NaN());
}

std::array<double, kActiveJointCount> JointImpedanceController::filtered_target(
    const std::array<double, kActiveJointCount>& command_target, const Config& config,
    const std::array<double, kActiveJointCount>& measured)
{
  if (config.cutoff_freq != cutoff_freq_) {
    cutoff_freq_ = config.cutoff_freq;
    alpha_       = lowpass_alpha(cutoff_freq_, dt_seconds_);
  }
  const double alpha    = config.filter_enabled ? alpha_ : 1.0;
  const double max_step = kMaxTargetVelocity * dt_seconds_;

  for (std::size_t i = 0; i < kActiveJointCount; ++i) {
    // Seed from the measured pose on (re-)entry; from the target if FK gave NaN
    if (std::isnan(target_[i])) {
      const double seed = std::isfinite(measured[i]) ? measured[i] : command_target[i];
      target_[i] = seed;
      section_[i].fill(seed);
    }

    // 3rd-order low-pass as first-order sections in cascade; alpha 1 passes the target through
    if (alpha >= 1.0) {
      section_[i].fill(command_target[i]);
    } else {
      section_[i][0] += alpha * (command_target[i] - section_[i][0]);
      section_[i][1] += alpha * (section_[i][0]   - section_[i][1]);
      section_[i][2] += alpha * (section_[i][1]   - section_[i][2]);
    }

    target_[i] += std::clamp(section_[i][2] - target_[i], -max_step, max_step);
  }
  target_ = off_hard_stop(solver_, target_);
  return target_;
}

void JointImpedanceController::compute(const JointImpedanceCommand& command,
                                       const ControllerConfig::JointImpedanceController& config,
                                       const HandState& state,
                                       const std::array<double, kActuatorCount>& max_effort,
                                       canfd::CommandFrames& frames)
{
  // 1) Joint position from FK and both Jacobians at it
  const std::array<double, kActiveJointCount> position = active_joints_of(state.joints.position_rad);
  JointJacobian joint_per_count{};
  JointJacobian count_per_joint{};
  solver_.jacobian_actuator_to_joint(position, joint_per_count);
  solver_.jacobian_joint_to_actuator(position, count_per_joint);

  // 2) Target through the low-pass and the velocity limit, stiffness on its way to the setting
  const std::array<double, kActiveJointCount> target = filtered_target(command.target, config, position);
  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    if (std::isnan(stiffness_[j])) stiffness_[j] = config.stiffness[j];
    stiffness_[j] += stiffness_alpha_ * (config.stiffness[j] - stiffness_[j]);
  }

  // 3) Joint law, then its effort through the transposed Jacobian
  const std::array<double, kActiveJointCount> velocity = joint_velocity(actuator_, state, joint_per_count);
  const std::array<double, kActiveJointCount> torque   = joint_impedance_torque(
      hard_stiffness_, actuator_, target, position, velocity, stiffness_, count_per_joint);
  const std::array<double, kActuatorCount> effort = joint_torque_to_effort(actuator_, torque, joint_per_count);

  // 4) CST command within ±max_effort, the only cap: the drive does not cap current in CST
  //    The bound toward the hard stop tapers to 0 over kStopZoneCount
  //    An effort that is not finite sends 0
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    const double value = std::isfinite(effort[i]) ? effort[i] : 0.0;
    const double toward_stop =
        max_effort[i] * std::clamp(state.actuators.position_count[i] / kStopZoneCount, 0.0, 1.0);
    frames.mode_of_operation[i] = canfd::mode_of_operation::kCST;
    frames.target_effort[i]     = static_cast<std::int16_t>(std::clamp(value, -toward_stop, max_effort[i]));
  }
}

}  // namespace aidin_hand2::controllers
