#include "hand_core/controllers/joint_impedance_controller.hpp"

#include <algorithm>
#include <cmath>

#include "hand_core/kinematics/hand_kinematics_core.hpp"

namespace aidin_hand2::controllers
{

namespace
{

using Config = ControllerConfig::JointImpedanceController;
constexpr std::size_t kTableBins = Config::PositionTableFriction::kBins;

// 256 CPT x 4 on the motor shaft
constexpr double kCountPerRevolution = 1024.0;
constexpr double kMotorRadPerCount   = 2.0 * 3.14159265358979323846 / kCountPerRevolution;

// N·m at effort 1000: maxon ECX SPEED 8 M at the drive's 0.4 A
constexpr double kRatedMotorTorqueNm = 3.62e-3 * 0.4;

// count/s, below it the impedance effort sets the friction direction
constexpr double kStickVelocity = 300.0;

// Effort, the least the direction ramps over at rest
constexpr double kMinStickRamp = 300.0;

// Linear between bin centres, the edge value held outside them, the first bin when start == end
double table_at(const std::array<double, kTableBins>& table, double start, double end, double count)
{
  constexpr double kBins = static_cast<double>(kTableBins);
  if (end == start) return table[0];

  // 0 at the first bin centre
  const double position = (count - start) / (end - start) * kBins - 0.5;
  if (position <= 0.0) return table[0];
  if (position >= kBins - 1.0) return table[kTableBins - 1];
  const auto lower = static_cast<std::size_t>(position);
  const double fraction = position - static_cast<double>(lower);
  return table[lower] + fraction * (table[lower + 1] - table[lower]);
}

// c at speed, s at rest
double stribeck_at(double c, double s, double stribeck_velocity, double shape, double speed)
{
  if (!(stribeck_velocity > 0.0)) return c;
  return c + (s - c) * std::exp(-std::pow(speed / stribeck_velocity, shape));
}

// Resisting effort toward +count and -count, and the viscous gain, of a static model
struct StaticFriction {
  double positive;
  double negative;
  double viscous;
};

StaticFriction static_friction_of(const Config& config, std::size_t i, double count, double speed)
{
  switch (config.friction_model) {
    case Config::FrictionModel::PositionTable: {
      const Config::PositionTableFriction& f = config.position_table_friction;
      return {table_at(f.positive[i], f.start[i], f.end[i], count),
              table_at(f.negative[i], f.start[i], f.end[i], count), f.viscous[i]};
    }
    case Config::FrictionModel::Stribeck: {
      const Config::StribeckFriction& f = config.stribeck_friction;
      return {stribeck_at(f.positive[i], f.static_positive[i], f.stribeck_velocity[i], f.shape, speed),
              stribeck_at(f.negative[i], f.static_negative[i], f.stribeck_velocity[i], f.shape, speed),
              f.viscous[i]};
    }
    default: {
      const Config::CoulombFriction& f = config.coulomb_friction;
      return {f.positive[i], f.negative[i], f.viscous[i]};
    }
  }
}

// Twice the largest resisting effort of the static model, so the ramp adds little stiffness
double stick_ramp_of(const Config& config)
{
  double largest = 0.0;
  const auto take = [&largest](const auto& values) {
    for (const double v : values) largest = std::max(largest, v);
  };
  switch (config.friction_model) {
    case Config::FrictionModel::PositionTable:
      for (std::size_t i = 0; i < kActuatorCount; ++i) {
        take(config.position_table_friction.positive[i]);
        take(config.position_table_friction.negative[i]);
      }
      break;
    case Config::FrictionModel::Stribeck:
      take(config.stribeck_friction.static_positive);
      take(config.stribeck_friction.static_negative);
      take(config.stribeck_friction.positive);
      take(config.stribeck_friction.negative);
      break;
    default:
      take(config.coulomb_friction.positive);
      take(config.coulomb_friction.negative);
      break;
  }
  return std::max(kMinStickRamp, 2.0 * largest);
}

#if !defined(AIDIN_HAND2_HAND_TYPE_C)
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

// rad/s from the drives' velocity_rpm
std::array<double, kActiveJointCount> joint_velocity(const HandState& state, const JointJacobian& joint_per_count)
{
  std::array<double, kActiveJointCount> velocity{};
  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      const double count_rate = state.actuators.velocity_rpm[i] * kCountPerRevolution / 60.0;
      velocity[j] += joint_per_count[j * kActuatorCount + i] * count_rate;
    }
  }
  return velocity;
}
#endif

}  // namespace

std::array<double, kActiveJointCount> joint_impedance_torque(
    const std::array<double, kActiveJointCount>& target, const std::array<double, kActiveJointCount>& position,
    const std::array<double, kActiveJointCount>& velocity, const Config& config,
    const JointJacobian& count_per_joint)
{
  std::array<double, kActiveJointCount> torque{};
  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    double inertia = config.link_inertia_kgm2[j];
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      const double motor_per_joint = count_per_joint[i * kActiveJointCount + j] * kMotorRadPerCount;
      inertia += motor_per_joint * motor_per_joint * config.motor_inertia_kgm2[i];
    }
    const double stiffness = config.stiffness_Nm_per_rad[j];
    const double damping   = 2.0 * config.damping_ratio[j] * std::sqrt(stiffness * inertia);
    torque[j] = stiffness * (target[j] - position[j]) - damping * velocity[j];
  }
  return torque;
}

std::array<double, kActuatorCount> joint_torque_to_effort(
    const std::array<double, kActiveJointCount>& torque, const JointJacobian& joint_per_count,
    const Config& config)
{
  std::array<double, kActuatorCount> effort{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    // Motor torque is the joint torques through d(joint)/d(motor rad)
    double motor_torque = 0.0;
    for (std::size_t j = 0; j < kActiveJointCount; ++j) {
      motor_torque += joint_per_count[j * kActuatorCount + i] / kMotorRadPerCount * torque[j];
    }
    effort[i] = motor_torque / (config.efficiency[i] * kRatedMotorTorqueNm) * 1000.0;
  }
  return effort;
}

JointImpedanceController::JointImpedanceController(long period_nanoseconds)
: dt_seconds_(static_cast<double>(period_nanoseconds) / 1e9)
{
}

void JointImpedanceController::reset()
{
  bristle_.fill(0.0);
}

void JointImpedanceController::compute(const JointImpedanceCommand& command,
                                       const ControllerConfig::JointImpedanceController& config,
                                       const HandState& state,
                                       const std::array<double, kActuatorCount>& max_effort,
                                       canfd::CommandFrames& frames)
{
  std::array<double, kActuatorCount> effort{};
#if !defined(AIDIN_HAND2_HAND_TYPE_C)
  // 1) Joint position from FK and both Jacobians at it
  const std::array<double, kActiveJointCount> position = active_joints_of(state.joints.position_rad);
  JointJacobian joint_per_count{};
  JointJacobian count_per_joint{};
  kinematics::jacobian_actuator_to_joint(position, joint_per_count);
  kinematics::jacobian_joint_to_actuator(position, count_per_joint);

  // 2) Joint law, then its effort through the transposed Jacobian
  const std::array<double, kActiveJointCount> velocity = joint_velocity(state, joint_per_count);
  const std::array<double, kActiveJointCount> torque =
      joint_impedance_torque(command.target, position, velocity, config, count_per_joint);
  const std::array<double, kActuatorCount> impedance_effort = joint_torque_to_effort(torque, joint_per_count, config);

  // 3) Friction compensation per actuator
  const std::array<double, kActuatorCount> compensation = friction_compensation(config, state, impedance_effort);
  for (std::size_t i = 0; i < kActuatorCount; ++i) effort[i] = impedance_effort[i] + compensation[i];
#else
  (void)command;
  (void)config;
  (void)state;
#endif

  // 4) CST command — the drive does not cap current in CST, so clamp to ±max_effort here.
  //    An effort that is not finite sends 0
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    const double value = std::isfinite(effort[i]) ? effort[i] : 0.0;
    frames.mode_of_operation[i] = canfd::mode_of_operation::kCST;
    frames.target_effort[i]     = static_cast<std::int16_t>(std::clamp(value, -max_effort[i], max_effort[i]));
  }
}

std::array<double, kActuatorCount> JointImpedanceController::friction_compensation(
    const ControllerConfig::JointImpedanceController& config, const HandState& state,
    const std::array<double, kActuatorCount>& impedance_effort)
{
  std::array<double, kActuatorCount> compensation{};
  const bool compensate = config.friction_scale > 0.0;
  const bool lugre = config.friction_model == Config::FrictionModel::LuGre;
  const double stick_ramp = compensate && !lugre ? stick_ramp_of(config) : kMinStickRamp;
  if (!compensate || !lugre) bristle_.fill(0.0);

  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    const double v = state.actuators.velocity_rpm[i] * kCountPerRevolution / 60.0;
    if (compensate && lugre) {
      // Implicit step of the bristle, stable for any bristle stiffness
      const Config::LuGreFriction& f = config.lugre_friction;
      const Config::StribeckFriction& curve = f.curve;
      const double speed = std::abs(v);
      const double g = v >= 0.0
                           ? stribeck_at(curve.positive[i], curve.static_positive[i], curve.stribeck_velocity[i],
                                         curve.shape, speed)
                           : stribeck_at(curve.negative[i], curve.static_negative[i], curve.stribeck_velocity[i],
                                         curve.shape, speed);
      const double sigma0 = f.bristle_stiffness[i];
      const double previous = bristle_[i];
      const double next = (g > 0.0 && sigma0 > 0.0)
                              ? (previous + v * dt_seconds_) / (1.0 + sigma0 * speed * dt_seconds_ / g)
                              : 0.0;
      bristle_[i] = next;
      const double rate = (next - previous) / dt_seconds_;
      compensation[i] = config.friction_scale * (sigma0 * next + f.bristle_damping[i] * rate + curve.viscous[i] * v);
    } else if (compensate) {
      // The impedance effort sets the direction at rest, ramping over ±stick_ramp
      const StaticFriction f =
          static_friction_of(config, i, state.actuators.position_count[i], std::abs(v));
      const bool moving = std::abs(v) > kStickVelocity;
      const double direction =
          moving ? (v > 0.0 ? 1.0 : -1.0) : std::clamp(impedance_effort[i] / stick_ramp, -1.0, 1.0);
      const double bias = 0.5 * (f.positive - f.negative);
      const double friction = 0.5 * (f.positive + f.negative);
      const double bias_weight = moving ? 1.0 : std::abs(direction);
      compensation[i] = config.friction_scale * (bias * bias_weight + friction * direction + f.viscous * v);
    }
  }
  return compensation;
}

}  // namespace aidin_hand2::controllers
