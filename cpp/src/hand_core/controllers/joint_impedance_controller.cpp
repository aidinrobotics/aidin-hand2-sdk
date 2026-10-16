#include "hand_core/controllers/joint_impedance_controller.hpp"

#include <algorithm>
#include <cmath>

#include "hand_core/kinematics/hand_kinematics_core.hpp"

namespace aidin_hand2::controllers
{

namespace
{

constexpr std::size_t kFrictionTableBins = ControllerConfig::JointImpedanceController::kFrictionTableBins;

// Linear between bin centres, the edge value held outside them, the first bin when start == end
double friction_table_at(const std::array<double, kFrictionTableBins>& table, double start, double end,
                         double count)
{
  constexpr double kBins = static_cast<double>(kFrictionTableBins);
  if (end == start) return table[0];

  // 0 at the first bin centre
  const double position = (count - start) / (end - start) * kBins - 0.5;
  if (position <= 0.0) return table[0];
  if (position >= kBins - 1.0) return table[kFrictionTableBins - 1];
  const auto lower = static_cast<std::size_t>(position);
  const double fraction = position - static_cast<double>(lower);
  return table[lower] + fraction * (table[lower + 1] - table[lower]);
}

}  // namespace

JointImpedanceController::JointImpedanceController(long period_nanoseconds)
: dt_seconds_(static_cast<double>(period_nanoseconds) / 1e9)
{
}

void JointImpedanceController::reset()
{
  have_previous_ = false;
  filtered_velocity_.fill(0.0);
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

  // 2) Error (count) and velocity (count/s) of all actuators first
  //    A pair's cross gains read the partner's
  constexpr double kCountPerRevolution = 256.0 * 4.0;
  const double alpha = config.velocity_cutoff_hz > 0.0
                           ? 1.0 - std::exp(-2.0 * M_PI * config.velocity_cutoff_hz * dt_seconds_)
                           : 1.0;
  std::array<double, kActuatorCount> error{};
  std::array<double, kActuatorCount> raw_velocity{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    error[i] = static_cast<double>(desired_cnt[i]) - state.actuators.position_count[i];
    double velocity;
    if (config.velocity_from_rpm) {
      velocity = state.actuators.velocity_rpm[i] * kCountPerRevolution / 60.0;
    } else {
      velocity = have_previous_
                     ? (state.actuators.position_count[i] - previous_position_count_[i]) / dt_seconds_
                     : 0.0;
    }
    raw_velocity[i] = velocity;
    filtered_velocity_[i] += alpha * (velocity - filtered_velocity_[i]);
  }

  // 3) Cross gains within each two-rod pair
  //    An actuator outside a pair is its own partner with gain 0
  constexpr std::array<std::array<std::size_t, 2>, 5> kCrossPairs{{{1, 2}, {4, 5}, {7, 8}, {10, 11}, {13, 14}}};
  std::array<std::size_t, kActuatorCount> partner{};
  std::array<double, kActuatorCount> cross_k{}, cross_d{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) partner[i] = i;
  for (std::size_t p = 0; p < kCrossPairs.size(); ++p) {
    const auto [a, b] = kCrossPairs[p];
    partner[a] = b;  partner[b] = a;
    cross_k[a] = cross_k[b] = config.cross_stiffness[p];
    cross_d[a] = cross_d[b] = config.cross_damping[p];
  }

  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    // 4) Stiffness term capped at p_saturation_effort, then damping
    const std::size_t j = partner[i];
    double p = config.stiffness[i] * error[i] + cross_k[i] * error[j];
    if (config.p_saturation_effort > 0.0) {
      p = std::clamp(p, -config.p_saturation_effort, config.p_saturation_effort);
    }
    const double impedance_effort =
        p - config.damping[i] * filtered_velocity_[i] - cross_d[i] * filtered_velocity_[j];

    // 5) Friction compensation, the impedance effort sets the direction at rest
    double compensation = 0.0;
    if (config.friction_scale > 0.0) {
      const double v = filtered_velocity_[i];
      const bool moving = std::abs(v) > config.friction_stick_velocity;
      const double direction = moving ? (v > 0.0 ? 1.0 : -1.0)
                                      : std::clamp(impedance_effort / config.friction_stick_ramp, -1.0, 1.0);
      double positive = config.friction_coulomb_positive[i];
      double negative = config.friction_coulomb_negative[i];
      if (config.friction_model == ControllerConfig::JointImpedanceController::FrictionModel::PositionTable) {
        positive = friction_table_at(config.friction_table_positive[i], config.friction_table_start[i],
                                     config.friction_table_end[i], state.actuators.position_count[i]);
        negative = friction_table_at(config.friction_table_negative[i], config.friction_table_start[i],
                                     config.friction_table_end[i], state.actuators.position_count[i]);
      }
      const double bias = 0.5 * (positive - negative);
      const double friction = 0.5 * (positive + negative);
      const double bias_weight = (moving || config.friction_bias_at_rest) ? 1.0 : std::abs(direction);
      compensation =
          config.friction_scale * (bias * bias_weight + friction * direction + config.friction_viscous[i] * v);
    }

    // 6) Brake above velocity_soft_limit, on the unfiltered velocity
    double brake = 0.0;
    if (config.velocity_soft_limit > 0.0) {
      const double v = raw_velocity[i];
      const double over = std::abs(v) - config.velocity_soft_limit;
      if (over > 0.0) brake = config.velocity_brake_gain * over * (v > 0.0 ? 1.0 : -1.0);
    }
    const double effort = impedance_effort + compensation - brake;

    // 7) CST command — the drive does not cap current in CST, so clamp to ±max_effort here.
    frames.mode_of_operation[i] = canfd::mode_of_operation::kCST;
    frames.target_effort[i]     = static_cast<std::int16_t>(std::clamp(effort, -max_effort[i], max_effort[i]));
  }

  // 8) Keep this position for the next cycle's velocity term.
  previous_position_count_ = state.actuators.position_count;
  have_previous_ = true;
}

}  // namespace aidin_hand2::controllers
