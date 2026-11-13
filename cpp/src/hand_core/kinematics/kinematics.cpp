#include "hand_core/kinematics/kinematics.hpp"

namespace aidin_hand2::kinematics
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

// The 21-joint FK output without each digit's passive q4
std::array<double, ACTIVE_JOINT_NUM> active_joints_of(const std::array<double, JOINT_NUM>& joints)
{
  std::array<double, ACTIVE_JOINT_NUM> active{};
  for (std::size_t k = 0; k < 4; ++k) active[k] = joints[k];
  for (std::size_t finger = 0; finger < 4; ++finger) {
    for (std::size_t k = 0; k < 3; ++k) active[4 + 3 * finger + k] = joints[5 + 4 * finger + k];
  }
  return active;
}

}  // namespace

// ------------------------- Joint velocity and effort ------------------------

// velocity = (d joint/d encoder) * encoder rate, and each q4 its slope times the velocity of its q3
std::array<double, JOINT_NUM> fk_actuator_to_joint_velocity(
    const Solver& solver, const ActuatorConstants& actuator, const HandState& state)
{
  const std::array<double, ACTIVE_JOINT_NUM> active = active_joints_of(state.joints.position_rad);

  // Row joint
  std::array<double, ACTIVE_JOINT_NUM * ACTUATOR_NUM> joint_per_count{};
  solver.jacobian_actuator_to_joint(active, joint_per_count);
  std::array<double, ACTIVE_JOINT_NUM> active_velocity{};
  for (std::size_t j = 0; j < ACTIVE_JOINT_NUM; ++j) {
    for (std::size_t i = 0; i < ACTUATOR_NUM; ++i) {
      const double count_rate = state.actuators.velocity_rpm[i] * actuator.count_per_revolution / 60.0;
      active_velocity[j] += joint_per_count[j * ACTUATOR_NUM + i] * count_rate;
    }
  }
  std::array<double, PASSIVE_JOINT_NUM> slope{};
  solver.passive_joint_slope(active, slope);

  // Thumb q0..q4, then q1..q4 of each long finger
  std::array<double, JOINT_NUM> velocity{};
  for (std::size_t k = 0; k < 4; ++k) velocity[k] = active_velocity[k];
  velocity[4] = slope[0] * velocity[3];
  for (std::size_t finger = 0; finger < 4; ++finger) {
    const std::size_t q1 = 5 + 4 * finger;
    for (std::size_t k = 0; k < 3; ++k) velocity[q1 + k] = active_velocity[4 + 3 * finger + k];
    velocity[q1 + 3] = slope[1 + finger] * velocity[q1 + 2];
  }
  return velocity;
}

// effort = (2 pi / count per revolution) * (d encoder/d joint)^T * motor torque
std::array<double, ACTIVE_JOINT_NUM> fk_actuator_to_joint_effort(
    const Solver& solver, const ActuatorConstants& actuator, const HandState& state)
{
  // Row actuator
  std::array<double, ACTUATOR_NUM * ACTIVE_JOINT_NUM> count_per_joint{};
  solver.jacobian_joint_to_actuator(active_joints_of(state.joints.position_rad), count_per_joint);
  const double motor_rad_per_count = 2.0 * kPi / actuator.count_per_revolution;

  std::array<double, ACTIVE_JOINT_NUM> effort{};
  for (std::size_t j = 0; j < ACTIVE_JOINT_NUM; ++j) {
    for (std::size_t i = 0; i < ACTUATOR_NUM; ++i) {
      const double motor_torque = actuator.torque_constant_Nm_per_A * state.actuators.current_mA[i] / 1000.0;
      effort[j] += motor_rad_per_count * count_per_joint[i * ACTIVE_JOINT_NUM + j] * motor_torque;
    }
  }
  return effort;
}

}  // namespace aidin_hand2::kinematics
