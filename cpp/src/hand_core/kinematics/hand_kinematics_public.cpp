#include <aidin_hand2/hand/hand_kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <variant>

#include "hand_core/kinematics/hand_kinematics_core.hpp"

// Wraps the internal flat-array core, turning its out-parameters into return values
// FK is exposed twice, once returning joints and once returning the fingertip task
// compute_joint_vel_eff maps the drives' velocity and effort to the joints through the Jacobian

namespace aidin_hand2
{

// ------------------------------- C boundary --------------------------------

namespace kinematics
{

namespace
{

constexpr Boundary kGen2{
    aidin_hand2_kin_a_fk_actuator_to_joint,       aidin_hand2_kin_a_ik_joint_to_actuator,
    aidin_hand2_kin_a_ik_task_to_joint,           aidin_hand2_kin_a_ik_oriented_task_to_joint,
    aidin_hand2_kin_a_jacobian_actuator_to_joint, aidin_hand2_kin_a_jacobian_joint_to_actuator};

constexpr Boundary kGen2B{
    aidin_hand2_kin_b_fk_actuator_to_joint,       aidin_hand2_kin_b_ik_joint_to_actuator,
    aidin_hand2_kin_b_ik_task_to_joint,           aidin_hand2_kin_b_ik_oriented_task_to_joint,
    aidin_hand2_kin_b_jacobian_actuator_to_joint, aidin_hand2_kin_b_jacobian_joint_to_actuator};

}  // namespace

const Boundary& boundary_of(HandType type)
{
  return type == HandType::Gen2_B ? kGen2B : kGen2;
}

}  // namespace kinematics

// ------------------------------- Public API --------------------------------

std::array<double, kJointCount> fk_actuator_to_joint(
    HandType hand_type, const std::array<int, kActuatorCount>& encoder)
{
  std::array<double, kinematics::JOINT_NUM> joint{};
  std::array<double, kinematics::TASK_NUM> task{};
  kinematics::fk_actuator_to_joint(kinematics::boundary_of(hand_type), encoder, joint, task);
  return joint;
}

std::array<double, kTaskCount> fk_actuator_to_task(
    HandType hand_type, const std::array<int, kActuatorCount>& encoder)
{
  std::array<double, kinematics::JOINT_NUM> joint{};
  std::array<double, kinematics::TASK_NUM> task{};
  kinematics::fk_actuator_to_joint(kinematics::boundary_of(hand_type), encoder, joint, task);
  return task;
}

std::array<int, kActuatorCount> ik_joint_to_actuator(
    HandType hand_type, const std::array<double, kActiveJointCount>& active_joint)
{
  std::array<int, kinematics::ACTUATOR_NUM> encoder{};
  kinematics::ik_joint_to_actuator(kinematics::boundary_of(hand_type), active_joint, encoder);
  return encoder;
}

std::array<double, kActiveJointCount> ik_task_to_joint(
    HandType hand_type, const std::array<double, kTaskCount>& task)
{
  std::array<double, kinematics::ACTIVE_JOINT_NUM> active_joint{};
  kinematics::ik_task_to_joint(kinematics::boundary_of(hand_type), task, active_joint);
  return active_joint;
}

std::array<double, kActiveJointCount> ik_oriented_task_to_joint(
    HandType hand_type, const std::array<double, kOrientedTaskCount>& oriented_task)
{
  std::array<double, kinematics::ACTIVE_JOINT_NUM> active_joint{};
  kinematics::ik_oriented_task_to_joint(kinematics::boundary_of(hand_type), oriented_task, active_joint);
  return active_joint;
}

// velocity = (d joint/d encoder) * encoder rate
// effort   = (2 pi / 1024) * (d encoder/d joint)^T * motor torque

namespace
{

// maxon ECX SPEED 8 M datasheet
constexpr double kTorqueConstantNmPerA = 3.62e-3;
// Current at effort 1000, the drive setting
constexpr double kRatedCurrentA        = 0.4;
constexpr double kRatedMotorTorqueNm   = kTorqueConstantNmPerA * kRatedCurrentA;

// 256 CPT x 4 on the motor shaft
constexpr double kCountPerRevolution = 1024.0;
constexpr double kMotorRadPerCount   = 2.0 * 3.14159265358979323846 / kCountPerRevolution;

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

}  // namespace

HandStateJointVelEff compute_joint_vel_eff(HandType hand_type, const HandState& state)
{
  HandStateJointVelEff out{};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const kinematics::Boundary& kin = kinematics::boundary_of(hand_type);
  const std::array<double, kActiveJointCount> active = active_joints_of(state.joints.position_rad);

  // Row = joint for joint_per_count, row = actuator for count_per_joint
  std::array<double, kActiveJointCount * kActuatorCount> joint_per_count{};
  std::array<double, kActuatorCount * kActiveJointCount> count_per_joint{};
  kinematics::jacobian_actuator_to_joint(kin, active, joint_per_count);
  kinematics::jacobian_joint_to_actuator(kin, active, count_per_joint);

  std::array<double, kActuatorCount> count_rate{};
  std::array<double, kActuatorCount> motor_torque{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    count_rate[i]   = state.actuators.velocity_rpm[i] * kCountPerRevolution / 60.0;
    motor_torque[i] = kTorqueConstantNmPerA * state.actuators.current_mA[i] / 1000.0;
  }
  const auto* setpoint = std::get_if<ActuatorEffortSetpoint>(&state.commanded.controller_output);

  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    double velocity = 0.0, effort = 0.0, commanded = 0.0, largest = 0.0;
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      const double per_count = joint_per_count[j * kActuatorCount + i];
      const double per_joint = count_per_joint[i * kActiveJointCount + j];
      velocity += per_count * count_rate[i];
      effort   += kMotorRadPerCount * per_joint * motor_torque[i];
      if (setpoint != nullptr) {
        commanded += kMotorRadPerCount * per_joint * setpoint->target_effort_pct[i] / 1000.0 * kRatedMotorTorqueNm;
      }
      // Motor torque on actuator i per joint torque on joint j is per_count / kMotorRadPerCount
      largest = std::max(largest, std::abs(per_count));
    }
    out.velocity_rad_s[j]      = velocity;
    out.effort_Nm[j]           = effort;
    out.commanded_effort_Nm[j] = setpoint != nullptr ? commanded : nan;
    out.rated_effort_Nm[j]     = kRatedMotorTorqueNm * kMotorRadPerCount / largest;
  }
  return out;
}

}  // namespace aidin_hand2
