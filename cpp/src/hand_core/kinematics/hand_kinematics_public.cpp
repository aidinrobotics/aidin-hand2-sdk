#include <aidin_hand2/hand/hand_kinematics.hpp>

#include "hand_core/kinematics/hand_kinematics_core.hpp"

// Wraps the internal flat-array core, turning its out-parameters into return values
// FK is exposed twice, once returning joints and once returning the fingertip task

namespace aidin_hand2
{

std::array<double, kJointCount> fk_actuator_to_joint(
    const std::array<int, kActuatorCount>& encoder)
{
  std::array<double, kinematics::JOINT_NUM> joint{};
  std::array<double, kinematics::TASK_NUM> task{};
  kinematics::fk_actuator_to_joint(encoder, joint, task);
  return joint;
}

std::array<double, kTaskCount> fk_actuator_to_task(
    const std::array<int, kActuatorCount>& encoder)
{
  std::array<double, kinematics::JOINT_NUM> joint{};
  std::array<double, kinematics::TASK_NUM> task{};
  kinematics::fk_actuator_to_joint(encoder, joint, task);
  return task;
}

std::array<int, kActuatorCount> ik_joint_to_actuator(
    const std::array<double, kActiveJointCount>& active_joint)
{
  std::array<int, kinematics::ACTUATOR_NUM> encoder{};
  kinematics::ik_joint_to_actuator(active_joint, encoder);
  return encoder;
}

std::array<double, kActiveJointCount> ik_task_to_joint(
    const std::array<double, kTaskCount>& task)
{
  std::array<double, kinematics::ACTIVE_JOINT_NUM> active_joint{};
  kinematics::ik_task_to_joint(task, active_joint);
  return active_joint;
}

std::array<double, kActiveJointCount> ik_oriented_task_to_joint(
    const std::array<double, kOrientedTaskCount>& oriented_task)
{
  std::array<double, kinematics::ACTIVE_JOINT_NUM> active_joint{};
  kinematics::ik_oriented_task_to_joint(oriented_task, active_joint);
  return active_joint;
}

}  // namespace aidin_hand2
