#include <aidin_hand2/hand/hand_kinematics.hpp>

#include "hand_core/kinematics/kinematics.hpp"
#include "hand_core/model/model_composition.hpp"

// Each call takes the hand model and uses its composition from model/

namespace aidin_hand2
{

// ---------------------------- Forward kinematics ----------------------------

std::array<double, kJointCount> fk_actuator_to_joint(
    HandModel hand_model, const std::array<int, kActuatorCount>& encoder)
{
  std::array<double, kinematics::JOINT_NUM> joint{};
  std::array<double, kinematics::TASK_NUM> task{};
  composition_of(hand_model).solver.fk_actuator_to_joint(encoder, joint, task);
  return joint;
}

std::array<double, kTaskCount> fk_actuator_to_task(
    HandModel hand_model, const std::array<int, kActuatorCount>& encoder)
{
  std::array<double, kinematics::JOINT_NUM> joint{};
  std::array<double, kinematics::TASK_NUM> task{};
  composition_of(hand_model).solver.fk_actuator_to_joint(encoder, joint, task);
  return task;
}

// ---------------------------- Inverse kinematics ----------------------------

std::array<int, kActuatorCount> ik_joint_to_actuator(
    HandModel hand_model, const std::array<double, kActiveJointCount>& active_joint)
{
  std::array<int, kinematics::ACTUATOR_NUM> encoder{};
  composition_of(hand_model).solver.ik_joint_to_actuator(active_joint, encoder);
  return encoder;
}

std::array<double, kActiveJointCount> ik_task_to_joint(
    HandModel hand_model, const std::array<double, kTaskCount>& task)
{
  std::array<double, kinematics::ACTIVE_JOINT_NUM> active_joint{};
  composition_of(hand_model).solver.ik_task_to_joint(task, active_joint);
  return active_joint;
}

std::array<double, kActiveJointCount> ik_oriented_task_to_joint(
    HandModel hand_model, const std::array<double, kOrientedTaskCount>& oriented_task)
{
  std::array<double, kinematics::ACTIVE_JOINT_NUM> active_joint{};
  composition_of(hand_model).solver.ik_oriented_task_to_joint(oriented_task, active_joint);
  return active_joint;
}

}  // namespace aidin_hand2
