#pragma once

#include "hand_core/kinematics/kinematics.hpp"

// Gen2B kinematics, from the archive built for it

namespace aidin_hand2::kinematics::gen2b
{

void fk_actuator_to_joint(const std::array<int, ACTUATOR_NUM>& encoder,
                          std::array<double, JOINT_NUM>& joint, std::array<double, TASK_NUM>& task);

void ik_joint_to_actuator(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                          std::array<int, ACTUATOR_NUM>& encoder);

void ik_task_to_joint(const std::array<double, TASK_NUM>& task,
                      std::array<double, ACTIVE_JOINT_NUM>& active_joint);

void ik_oriented_task_to_joint(const std::array<double, ORIENTED_TASK_NUM>& task,
                               std::array<double, ACTIVE_JOINT_NUM>& active_joint);

void jacobian_actuator_to_joint(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                                std::array<double, ACTIVE_JOINT_NUM * ACTUATOR_NUM>& jacobian);

void jacobian_joint_to_actuator(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                                std::array<double, ACTUATOR_NUM * ACTIVE_JOINT_NUM>& jacobian);

void passive_joint_slope(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                         std::array<double, PASSIVE_JOINT_NUM>& slope);

inline constexpr Solver kSolver{
    fk_actuator_to_joint,      ik_joint_to_actuator,       ik_task_to_joint,
    ik_oriented_task_to_joint, jacobian_actuator_to_joint, jacobian_joint_to_actuator,
    passive_joint_slope};

}  // namespace aidin_hand2::kinematics::gen2b
