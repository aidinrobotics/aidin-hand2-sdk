// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>

#include <aidin_hand2/types/description.hpp>

namespace aidin_hand2
{
// Angles in rad, actuators in encoder count

// ---------------------------- Forward kinematics ----------------------------

// Actuator encoders to joint angles
std::array<double, kJointCount> fk_actuator_to_joint(
    const std::array<int, kActuatorCount>& encoder);

// Actuator encoders to fingertip positions (not using)
std::array<double, kTaskCount> fk_actuator_to_task(
    const std::array<int, kActuatorCount>& encoder);

// ---------------------------- Inverse kinematics ----------------------------

// Joint angles to actuator encoders
std::array<int, kActuatorCount> ik_joint_to_actuator(
    const std::array<double, kActiveJointCount>& active_joint);

// Fingertip positions to joint angles (not using)
std::array<double, kActiveJointCount> ik_task_to_joint(
    const std::array<double, kTaskCount>& task);

// Thumb pose and fingertip positions to joint angles (not using)
std::array<double, kActiveJointCount> ik_oriented_task_to_joint(
    const std::array<double, kOrientedTaskCount>& oriented_task);

}  // namespace aidin_hand2
