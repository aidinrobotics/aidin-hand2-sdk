#pragma once

#include <array>

// Declaration-only on purpose. This header ships with the public SDK, while the hand geometry
// constants (hand_kinematics_constants.hpp) and the equations stay in the internal repo and
// reach the public SDK only as prebuilt/<arch>/libaidin_hand2_kinematics.so

// Flat-array gen2 kinematics core
// The long fingers use the hand1 equations, and the thumb is a separate 4-actuator,
// 5-joint block at the front of the array

namespace aidin_hand2::kinematics
{

// ---- Layout ----

// ===== THUMB block, at the front of the array =====
constexpr int THUMB_ACTUATOR_COUNT       = 4;   // d0..d3
constexpr int THUMB_JOINT_COUNT          = 5;   // q0..q4
constexpr int THUMB_ACTIVE_JOINT_COUNT   = 4;   // q0..q3
constexpr int THUMB_TASK_COUNT           = 3;   // xyz
constexpr int THUMB_ORIENTED_TASK_COUNT  = 6;   // xyz + dir (orient mode)

// ===== LONG_FINGER block, the counts being per finger =====
constexpr int LONG_FINGER_COUNT              = 4;   // index, middle, ring, baby
constexpr int LONG_FINGER_ACTUATOR_COUNT     = 3;   // d1..d3
constexpr int LONG_FINGER_JOINT_COUNT        = 4;   // q1..q4
constexpr int LONG_FINGER_ACTIVE_JOINT_COUNT = 3;   // q1..q3
constexpr int LONG_FINGER_TASK_COUNT         = 3;   // xyz

// ===== totals =====
constexpr int ACTUATOR_NUM      = 16;
constexpr int JOINT_NUM         = 21;
constexpr int ACTIVE_JOINT_NUM  = 16;
constexpr int PASSIVE_JOINT_NUM = 5;   // the 4-bar coupled q4 of each digit, solved by FK
constexpr int TASK_NUM          = 15;
constexpr int ORIENTED_TASK_NUM = 18;


// ---- Internal interface, used by HandCore and never shipped in include/ ----

// Encoder to joint angles and fingertip task
void fk_actuator_to_joint(
  const std::array<int, ACTUATOR_NUM>& encoder,
  std::array<double, JOINT_NUM>& joint,
  std::array<double, TASK_NUM>& task);

// Active joint angles in rad to actuator encoders
void ik_joint_to_actuator(
  const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
  std::array<int, ACTUATOR_NUM>& encoder);

// task xyz → active joint (Newton-Raphson IK).
void ik_task_to_joint(
  const std::array<double, TASK_NUM>& task,
  std::array<double, ACTIVE_JOINT_NUM>& active_joint);

// Oriented form of ik_task_to_joint, taking a 6-D thumb pose and xyz for the long fingers
void ik_oriented_task_to_joint(
  const std::array<double, ORIENTED_TASK_NUM>& task,
  std::array<double, ACTIVE_JOINT_NUM>& active_joint);

}  // namespace aidin_hand2::kinematics
