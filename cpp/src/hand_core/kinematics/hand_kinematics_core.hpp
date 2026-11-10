#pragma once

#include <array>

#include <aidin_hand2/types/description.hpp>

// Declaration-only on purpose. The equations live in the private aidin-hand2-kinematics
// repository and reach this tree as prebuilt/<arch>/libaidin_hand2_kinematics_type_<t>.a, one
// archive per hand type, which the build absorbs into libaidin_hand2.so.
//
// The archives are linked by whatever compiler builds this SDK, so their boundary is C, and each
// hand type's entry points carry its letter so that both archives link at once. Boundary gathers
// one type's six, boundary_of picks it from the HandType, and the inline adapters below restore
// the array-taking signatures the rest of the tree calls — each one forwards a pointer.

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


}  // namespace aidin_hand2::kinematics

// ---- The archives' C boundary ----

extern "C" {

// HandType::Gen2
void aidin_hand2_kin_a_fk_actuator_to_joint(const int* encoder, double* joint, double* task);
void aidin_hand2_kin_a_ik_joint_to_actuator(const double* active_joint, int* encoder);
void aidin_hand2_kin_a_ik_task_to_joint(const double* task, double* active_joint);
void aidin_hand2_kin_a_ik_oriented_task_to_joint(const double* task, double* active_joint);
void aidin_hand2_kin_a_jacobian_actuator_to_joint(const double* active_joint, double* jacobian);
void aidin_hand2_kin_a_jacobian_joint_to_actuator(const double* active_joint, double* jacobian);

// HandType::Gen2_B
void aidin_hand2_kin_b_fk_actuator_to_joint(const int* encoder, double* joint, double* task);
void aidin_hand2_kin_b_ik_joint_to_actuator(const double* active_joint, int* encoder);
void aidin_hand2_kin_b_ik_task_to_joint(const double* task, double* active_joint);
void aidin_hand2_kin_b_ik_oriented_task_to_joint(const double* task, double* active_joint);
void aidin_hand2_kin_b_jacobian_actuator_to_joint(const double* active_joint, double* jacobian);
void aidin_hand2_kin_b_jacobian_joint_to_actuator(const double* active_joint, double* jacobian);

}  // extern "C"

// ---- Internal interface, used by HandCore and never shipped in include/ ----

namespace aidin_hand2::kinematics
{

// One hand type's six entry points
struct Boundary
{
  void (*fk_actuator_to_joint)(const int* encoder, double* joint, double* task);
  void (*ik_joint_to_actuator)(const double* active_joint, int* encoder);
  void (*ik_task_to_joint)(const double* task, double* active_joint);
  void (*ik_oriented_task_to_joint)(const double* task, double* active_joint);
  void (*jacobian_actuator_to_joint)(const double* active_joint, double* jacobian);
  void (*jacobian_joint_to_actuator)(const double* active_joint, double* jacobian);
};

// The boundary of a hand type
const Boundary& boundary_of(HandType type);

// Encoder to joint angles and fingertip task
inline void fk_actuator_to_joint(
  const Boundary& kin,
  const std::array<int, ACTUATOR_NUM>& encoder,
  std::array<double, JOINT_NUM>& joint,
  std::array<double, TASK_NUM>& task)
{
  kin.fk_actuator_to_joint(encoder.data(), joint.data(), task.data());
}

// Active joint angles in rad to actuator encoders
inline void ik_joint_to_actuator(
  const Boundary& kin,
  const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
  std::array<int, ACTUATOR_NUM>& encoder)
{
  kin.ik_joint_to_actuator(active_joint.data(), encoder.data());
}

// task xyz → active joint (Newton-Raphson IK).
inline void ik_task_to_joint(
  const Boundary& kin,
  const std::array<double, TASK_NUM>& task,
  std::array<double, ACTIVE_JOINT_NUM>& active_joint)
{
  kin.ik_task_to_joint(task.data(), active_joint.data());
}

// Oriented form of ik_task_to_joint, taking a 6-D thumb pose and xyz for the long fingers
inline void ik_oriented_task_to_joint(
  const Boundary& kin,
  const std::array<double, ORIENTED_TASK_NUM>& task,
  std::array<double, ACTIVE_JOINT_NUM>& active_joint)
{
  kin.ik_oriented_task_to_joint(task.data(), active_joint.data());
}

// Row-major 16 x 16, block diagonal by digit
// d(active joint)/d(encoder) in rad per count
inline void jacobian_actuator_to_joint(
  const Boundary& kin,
  const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
  std::array<double, ACTIVE_JOINT_NUM * ACTUATOR_NUM>& jacobian)
{
  kin.jacobian_actuator_to_joint(active_joint.data(), jacobian.data());
}

// d(encoder)/d(active joint) in count per rad
inline void jacobian_joint_to_actuator(
  const Boundary& kin,
  const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
  std::array<double, ACTUATOR_NUM * ACTIVE_JOINT_NUM>& jacobian)
{
  kin.jacobian_joint_to_actuator(active_joint.data(), jacobian.data());
}

}  // namespace aidin_hand2::kinematics
