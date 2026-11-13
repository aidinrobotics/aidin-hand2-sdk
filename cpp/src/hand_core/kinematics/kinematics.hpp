#pragma once

#include <array>

#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>

#include "types/actuator_constants.hpp"

// Solver: one hand model's kinematics as a table of functions, filled in kinematics/<model>/
// model/ picks the Solver at run time, and the rest of the tree calls through it

namespace aidin_hand2::kinematics
{

// ---------------------------------- Layout ----------------------------------

// Thumb, at the front of the arrays
constexpr int THUMB_ACTUATOR_COUNT       = 4;   // d0..d3
constexpr int THUMB_JOINT_COUNT          = 5;   // q0..q4
constexpr int THUMB_ACTIVE_JOINT_COUNT   = 4;   // q0..q3
constexpr int THUMB_TASK_COUNT           = 3;   // xyz
constexpr int THUMB_ORIENTED_TASK_COUNT  = 6;   // xyz + direction

// Each long finger
constexpr int LONG_FINGER_COUNT              = 4;   // index, middle, ring, baby
constexpr int LONG_FINGER_ACTUATOR_COUNT     = 3;   // d1..d3
constexpr int LONG_FINGER_JOINT_COUNT        = 4;   // q1..q4
constexpr int LONG_FINGER_ACTIVE_JOINT_COUNT = 3;   // q1..q3
constexpr int LONG_FINGER_TASK_COUNT         = 3;   // xyz

// Whole hand
constexpr int ACTUATOR_NUM      = 16;
constexpr int JOINT_NUM         = 21;
constexpr int ACTIVE_JOINT_NUM  = 16;
constexpr int PASSIVE_JOINT_NUM = 5;   // q4 of each finger
constexpr int TASK_NUM          = 15;
constexpr int ORIENTED_TASK_NUM = 18;

// ---------------------------------- Solver ----------------------------------

// Angles in rad, actuators in encoder count
struct Solver {
  // Encoder to joint angles and fingertip task
  void (*fk_actuator_to_joint)(const std::array<int, ACTUATOR_NUM>& encoder,
                               std::array<double, JOINT_NUM>& joint, std::array<double, TASK_NUM>& task);

  // Active joint angles to actuator encoders
  void (*ik_joint_to_actuator)(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                               std::array<int, ACTUATOR_NUM>& encoder);

  // Fingertip xyz to active joint angles
  void (*ik_task_to_joint)(const std::array<double, TASK_NUM>& task,
                           std::array<double, ACTIVE_JOINT_NUM>& active_joint);

  // Thumb xyz and direction, long finger xyz to active joint angles
  void (*ik_oriented_task_to_joint)(const std::array<double, ORIENTED_TASK_NUM>& task,
                                    std::array<double, ACTIVE_JOINT_NUM>& active_joint);

  // d(active joint)/d(encoder), rad per count, row-major 16 x 16
  void (*jacobian_actuator_to_joint)(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                                     std::array<double, ACTIVE_JOINT_NUM * ACTUATOR_NUM>& jacobian);

  // d(encoder)/d(active joint), count per rad, row-major 16 x 16
  void (*jacobian_joint_to_actuator)(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                                     std::array<double, ACTUATOR_NUM * ACTIVE_JOINT_NUM>& jacobian);

  // d(passive q4)/d(q3) of each digit, thumb first, rad per rad
  // The q4 row of d(joint)/d(encoder) is this times the q3 row of the same digit
  void (*passive_joint_slope)(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                              std::array<double, PASSIVE_JOINT_NUM>& slope);
};

// ------------------------- Joint velocity and effort ------------------------

// rad/s per joint, velocity_rpm through the Jacobian at the pose in state, the passive q4 included
std::array<double, JOINT_NUM> fk_actuator_to_joint_velocity(
    const Solver& solver, const ActuatorConstants& actuator, const HandState& state);

// Nm per active joint, current_mA through the transposed Jacobian at the pose in state
// Effort leaves out gear losses
std::array<double, ACTIVE_JOINT_NUM> fk_actuator_to_joint_effort(
    const Solver& solver, const ActuatorConstants& actuator, const HandState& state);

}  // namespace aidin_hand2::kinematics
