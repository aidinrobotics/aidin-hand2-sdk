#include "hand_core/kinematics/gen2/kinematics.hpp"

// Defined in prebuilt/<arch>/libaidin_hand2_kinematics_type_a.a
extern "C" {
void aidin_hand2_kin_a_fk_actuator_to_joint(const int* encoder, double* joint, double* task);
void aidin_hand2_kin_a_ik_joint_to_actuator(const double* active_joint, int* encoder);
void aidin_hand2_kin_a_ik_task_to_joint(const double* task, double* active_joint);
void aidin_hand2_kin_a_ik_oriented_task_to_joint(const double* task, double* active_joint);
void aidin_hand2_kin_a_jacobian_actuator_to_joint(const double* active_joint, double* jacobian);
void aidin_hand2_kin_a_jacobian_joint_to_actuator(const double* active_joint, double* jacobian);
void aidin_hand2_kin_a_passive_joint_slope(const double* active_joint, double* slope);
}  // extern "C"

namespace aidin_hand2::kinematics::gen2
{

void fk_actuator_to_joint(const std::array<int, ACTUATOR_NUM>& encoder,
                          std::array<double, JOINT_NUM>& joint, std::array<double, TASK_NUM>& task)
{
  aidin_hand2_kin_a_fk_actuator_to_joint(encoder.data(), joint.data(), task.data());
}

void ik_joint_to_actuator(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                          std::array<int, ACTUATOR_NUM>& encoder)
{
  aidin_hand2_kin_a_ik_joint_to_actuator(active_joint.data(), encoder.data());
}

void ik_task_to_joint(const std::array<double, TASK_NUM>& task,
                      std::array<double, ACTIVE_JOINT_NUM>& active_joint)
{
  aidin_hand2_kin_a_ik_task_to_joint(task.data(), active_joint.data());
}

void ik_oriented_task_to_joint(const std::array<double, ORIENTED_TASK_NUM>& task,
                               std::array<double, ACTIVE_JOINT_NUM>& active_joint)
{
  aidin_hand2_kin_a_ik_oriented_task_to_joint(task.data(), active_joint.data());
}

void jacobian_actuator_to_joint(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                                std::array<double, ACTIVE_JOINT_NUM * ACTUATOR_NUM>& jacobian)
{
  aidin_hand2_kin_a_jacobian_actuator_to_joint(active_joint.data(), jacobian.data());
}

void jacobian_joint_to_actuator(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                                std::array<double, ACTUATOR_NUM * ACTIVE_JOINT_NUM>& jacobian)
{
  aidin_hand2_kin_a_jacobian_joint_to_actuator(active_joint.data(), jacobian.data());
}

void passive_joint_slope(const std::array<double, ACTIVE_JOINT_NUM>& active_joint,
                         std::array<double, PASSIVE_JOINT_NUM>& slope)
{
  aidin_hand2_kin_a_passive_joint_slope(active_joint.data(), slope.data());
}

}  // namespace aidin_hand2::kinematics::gen2
