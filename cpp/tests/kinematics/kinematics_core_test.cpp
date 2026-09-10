// Purpose: regression-check the kinematics core standalone, without the SDK. This is the only
//   test this repo has, so it has to catch what the SDK's integration tests used to catch first.
//
// Oracle: round trips rather than hardcoded numbers. FK is the reference for IK.
//
// Covered:
//   1) active joint --ik_joint_to_actuator--> encoder --fk_actuator_to_joint--> joint
//      The active slots of the FK output must recover the input. Encoders cannot go negative.
//   2) task --ik_task_to_joint--> joint --IK/FK--> task
//      The fingertip position must close. The thumb is a redundant chain (4 DOF for 3
//      coordinates), so its joint values are not recovered — a different valid solution comes
//      back — and only the task is checked.
//   3) Every output is finite, and the same input gives the same output.
//
// Where the tolerances come from: across the five poses below the measured worst cases were
//   0.0043 deg and 0.7301 mm. The margins are 10x and 2.7x. The computation is deterministic,
//   so these do not drift between runs.

#include <array>
#include <cmath>
#include <cstdio>

#include "hand_core/kinematics/hand_kinematics_core.hpp"

using namespace aidin_hand2::kinematics;

namespace
{

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) { std::printf("  FAIL: %s\n", what); ++g_failures; }
}

constexpr double kDeg2Rad  = 3.14159265358979323846 / 180.0;
constexpr double kJointTolDeg = 0.05;
constexpr double kTaskTolMm   = 2.0;

// Active joint index to 21-slot joint index. The thumb block comes first and every digit carries
// one passive q4, so the two arrays do not line up.
int active_to_joint_index(int active)
{
  if (active < THUMB_ACTIVE_JOINT_COUNT) { return active; }
  const int f = (active - THUMB_ACTIVE_JOINT_COUNT) / LONG_FINGER_ACTIVE_JOINT_COUNT;
  const int k = (active - THUMB_ACTIVE_JOINT_COUNT) % LONG_FINGER_ACTIVE_JOINT_COUNT;
  return THUMB_JOINT_COUNT + f * LONG_FINGER_JOINT_COUNT + k;
}

// On the thumb q0, q1 and q3 are flexion and q2 is abduction. On a long finger q1 is abduction
// and q2, q3 are flexion.
std::array<double, ACTIVE_JOINT_NUM> pose(double flex_deg, double abd_deg)
{
  const double f = flex_deg * kDeg2Rad;
  const double a = abd_deg * kDeg2Rad;
  std::array<double, ACTIVE_JOINT_NUM> aj{};
  aj[0] = f; aj[1] = f; aj[2] = a; aj[3] = f;
  for (int k = 0; k < LONG_FINGER_COUNT; ++k) {
    const int base = THUMB_ACTIVE_JOINT_COUNT + k * LONG_FINGER_ACTIVE_JOINT_COUNT;
    aj[base + 0] = a; aj[base + 1] = f; aj[base + 2] = f;
  }
  return aj;
}

bool all_finite(const double* v, int n)
{
  for (int i = 0; i < n; ++i) { if (!std::isfinite(v[i])) { return false; } }
  return true;
}

struct Pose { double flex_deg; double abd_deg; };
constexpr Pose kPoses[] = {{10, 0}, {25, 0}, {40, 0}, {25, 5}, {35, 8}};

void test_joint_round_trip()
{
  double worst_deg = 0.0;
  for (const Pose& p : kPoses) {
    const auto aj = pose(p.flex_deg, p.abd_deg);

    std::array<int, ACTUATOR_NUM> encoder{};
    ik_joint_to_actuator(aj, encoder);
    for (int i = 0; i < ACTUATOR_NUM; ++i) {
      check(encoder[i] >= 0, "encoder 가 음수다 (0 클램프가 깨졌다)");
    }

    std::array<double, JOINT_NUM> joint{};
    std::array<double, TASK_NUM> task{};
    fk_actuator_to_joint(encoder, joint, task);
    check(all_finite(joint.data(), JOINT_NUM), "FK joint 에 비유한값이 있다");
    check(all_finite(task.data(), TASK_NUM), "FK task 에 비유한값이 있다");

    for (int i = 0; i < ACTIVE_JOINT_NUM; ++i) {
      const double d = std::fabs(joint[active_to_joint_index(i)] - aj[i]) / kDeg2Rad;
      if (d > worst_deg) { worst_deg = d; }
    }
  }
  std::printf("  joint 왕복 최악 %.4f deg (허용 %.2f)\n", worst_deg, kJointTolDeg);
  check(worst_deg < kJointTolDeg, "active joint 왕복 복원 실패");
}

void test_task_round_trip()
{
  double worst_mm = 0.0;
  for (const Pose& p : kPoses) {
    const auto aj = pose(p.flex_deg, p.abd_deg);

    std::array<int, ACTUATOR_NUM> encoder{};
    ik_joint_to_actuator(aj, encoder);
    std::array<double, JOINT_NUM> joint{};
    std::array<double, TASK_NUM> task{};
    fk_actuator_to_joint(encoder, joint, task);

    std::array<double, ACTIVE_JOINT_NUM> solved{};
    ik_task_to_joint(task, solved);
    check(all_finite(solved.data(), ACTIVE_JOINT_NUM), "IK 해에 비유한값이 있다");

    std::array<int, ACTUATOR_NUM> encoder2{};
    ik_joint_to_actuator(solved, encoder2);
    std::array<double, JOINT_NUM> joint2{};
    std::array<double, TASK_NUM> task2{};
    fk_actuator_to_joint(encoder2, joint2, task2);

    for (int i = 0; i < TASK_NUM; ++i) {
      const double d = std::fabs(task[i] - task2[i]);
      if (d > worst_mm) { worst_mm = d; }
    }
  }
  std::printf("  task 왕복 최악 %.4f mm (허용 %.2f)\n", worst_mm, kTaskTolMm);
  check(worst_mm < kTaskTolMm, "fingertip task 왕복 복원 실패");
}

void test_determinism()
{
  const auto aj = pose(25, 0);
  std::array<int, ACTUATOR_NUM> encoder{};
  ik_joint_to_actuator(aj, encoder);
  std::array<double, JOINT_NUM> joint{};
  std::array<double, TASK_NUM> task{};
  fk_actuator_to_joint(encoder, joint, task);

  std::array<double, ACTIVE_JOINT_NUM> first{};
  std::array<double, ACTIVE_JOINT_NUM> second{};
  ik_task_to_joint(task, first);
  ik_task_to_joint(task, second);
  check(first == second, "ik_task_to_joint 이 같은 입력에 다른 해를 준다");

  // Oriented form: the thumb takes xyz plus a unit direction, the long fingers stay xyz.
  std::array<double, ORIENTED_TASK_NUM> oriented{};
  for (int i = 0; i < THUMB_TASK_COUNT; ++i) { oriented[i] = task[i]; }
  oriented[3] = 0.0; oriented[4] = 0.0; oriented[5] = 1.0;
  for (int i = THUMB_ORIENTED_TASK_COUNT; i < ORIENTED_TASK_NUM; ++i) {
    oriented[i] = task[i - (THUMB_ORIENTED_TASK_COUNT - THUMB_TASK_COUNT)];
  }
  std::array<double, ACTIVE_JOINT_NUM> oriented_solved{};
  ik_oriented_task_to_joint(oriented, oriented_solved);
  check(all_finite(oriented_solved.data(), ACTIVE_JOINT_NUM),
        "ik_oriented_task_to_joint 해에 비유한값이 있다");
}

}  // namespace

int main()
{
  std::printf("[joint 왕복]\n");
  test_joint_round_trip();
  std::printf("[task 왕복]\n");
  test_task_round_trip();
  std::printf("[결정성·유한성]\n");
  test_determinism();

  std::printf("kinematics_core_test: %s\n", g_failures == 0 ? "PASS" : "FAIL");
  return g_failures;
}
