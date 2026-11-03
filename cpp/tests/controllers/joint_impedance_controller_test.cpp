// 목적(Tier 1, HW 무관): joint 공간 impedance 제어기를 host 에서 회귀 점검한다.
//   1) joint torque → actuator effort: Jacobian 전치와 효율, 2-rod 쌍과 d3 결합.
//   2) joint impedance torque: stiffness 항, 현재 자세 관성에서 구한 damping 항.
// 기준값은 kinematics archive(type a)로 기준 자세(엄지 20/20/0/20°, 긴 손가락 0/20/20°)에서 구했다.
// type c 에는 Jacobian 이 없으므로 그 빌드에서는 a·b 전용 시험을 건너뛴다.

#include <array>
#include <cmath>
#include <cstdio>

#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/description.hpp>

#include "hand_core/controllers/joint_impedance_controller.hpp"
#include "hand_core/kinematics/hand_kinematics_core.hpp"

using namespace aidin_hand2;

namespace
{
int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) { std::printf("  FAIL: %s\n", what); ++g_failures; }
}

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kCountPerRad = 1024.0 / (2.0 * 3.14159265358979323846);
constexpr double kRatedMotorTorqueNm = 3.62e-3 * 0.4;

using Config = ControllerConfig::JointImpedanceController;
using Active = std::array<double, kActiveJointCount>;

// 엄지 q0..q3, 긴 손가락마다 q1..q3 (deg)
Active pose_deg(std::array<double, 4> thumb, std::array<double, 3> finger)
{
  Active q{};
  for (std::size_t k = 0; k < 4; ++k) q[k] = thumb[k] * kDeg;
  for (std::size_t f = 0; f < 4; ++f) {
    for (std::size_t k = 0; k < 3; ++k) q[4 + 3 * f + k] = finger[k] * kDeg;
  }
  return q;
}

// 기준 자세: GUI 가 n 상수를 읽던 자세.
Active reference_pose() { return pose_deg({20, 20, 0, 20}, {0, 20, 20}); }

#if !defined(AIDIN_HAND2_HAND_TYPE_C)
struct Jacobians {
  controllers::JointJacobian joint_per_count{};
  controllers::JointJacobian count_per_joint{};
};

Jacobians jacobians_at(const Active& q)
{
  Jacobians j{};
  kinematics::jacobian_actuator_to_joint(q, j.joint_per_count);
  kinematics::jacobian_joint_to_actuator(q, j.count_per_joint);
  return j;
}

// joint 하나에만 torque 를 준다.
Active torque_on(std::size_t joint, double torque)
{
  Active tau{};
  tau[joint] = torque;
  return tau;
}

// expected 에 없는 actuator 는 0 이어야 한다.
bool efforts_are(const std::array<double, kActuatorCount>& effort,
                 std::initializer_list<std::pair<std::size_t, double>> expected)
{
  std::array<bool, kActuatorCount> listed{};
  for (const auto& [i, value] : expected) {
    listed[i] = true;
    if (std::abs(effort[i] - value) > 0.05) return false;
  }
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (!listed[i] && std::abs(effort[i]) > 0.01) return false;
  }
  return true;
}

// ── Test 1: joint torque → actuator effort ──
void test_joint_torque_to_effort()
{
  const Jacobians j = jacobians_at(reference_pose());
  const Config config{};
  auto effort = [&](std::size_t joint, double k) {
    return controllers::joint_torque_to_effort(torque_on(joint, k * kDeg), j.joint_per_count, config);
  };
  check(efforts_are(effort(0, 27), {{0, 319.57}}), "thumb j0 1 deg at K 27");
  check(efforts_are(effort(1, 90), {{1, 323.94}, {2, 323.94}}), "thumb j1 is the common mode of d1 d2");
  check(efforts_are(effort(2, 24), {{1, -248.06}, {2, 248.06}}), "thumb j2 is the differential mode of d1 d2");
  check(efforts_are(effort(3, 5), {{3, 82.76}}), "thumb j3 1 deg at K 5");
  check(efforts_are(effort(4, 3.75), {{4, 77.52}, {5, -77.52}}), "index j1 is the differential mode");
  check(efforts_are(effort(5, 22.5), {{4, 186.75}, {5, 186.75}}), "index j2 is the common mode");
  check(efforts_are(effort(6, 6.75), {{4, 5.64}, {5, 5.64}, {6, 151.90}}), "index j3 pulls d1 d2 through the d3 coupling");

  Config half = config;
  half.efficiency[0] = 0.5;
  check(efforts_are(controllers::joint_torque_to_effort(torque_on(0, 27 * kDeg), j.joint_per_count, half), {{0, 639.14}}),
        "half the efficiency doubles the effort");

  // 엄지 q2 20°: 두 rod 의 몫이 갈라져도 j1 torque 는 j1 에만 걸린다.
  const Jacobians tilted = jacobians_at(pose_deg({20, 20, 20, 20}, {0, 20, 20}));
  const auto split = controllers::joint_torque_to_effort(torque_on(1, 90 * kDeg), tilted.joint_per_count, config);
  check(efforts_are(split, {{1, 308.99}, {2, 373.60}}), "thumb j1 at j2 20 deg splits 308.99 / 373.60");
  Active back{};
  for (std::size_t q = 0; q < kActiveJointCount; ++q) {
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      back[q] += tilted.count_per_joint[i * kActiveJointCount + q] / kCountPerRad * split[i] / 1000.0 * kRatedMotorTorqueNm;
    }
  }
  check(std::abs(back[1] - 90 * kDeg) < 1e-4, "the efforts give back the j1 torque");
  check(std::abs(back[2]) < 1e-6, "no torque leaks to j2");
}

// ── Test 2: joint impedance torque ──
void test_joint_impedance_torque()
{
  const Active q = reference_pose();
  const Jacobians j = jacobians_at(q);
  Config config{};
  config.stiffness_Nm_per_rad.fill(0.0);
  config.stiffness_Nm_per_rad[0] = 27.0;

  Active target = q;
  target[0] += 1.0 * kDeg;
  const Active still{};
  config.damping_ratio.fill(0.0);
  const Active spring = controllers::joint_impedance_torque(target, q, still, config, j.count_per_joint);
  bool others_zero = true;
  for (std::size_t k = 1; k < kActiveJointCount; ++k) others_zero = others_zero && spring[k] == 0.0;
  check(std::abs(spring[0] - 0.471239) < 1e-6 && others_zero, "stiffness term is K times the joint error");

  // q̇0 0.61698 rad/s 는 d0 6000 rpm. 관성 1018.4² × 3.3e-9 = 3.4224e-3 kg·m², D = 0.42557.
  config.damping_ratio.fill(0.7);
  Active moving{};
  moving[0] = 0.61698;
  const Active damped = controllers::joint_impedance_torque(q, q, moving, config, j.count_per_joint);
  check(std::abs(damped[0] - (-0.26257)) < 1e-4, "damping from the reflected motor inertia");

  Config soft = config;
  soft.stiffness_Nm_per_rad[0] = 0.0;
  check(controllers::joint_impedance_torque(q, q, moving, soft, j.count_per_joint)[0] == 0.0,
        "no stiffness, no damping");

  Config heavy = config;
  heavy.link_inertia_kgm2[0] = 3.4224e-3;
  check(std::abs(controllers::joint_impedance_torque(q, q, moving, heavy, j.count_per_joint)[0] - (-0.37133)) < 1e-4,
        "link inertia adds to the motor inertia");
}
#endif
}  // namespace

int main()
{
#if !defined(AIDIN_HAND2_HAND_TYPE_C)
  test_joint_torque_to_effort();
  test_joint_impedance_torque();
#endif

  std::printf("joint_impedance_controller test: %s\n", g_failures == 0 ? "OK" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
