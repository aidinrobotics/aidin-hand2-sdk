// fk_actuator_to_joint_velocity and fk_actuator_to_joint_effort against the public FK
//   velocity   central difference of fk_actuator_to_joint over the encoder step rpm makes, every joint
//   effort     virtual work, joint power equals motor power
//   HandState  fill_joint_state puts the same velocity on every joint and the same effort on the
//              active joints, NaN effort on the passive
// Hand model Gen2; Gen2B differs in the thumb d1·d2 Jacobian only

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include <aidin_hand2/hand/hand_kinematics.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/state.hpp>

#include "hand_core/hand_core.hpp"
#include "hand_core/model/model_composition.hpp"

using namespace aidin_hand2;

namespace aidin_hand2::test
{
// Reaches HandCore::fill_joint_state, a friend in hand_core.hpp
struct HandCoreTestPeer {
  static void fill_joint_state(HandCore& core, HandState& state) { core.fill_joint_state(state); }
};
}  // namespace aidin_hand2::test

namespace
{

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr HandModel kModel = HandModel::Gen2;
const ModelComposition& kGen2 = kGen2Composition;

// A pose inside the workspace, away from the home stops
HandState state_at_pose(const std::array<std::int32_t, kActuatorCount>& rpm)
{
  std::array<double, kActiveJointCount> pose{30 * kDeg, 20 * kDeg, 5 * kDeg, 30 * kDeg};
  for (std::size_t f = 0; f < 4; ++f) {
    pose[4 + 3 * f] = 5 * kDeg;
    pose[5 + 3 * f] = 40 * kDeg;
    pose[6 + 3 * f] = 30 * kDeg;
  }
  const auto encoder = ik_joint_to_actuator(kModel, pose);
  HandState state{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) state.actuators.position_count[i] = encoder[i];
  state.actuators.velocity_rpm = rpm;
  state.joints.position_rad = fk_actuator_to_joint(kModel, encoder);
  return state;
}

std::array<std::int32_t, kActuatorCount> test_rpm()
{
  std::array<std::int32_t, kActuatorCount> rpm{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    rpm[i] = static_cast<std::int32_t>((i % 2 == 0 ? 1 : -1) * (300 + 40 * static_cast<int>(i)));
  }
  return rpm;
}

void set_test_current(HandState& state)
{
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    state.actuators.current_mA[i] = static_cast<std::int16_t>((i % 3 == 0 ? -1 : 1) * (50 + 15 * static_cast<int>(i)));
  }
}

constexpr double kTorqueConstantNmPerA = 3.62e-3;

std::array<double, kActiveJointCount> active_of(const std::array<double, kJointCount>& joints)
{
  std::array<double, kActiveJointCount> active{};
  for (std::size_t k = 0; k < 4; ++k) active[k] = joints[k];
  for (std::size_t f = 0; f < 4; ++f) {
    for (std::size_t k = 0; k < 3; ++k) active[4 + 3 * f + k] = joints[5 + 4 * f + k];
  }
  return active;
}

void test_velocity()
{
  // One rpm moves rpm counts in 60 / 1024 s, so +-rpm counts span twice that
  const auto rpm = test_rpm();
  const HandState state = state_at_pose(rpm);
  std::array<int, kActuatorCount> plus{}, minus{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    plus[i]  = state.actuators.position_count[i] + rpm[i];
    minus[i] = state.actuators.position_count[i] - rpm[i];
  }
  const auto qp = fk_actuator_to_joint(kModel, plus);
  const auto qm = fk_actuator_to_joint(kModel, minus);
  // Every joint, the passive j4 of each digit included
  const std::array<double, kJointCount> velocity =
      kinematics::fk_actuator_to_joint_velocity(kGen2.solver, kGen2.actuator, state);
  double worst = 0.0, largest = 0.0;
  for (std::size_t j = 0; j < kJointCount; ++j) {
    const double fd = (qp[j] - qm[j]) / (2.0 * 60.0 / 1024.0);
    // std::max would drop a NaN, so a missing velocity counts as infinitely wrong
    const double diff = std::abs(velocity[j] - fd);
    worst = std::isfinite(diff) ? std::max(worst, diff) : std::numeric_limits<double>::infinity();
    largest = std::max(largest, std::abs(fd));
  }
  std::printf("  velocity vs FK difference, worst %.2e rad/s of %.2e\n", worst, largest);
  check(std::isfinite(worst) && worst < 0.01 * largest, "velocity differs from the FK difference");
}

void test_virtual_work()
{
  const auto rpm = test_rpm();
  HandState state = state_at_pose(rpm);
  set_test_current(state);
  // The passive joints take no effort, so the power balance runs over the active ones
  const auto velocity = active_of(kinematics::fk_actuator_to_joint_velocity(kGen2.solver, kGen2.actuator, state));
  const auto effort   = kinematics::fk_actuator_to_joint_effort(kGen2.solver, kGen2.actuator, state);
  double joint_power = 0.0, motor_power = 0.0;
  for (std::size_t j = 0; j < kActiveJointCount; ++j) joint_power += effort[j] * velocity[j];
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    motor_power += kTorqueConstantNmPerA * state.actuators.current_mA[i] / 1000.0 * rpm[i] * 2.0 * 3.14159265358979323846 / 60.0;
  }
  std::printf("  joint power %.6e W, motor power %.6e W\n", joint_power, motor_power);
  check(std::abs(joint_power - motor_power) < 1e-9 * std::abs(motor_power), "joint power differs from motor power");
}

void test_hand_state()
{
  HandState state = state_at_pose(test_rpm());
  set_test_current(state);
  HandCore core(HandConfig{"can0", HandSide::Right, kModel});
  test::HandCoreTestPeer::fill_joint_state(core, state);
  const auto expected_velocity = kinematics::fk_actuator_to_joint_velocity(kGen2.solver, kGen2.actuator, state);
  const auto expected_effort   = kinematics::fk_actuator_to_joint_effort(kGen2.solver, kGen2.actuator, state);

  const auto effort = active_of(state.joints.effort_Nm);
  bool same = true;
  for (std::size_t j = 0; j < kJointCount; ++j) same = same && state.joints.velocity_rad_s[j] == expected_velocity[j];
  for (std::size_t j = 0; j < kActiveJointCount; ++j) same = same && effort[j] == expected_effort[j];
  check(same, "HandState joint velocity and effort match the Jacobian mapping");

  // The passive j4 against the FK difference over the same encoder step as test_velocity
  const auto rpm = test_rpm();
  std::array<int, kActuatorCount> plus{}, minus{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    plus[i]  = state.actuators.position_count[i] + rpm[i];
    minus[i] = state.actuators.position_count[i] - rpm[i];
  }
  const auto qp = fk_actuator_to_joint(kModel, plus);
  const auto qm = fk_actuator_to_joint(kModel, minus);
  double worst = 0.0, largest = 0.0;
  bool effort_nan = true;
  for (std::size_t joint : {4, 8, 12, 16, 20}) {
    const double fd = (qp[joint] - qm[joint]) / (2.0 * 60.0 / 1024.0);
    // std::max would drop a NaN, so a missing velocity counts as infinitely wrong
    const double diff = std::abs(state.joints.velocity_rad_s[joint] - fd);
    worst = std::isfinite(diff) ? std::max(worst, diff) : std::numeric_limits<double>::infinity();
    largest = std::max(largest, std::abs(fd));
    effort_nan = effort_nan && std::isnan(state.joints.effort_Nm[joint]);
  }
  std::printf("  passive velocity vs FK difference, worst %.2e rad/s of %.2e\n", worst, largest);
  check(std::isfinite(worst) && worst < 0.01 * largest, "HandState passive joint velocity follows the FK difference");
  check(effort_nan, "HandState joint effort is NaN on the passive joints");
}

}  // namespace

int main()
{
  test_velocity();
  test_virtual_work();
  test_hand_state();
  std::printf("joint_vel_eff test: %s\n", g_failures == 0 ? "OK" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
