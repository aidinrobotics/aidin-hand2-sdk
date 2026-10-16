// compute_joint_vel_eff against the public FK and IK
//   velocity   central difference of fk_actuator_to_joint over the encoder step rpm makes
//   effort     virtual work, joint power equals motor power
//   rated      thumb j0 has one actuator, so rated current there gives rated_effort_Nm
//   commanded  NaN without an effort output, the output's torque with one
// Hand type c expects NaN everywhere

#include <array>
#include <cmath>
#include <cstdio>

#include <aidin_hand2/hand/hand_kinematics.hpp>
#include <aidin_hand2/types/state.hpp>

using namespace aidin_hand2;

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

// A pose inside the workspace, away from the home stops
HandState state_at_pose(const std::array<std::int32_t, kActuatorCount>& rpm)
{
  std::array<double, kActiveJointCount> pose{30 * kDeg, 20 * kDeg, 5 * kDeg, 30 * kDeg};
  for (std::size_t f = 0; f < 4; ++f) {
    pose[4 + 3 * f] = 5 * kDeg;
    pose[5 + 3 * f] = 40 * kDeg;
    pose[6 + 3 * f] = 30 * kDeg;
  }
  const auto encoder = ik_joint_to_actuator(pose);
  HandState state{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) state.actuators.position_count[i] = encoder[i];
  state.actuators.velocity_rpm = rpm;
  state.joints.position_rad = fk_actuator_to_joint(encoder);
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

#if defined(AIDIN_HAND2_HAND_TYPE_C)
void test_type_c()
{
  const HandStateJointVelEff out = compute_joint_vel_eff(state_at_pose(test_rpm()));
  bool all_nan = true;
  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    all_nan = all_nan && std::isnan(out.velocity_rad_s[j]) && std::isnan(out.effort_Nm[j]) &&
              std::isnan(out.commanded_effort_Nm[j]) && std::isnan(out.rated_effort_Nm[j]);
  }
  check(all_nan, "hand type c gives NaN");
}
#else
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
  const auto qp = active_of(fk_actuator_to_joint(plus));
  const auto qm = active_of(fk_actuator_to_joint(minus));
  const HandStateJointVelEff out = compute_joint_vel_eff(state);
  double worst = 0.0, largest = 0.0;
  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    const double fd = (qp[j] - qm[j]) / (2.0 * 60.0 / 1024.0);
    worst = std::max(worst, std::abs(out.velocity_rad_s[j] - fd));
    largest = std::max(largest, std::abs(fd));
  }
  std::printf("  velocity vs FK difference, worst %.2e rad/s of %.2e\n", worst, largest);
  check(std::isfinite(worst) && worst < 0.01 * largest, "velocity differs from the FK difference");
}

void test_virtual_work()
{
  const auto rpm = test_rpm();
  HandState state = state_at_pose(rpm);
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    state.actuators.current_mA[i] = static_cast<std::int16_t>((i % 3 == 0 ? -1 : 1) * (50 + 15 * static_cast<int>(i)));
  }
  const HandStateJointVelEff out = compute_joint_vel_eff(state);
  double joint_power = 0.0, motor_power = 0.0;
  for (std::size_t j = 0; j < kActiveJointCount; ++j) joint_power += out.effort_Nm[j] * out.velocity_rad_s[j];
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    motor_power += kTorqueConstantNmPerA * state.actuators.current_mA[i] / 1000.0 * rpm[i] * 2.0 * 3.14159265358979323846 / 60.0;
  }
  std::printf("  joint power %.6e W, motor power %.6e W\n", joint_power, motor_power);
  check(std::abs(joint_power - motor_power) < 1e-9 * std::abs(motor_power), "joint power differs from motor power");
}

void test_rated_and_commanded()
{
  HandState state = state_at_pose({});
  state.actuators.current_mA[0] = 390;
  const HandStateJointVelEff idle = compute_joint_vel_eff(state);
  check(std::abs(std::abs(idle.effort_Nm[0]) - idle.rated_effort_Nm[0]) < 1e-9 * idle.rated_effort_Nm[0],
        "rated current on thumb_actuator0 gives rated_effort_Nm on thumb_joint0");
  check(std::isnan(idle.commanded_effort_Nm[0]), "no effort output gives NaN");

  ActuatorEffortSetpoint output{};
  output.target_effort_pct[0] = 1000.0;
  state.commanded.controller_output = output;
  const HandStateJointVelEff driven = compute_joint_vel_eff(state);
  check(std::abs(driven.commanded_effort_Nm[0] - idle.effort_Nm[0]) < 1e-9 * idle.rated_effort_Nm[0],
        "effort output 1000 gives the torque of rated current");
  std::printf("  thumb_joint0 rated %.3f N m\n", idle.rated_effort_Nm[0]);
}
#endif

}  // namespace

int main()
{
#if defined(AIDIN_HAND2_HAND_TYPE_C)
  test_type_c();
#else
  test_velocity();
  test_virtual_work();
  test_rated_and_commanded();
#endif
  std::printf("joint_vel_eff test: %s\n", g_failures == 0 ? "OK" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
