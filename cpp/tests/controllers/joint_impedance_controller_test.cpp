// 목적(Tier 1, HW 무관): joint 공간 impedance 제어기를 host 에서 회귀 점검한다.
//   1) joint torque → actuator effort: Jacobian 전치와 효율, 2-rod 쌍과 d3 결합.
//   2) joint impedance torque: stiffness 항, 현재 자세 관성에서 구한 damping 항.
//   3) compute(): FK 각도·rpm 에서 frame 까지, clamp, NaN 자세, home 자세.
//   4) 마찰 보상: Coulomb·위치 표·Stribeck·LuGre 를 impedance effort 를 직접 넣어 본다.
// 기준값은 kinematics archive(type a)로 기준 자세(엄지 20/20/0/20°, 긴 손가락 0/20/20°)에서 구했다.
// type c 에는 Jacobian 이 없으므로 그 빌드에서는 a·b 전용 시험을 건너뛴다.

#include <array>
#include <cmath>
#include <cstdio>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>

#include "hand_core/comms/canfd/protocol.hpp"
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
constexpr long kPeriodNs = 2'000'000;  // 500 Hz

using Config = ControllerConfig::JointImpedanceController;
using Active = std::array<double, kActiveJointCount>;

#if !defined(AIDIN_HAND2_HAND_TYPE_C)
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

// 자세 q 의 IK count 를 actuator 에, 그 count 의 FK 를 joint 에 채운 상태. 실제 루프와 같은 경로다.
HandState state_at(const Active& q)
{
  HandState state{};
  std::array<int, kActuatorCount> count{};
  kinematics::ik_joint_to_actuator(q, count);
  for (std::size_t i = 0; i < kActuatorCount; ++i) state.actuators.position_count[i] = count[i];
  std::array<double, kinematics::JOINT_NUM> joint{};
  std::array<double, kinematics::TASK_NUM> task{};
  kinematics::fk_actuator_to_joint(count, joint, task);
  for (std::size_t k = 0; k < kJointCount; ++k) state.joints.position_rad[k] = joint[k];
  return state;
}

// 21 개 FK 각도에서 passive q4 를 뺀 16 개.
Active active_of(const HandState& state)
{
  Active active{};
  for (std::size_t k = 0; k < 4; ++k) active[k] = state.joints.position_rad[k];
  for (std::size_t f = 0; f < 4; ++f) {
    for (std::size_t k = 0; k < 3; ++k) active[4 + 3 * f + k] = state.joints.position_rad[5 + 4 * f + k];
  }
  return active;
}

// 엄지 j0 에만 K 27, 나머지 0, 마찰 없음.
Config thumb_j0_gains(double damping_ratio)
{
  Config config{};
  config.stiffness_Nm_per_rad.fill(0.0);
  config.stiffness_Nm_per_rad[0] = 27.0;
  config.damping_ratio.fill(damping_ratio);
  return config;
}

canfd::CommandFrames run_once(const Active& target, const Config& config, const HandState& state, double max_effort)
{
  controllers::JointImpedanceController controller(kPeriodNs);
  JointImpedanceCommand command{};
  command.target = target;
  std::array<double, kActuatorCount> limit{};
  limit.fill(max_effort);
  canfd::CommandFrames frames{};
  controller.compute(command, config, state, limit, frames);
  return frames;
}

bool all_cst(const canfd::CommandFrames& frames)
{
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (frames.mode_of_operation[i] != canfd::mode_of_operation::kCST) return false;
  }
  return true;
}

// ── Test 3: compute() ──
void test_compute()
{
  const HandState state = state_at(reference_pose());
  const Active pose = active_of(state);

  // a. 엄지 j0 1° 오차 → 319.57, int16 절단
  Active target = pose;
  target[0] += 1.0 * kDeg;
  const auto stiff = run_once(target, thumb_j0_gains(0.0), state, 30000.0);
  bool others_zero = true;
  for (std::size_t i = 1; i < kActuatorCount; ++i) others_zero = others_zero && stiff.target_effort[i] == 0;
  check(stiff.target_effort[0] >= 318 && stiff.target_effort[0] <= 320 && others_zero && all_cst(stiff),
        "compute: thumb j0 1 deg gives 318..320 on d0 only, CST");

  // b. 오차 0, d0 6000 rpm, ζ 0.7 → −178.06
  HandState moving = state;
  moving.actuators.velocity_rpm[0] = 6000;
  const auto damped = run_once(pose, thumb_j0_gains(0.7), moving, 30000.0);
  check(damped.target_effort[0] >= -179 && damped.target_effort[0] <= -177, "compute: damping from the drive rpm");

  // c. 10° 오차는 max_effort 에서 잘린다
  Active far = pose;
  far[0] += 10.0 * kDeg;
  check(run_once(far, thumb_j0_gains(0.0), state, 1000.0).target_effort[0] == 1000, "compute: clamped to max_effort");

  // d. homing 전처럼 joint 가 NaN 이면 effort 0
  HandState unknown = state;
  unknown.joints.position_rad.fill(std::nan(""));
  const auto blind = run_once(target, thumb_j0_gains(0.7), unknown, 1000.0);
  bool zero = all_cst(blind);
  for (std::size_t i = 0; i < kActuatorCount; ++i) zero = zero && blind.target_effort[i] == 0;
  check(zero, "compute: a NaN pose sends zero effort");

  // e. home 자세(count 0, joint 0), 엄지 j0 1° → 유한하고 max_effort 안
  HandState home{};
  Active home_target{};
  home_target[0] = 1.0 * kDeg;
  const auto edge = run_once(home_target, thumb_j0_gains(0.7), home, 1000.0);
  bool bounded = all_cst(edge);
  for (std::size_t i = 0; i < kActuatorCount; ++i) bounded = bounded && std::abs(edge.target_effort[i]) <= 1000;
  check(bounded && edge.target_effort[0] > 0, "compute: the home pose gives a bounded effort toward the target");
}
#else
// ── Test 3: type c 는 Jacobian 이 없어 모든 actuator 에 CST 0 ──
void test_compute_type_c()
{
  JointImpedanceCommand command{};
  command.target.fill(0.3);
  std::array<double, kActuatorCount> max_effort{};
  max_effort.fill(1000.0);
  controllers::JointImpedanceController controller(kPeriodNs);
  canfd::CommandFrames frames{};
  controller.compute(command, Config{}, HandState{}, max_effort, frames);
  bool zero = true;
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    zero = zero && frames.mode_of_operation[i] == canfd::mode_of_operation::kCST && frames.target_effort[i] == 0;
  }
  check(zero, "compute: type c sends CST 0 to every actuator");
}
#endif

// ── Test 4: 마찰 보상 (impedance effort 를 직접 넣는다) ──
void test_friction_compensation()
{
  using Model = Config::FrictionModel;
  auto near = [](double a, double b) { return std::abs(a - b) < 0.05; };
  std::array<double, kActuatorCount> none{};

  // Coulomb: 이동 중(60 rpm = 1024 count/s) c+ 50 과 점성 0.01 → 60.24. 정지 대역에서는 effort 의 방향을
  // ramp max(300, 2 × 80) = 300 에 걸쳐 따른다. +150 → +25, −150 → −40. scale 0 이면 0.
  {
    Config gains{};
    gains.friction_scale = 1.0;
    gains.coulomb_friction.positive.fill(50.0);
    gains.coulomb_friction.negative.fill(80.0);
    gains.coulomb_friction.viscous.fill(0.01);
    HandState sliding{};
    sliding.actuators.velocity_rpm[0] = 60;
    controllers::JointImpedanceController c(kPeriodNs);
    const auto moving = c.friction_compensation(gains, sliding, none);
    check(near(moving[0], 60.24) && moving[1] == 0.0, "coulomb: moving + adds c+ and viscous, at rest with zero effort nothing");

    std::array<double, kActuatorCount> effort{};
    effort[0] = 150.0;
    effort[1] = -150.0;
    const auto resting = c.friction_compensation(gains, HandState{}, effort);
    check(near(resting[0], 25.0) && near(resting[1], -40.0), "coulomb: stick band follows the effort over the ramp");

    gains.friction_scale = 0.0;
    const auto off = c.friction_compensation(gains, HandState{}, effort);
    check(off[0] == 0.0 && off[1] == 0.0, "coulomb: friction_scale 0 adds nothing");
  }
  // 위치 표: actuator 0 의 표는 count 0~8000 에 8 칸, + 방향 10..80, − 방향 40.
  // count 2500 은 칸 2 중심 → 30, 3000 은 두 중심 사이 → 35, −500 은 첫 칸 → 10, − 이동이면 −40, 정지·effort 0 → 0.
  {
    Config gains{};
    gains.friction_scale = 1.0;
    gains.friction_model = Model::PositionTable;
    auto& table = gains.position_table_friction;
    for (std::size_t k = 0; k < Config::PositionTableFriction::kBins; ++k) {
      table.positive[0][k] = 10.0 * static_cast<double>(k + 1);
      table.negative[0][k] = 40.0;
    }
    table.start[0] = 0.0;
    table.end[0] = 8000.0;
    auto at = [&](std::int32_t count, std::int32_t rpm) {
      HandState s{};
      s.actuators.position_count[0] = count;
      s.actuators.velocity_rpm[0] = rpm;
      controllers::JointImpedanceController c(kPeriodNs);
      return c.friction_compensation(gains, s, none)[0];
    };
    check(near(at(2500, 60), 30.0), "table: bin centre while moving +");
    check(near(at(3000, 60), 35.0), "table: halfway between two bin centres");
    check(near(at(-500, 60), 10.0), "table: the edge value held outside the range");
    check(near(at(2500, -60), -40.0), "table: moving - gives c- at that count");
    check(at(2500, 0) == 0.0, "table: no bias at rest with zero effort");
  }
  // Stribeck: c 50, s 100, stribeck velocity 1024 count/s, shape 2. 60 rpm → 50 + 50·e^−1 = 68.39, 240 rpm → 50.
  {
    Config gains{};
    gains.friction_scale = 1.0;
    gains.friction_model = Model::Stribeck;
    auto& f = gains.stribeck_friction;
    f.positive.fill(50.0);  f.negative.fill(50.0);
    f.static_positive.fill(100.0);  f.static_negative.fill(100.0);
    f.stribeck_velocity.fill(1024.0);
    HandState s{};
    s.actuators.velocity_rpm[0] = 60;
    s.actuators.velocity_rpm[1] = 240;
    controllers::JointImpedanceController c(kPeriodNs);
    const auto comp = c.friction_compensation(gains, s, none);
    check(near(comp[0], 68.39), "stribeck: c + (s - c) e^-1 at the stribeck velocity");
    check(near(comp[1], 50.0), "stribeck: c well above the stribeck velocity");
  }
  // LuGre: bristle stiffness 10, damping 0, g 50. 1024 count/s 로 계속 움직이면 z → g/σ0 = 5, effort 50.
  // 첫 주기는 z = 2.048 / (1 + 10·1024·0.002/50) = 1.4529 → 14.53. reset 하면 다시 첫 주기 값.
  {
    Config gains{};
    gains.friction_scale = 1.0;
    gains.friction_model = Model::LuGre;
    auto& f = gains.lugre_friction;
    f.curve.positive.fill(50.0);  f.curve.negative.fill(50.0);
    f.curve.static_positive.fill(50.0);  f.curve.static_negative.fill(50.0);
    f.bristle_stiffness.fill(10.0);
    HandState s{};
    s.actuators.velocity_rpm[0] = 60;
    controllers::JointImpedanceController c(kPeriodNs);
    check(near(c.friction_compensation(gains, s, none)[0], 14.53), "lugre: the first cycle deflects the bristle by the implicit step");
    double later = 0.0;
    for (int k = 0; k < 200; ++k) later = c.friction_compensation(gains, s, none)[0];
    check(std::abs(later - 50.0) <= 1.0, "lugre: steady sliding reaches g");
    c.reset();
    check(near(c.friction_compensation(gains, s, none)[0], 14.53), "lugre: reset relaxes the bristle");
  }
}
}  // namespace

int main()
{
#if !defined(AIDIN_HAND2_HAND_TYPE_C)
  test_joint_torque_to_effort();
  test_joint_impedance_torque();
  test_compute();
#else
  test_compute_type_c();
#endif
  test_friction_compensation();

  std::printf("joint_impedance_controller test: %s\n", g_failures == 0 ? "OK" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
