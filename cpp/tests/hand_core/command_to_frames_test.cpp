// 목적(Tier 1, HW 무관): HandCore 의 command→frames 변환과 FK joint 채움을 host 에서 회귀 점검한다.
//   - JointPosition FK∘IK 왕복: 도달가능 pose(rad) → command_to_frames(IK→encoder) → fill_joint_state(FK)
//     → 복원 rad 가 원본과 일치? index 매핑·IK/FK 정합을 한 번에 검증.
//   - fault-mask: faulted/미-enabled actuator 는 CSP=현재위치 hold, CST=0.
//   - effort clamp: ±max_effort, max_effort=0 → 0.
//   - impedance 법칙: effort = K·err − D·vel 의 부호·배선.
// device-in-the-loop 아님(HW 거동은 별도 HW probe). HandCore 는 생성만(소켓 open 안 함).

#include <array>
#include <cmath>
#include <cstdio>

#include <aidin_hand2/types/config.hpp>

#include "hand_core/controllers/joint_impedance_controller.hpp"
#include "hand_core/hand_core.hpp"
#include "hand_core/kinematics/hand_kinematics_core.hpp"
#include <aidin_hand2/types/command.hpp>

using namespace aidin_hand2;

namespace aidin_hand2::test
{
// private 변환에 접근하는 테스트 peer (hand_core.hpp 가 friend 선언).
struct HandCoreTestPeer {
  static canfd::CommandFrames command_to_frames(
      HandCore& core, const HandCommand& command, const HandState& state,
      const ActuatorHealth& health, const ControllerConfig& config = ControllerConfig{})
  {
    return core.command_to_frames(command, config, state, health);
  }
  static void fill_joint_state(HandCore& core, HandState& state)
  {
    core.fill_joint_state(state);
  }
};
}  // namespace aidin_hand2::test

namespace
{
using test::HandCoreTestPeer;

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) { std::printf("  FAIL: %s\n", what); ++g_failures; }
}

HandConfig make_config()
{
  HandConfig config{"can0", HandSide::Right};  // 절대 open 안 함 (생성만); Right = CAN index identity
  config.control_rate   = 500;
  return config;
}

HandState healthy_state()
{
  return HandState{};
}

// active_joint(16) 인덱스 → joint(21) 인덱스. thumb q0..3, long finger 별 q1..3 (passive q4 제외).
std::array<std::size_t, kActiveJointCount> active_to_joint_index()
{
  std::array<std::size_t, kActiveJointCount> map{};
  for (std::size_t i = 0; i < 4; ++i) map[i] = i;  // thumb
  for (std::size_t f = 0; f < 4; ++f) {
    const std::size_t active_base = 4 + 3 * f;
    const std::size_t joint_base  = 5 + 4 * f;
    for (std::size_t k = 0; k < 3; ++k) map[active_base + k] = joint_base + k;
  }
  return map;
}

// ── Test 1: JointPosition FK∘IK 왕복 ──
void test_joint_position_roundtrip(HandCore& core)
{
  const auto map = active_to_joint_index();
  constexpr double kTolRad = 0.0175;  // ≈ 1 deg

  double max_error = 0.0;
  int    compared  = 0;
  int    skipped   = 0;

  for (double encoder_scalar : {0.0, 800.0, 1600.0, 2400.0, 3200.0, 4000.0}) {
    // (a) 기준 encoder → FK → active_ref(rad).
    HandState s0 = healthy_state();
    s0.actuators.position_count.fill(encoder_scalar);
    HandCoreTestPeer::fill_joint_state(core, s0);

    std::array<double, kActiveJointCount> active_ref{};
    bool reachable = true;
    for (std::size_t k = 0; k < kActiveJointCount; ++k) {
      active_ref[k] = s0.joints.position_rad[map[k]];
      if (!std::isfinite(active_ref[k])) reachable = false;
    }
    if (!reachable) { ++skipped; continue; }  // LUT 밖 샘플 — 건너뜀

    // (b) active_ref(rad) → command_to_frames(IK) → encoder_rt.
    HandCommand command{};
    JointPositionCommand joint_position{};
    joint_position.target = active_ref;
    command.controller = joint_position;
    command.max_effort_pct.fill(30000.0);
    ActuatorHealth health{};
    health.enabled.fill(true);
    // JointPositionController 의 filter 때문에 정착까지 같은 명령을 반복한다. 이 테스트가
    // 보려는 건 IK/FK 왕복이고, filter 상태 로직은 joint_position_controller_test 가 본다.
    // 정착값은 deadband(0.05°) 만큼 목표에 못 미치는데, 이는 허용오차(1°)의 5% 다.
    canfd::CommandFrames frames{};
    for (int settle = 0; settle < 400; ++settle)
      frames = HandCoreTestPeer::command_to_frames(core, command, healthy_state(), health);

    // (c) encoder_rt → FK → active_rt(rad), 원본과 비교.
    HandState s1 = healthy_state();
    for (std::size_t i = 0; i < kActuatorCount; ++i)
      s1.actuators.position_count[i] = static_cast<double>(frames.target_position[i]);
    HandCoreTestPeer::fill_joint_state(core, s1);

    for (std::size_t k = 0; k < kActiveJointCount; ++k) {
      const double rt = s1.joints.position_rad[map[k]];
      check(std::isfinite(rt), "roundtrip joint finite");
      const double error = std::fabs(rt - active_ref[k]);
      if (error > max_error) max_error = error;
      ++compared;
    }
  }

  std::printf("[roundtrip] compared %d joints (%d samples skipped), max|err| = %.5f rad (tol %.4f)\n",
              compared, skipped, max_error, kTolRad);
  check(compared > 0, "roundtrip has comparable samples");
  check(max_error < kTolRad, "roundtrip within tolerance");
}

// ── Test 2: fault-mask ──
void test_fault_mask(HandCore& core)
{
  // CSP(ActuatorPosition): faulted → 현재위치 hold, 정상은 목표.
  {
    HandState state = healthy_state();
    state.actuators.position_count[3] = 1234.0;
    ActuatorHealth health{};
    health.enabled.fill(true);
    health.fault[3] = ActuatorFault::OverCurrentError;
    HandCommand command{};
    ActuatorPositionCommand actuator_position{};
    actuator_position.target.fill(5000.0);
    command.controller = actuator_position;
    command.max_effort_pct.fill(500.0);
    const auto frames = HandCoreTestPeer::command_to_frames(core, command, state, health);
    check(frames.target_position[3] == 1234, "CSP faulted holds actual position");
    check(frames.target_position[0] == 5000, "CSP healthy follows target");
  }
  // CSP: 미-enabled → 현재위치 hold.
  {
    HandState state = healthy_state();
    state.actuators.position_count[5] = 777.0;
    ActuatorHealth health{};
    health.enabled.fill(true);
    health.enabled[5] = false;
    HandCommand command{};
    ActuatorPositionCommand actuator_position{};
    actuator_position.target.fill(5000.0);
    command.controller = actuator_position;
    const auto frames = HandCoreTestPeer::command_to_frames(core, command, state, health);
    check(frames.target_position[5] == 777, "CSP disabled holds actual position");
  }
  // CST(ActuatorEffort): faulted → 0 토크.
  {
    HandState state = healthy_state();
    ActuatorHealth health{};
    health.enabled.fill(true);
    health.fault[7] = ActuatorFault::HallSensorError;
    HandCommand command{};
    ActuatorEffortCommand actuator_effort{};
    actuator_effort.target.fill(300.0);
    command.controller = actuator_effort;
    command.max_effort_pct.fill(500.0);
    const auto frames = HandCoreTestPeer::command_to_frames(core, command, state, health);
    check(frames.target_effort[7] == 0, "CST faulted zero torque");
    check(frames.target_effort[0] == 300, "CST healthy follows target");
  }
}

// ── Test 3: effort clamp ──
void test_effort_clamp(HandCore& core)
{
  HandState state = healthy_state();
  ActuatorHealth health{};
  health.enabled.fill(true);
  HandCommand command{};
  ActuatorEffortCommand actuator_effort{};
  actuator_effort.target = {10000.0, -10000.0, 300.0};  // 나머지 0
  command.controller = actuator_effort;
  command.max_effort_pct = {500.0, 500.0, 0.0};  // 나머지 0
  const auto frames = HandCoreTestPeer::command_to_frames(core, command, state, health);
  check(frames.target_effort[0] == 500, "effort clamps to +max_effort");
  check(frames.target_effort[1] == -500, "effort clamps to -max_effort");
  check(frames.target_effort[2] == 0, "max_effort=0 zeroes torque");
}

// ── Test 4: impedance 법칙 (K 배선 + D 부호) ──
// controller 가 속도항 상태를 소유하므로 직접 단위 테스트한다(D 항은 두 cycle: seed → 측정).
void test_impedance_law()
{
  constexpr long kPeriodNs = 2'000'000;  // 500 Hz → dt = 0.002 s
  std::array<double, kActuatorCount> max_effort{};
  max_effort.fill(30000.0);

  // K 항: damping 0(속도 무관), 같은 target, actuator0 만 position 다르게 → Δeffort = K·(pos_b − pos_a).
  {
    controllers::JointImpedanceController controller(kPeriodNs);
    JointImpedanceCommand impedance{};
    impedance.target.fill(0.0);
    ControllerConfig::JointImpedanceController gains{};
    gains.stiffness.fill(1.0);
    gains.damping.fill(0.0);

    HandState a = healthy_state();  a.actuators.position_count[0] = 100.0;
    HandState b = healthy_state();  b.actuators.position_count[0] = 200.0;
    canfd::CommandFrames fa{}, fb{};
    controller.compute(impedance, gains, a, max_effort, fa);
    controller.compute(impedance, gains, b, max_effort, fb);
    const int delta = fa.target_effort[0] - fb.target_effort[0];  // K·(200−100) = 100
    check(std::abs(delta - 100) <= 1, "impedance K term wiring (ΔK·Δpos)");
  }
  // D 항: stiffness 0. 첫 cycle seed(속도 0) → 둘째 cycle 에서 +속도 → −effort.
  {
    controllers::JointImpedanceController controller(kPeriodNs);
    JointImpedanceCommand impedance{};
    impedance.target.fill(0.0);
    ControllerConfig::JointImpedanceController gains{};
    gains.stiffness.fill(0.0);
    gains.damping.fill(0.001);

    HandState s1 = healthy_state();  s1.actuators.position_count[0] = 100.0;  // seed
    HandState s2 = healthy_state();  s2.actuators.position_count[0] = 200.0;  // vel = (200−100)/0.002 = 50000
    canfd::CommandFrames f1{}, f2{};
    controller.compute(impedance, gains, s1, max_effort, f1);
    check(f1.target_effort[0] == 0, "impedance first cycle zero velocity");
    controller.compute(impedance, gains, s2, max_effort, f2);
    check(f2.target_effort[0] == -50, "impedance D term sign/magnitude (−D·vel)");
  }
  // 무 gain → 무토크.
  {
    controllers::JointImpedanceController controller(kPeriodNs);
    JointImpedanceCommand impedance{};
    impedance.target.fill(0.0);
    ControllerConfig::JointImpedanceController gains{};
    gains.stiffness.fill(0.0);
    gains.damping.fill(0.0);
    HandState state = healthy_state();  state.actuators.position_count[0] = 4321.0;
    canfd::CommandFrames frames{};
    controller.compute(impedance, gains, state, max_effort, frames);
    check(frames.target_effort[0] == 0, "zero gains give zero torque");
  }
  // P 포화: stiffness 1, 오차 5000 → p_saturation 1000 이면 1000 에서 멈추고(max_effort 2000 보다 먼저),
  // 0 이면 기존대로 max_effort 2000 에서 잘린다. 포화 뒤에도 D 항은 더해진다(rpm 속도 + 로 −effort).
  {
    std::array<double, kActuatorCount> max2000{};
    max2000.fill(2000.0);
    JointImpedanceCommand impedance{};
    impedance.target.fill(0.0);
    ControllerConfig::JointImpedanceController gains{};
    gains.stiffness.fill(1.0);
    gains.damping.fill(0.0);
    HandState far = healthy_state();  far.actuators.position_count[0] = -5000.0;  // error = +5000

    controllers::JointImpedanceController plain(kPeriodNs);
    canfd::CommandFrames fp{};
    plain.compute(impedance, gains, far, max2000, fp);
    check(fp.target_effort[0] == 2000, "default law keeps the max_effort clamp only");

    gains.p_saturation_effort = 1000.0;
    controllers::JointImpedanceController saturating(kPeriodNs);
    canfd::CommandFrames fs{};
    saturating.compute(impedance, gains, far, max2000, fs);
    check(fs.target_effort[0] == 1000, "p_saturation caps the stiffness term at 1000");

    gains.damping.fill(0.1);
    gains.velocity_from_rpm = true;
    far.actuators.velocity_rpm[0] = 60;  // 1024 count/s → D term −102.4
    controllers::JointImpedanceController damped(kPeriodNs);
    canfd::CommandFrames fd{};
    damped.compute(impedance, gains, far, max2000, fd);
    check(std::abs(fd.target_effort[0] - (1000 - 102)) <= 1, "damping still acts after the stiffness term saturates");
  }
  // rpm 속도원: 첫 cycle 부터 drive 의 rpm 을 쓴다(차분과 달리 seed 가 필요 없다). 60 rpm = 1024 count/s.
  {
    controllers::JointImpedanceController controller(kPeriodNs);
    JointImpedanceCommand impedance{};
    impedance.target.fill(0.0);
    ControllerConfig::JointImpedanceController gains{};
    gains.stiffness.fill(0.0);
    gains.damping.fill(0.1);
    gains.velocity_from_rpm = true;
    HandState s = healthy_state();  s.actuators.velocity_rpm[0] = 60;
    canfd::CommandFrames f{};
    controller.compute(impedance, gains, s, max_effort, f);
    check(f.target_effort[0] == -102, "velocity_from_rpm uses 1024 count per revolution on the first cycle");
  }
  // 속도 필터: cutoff 가 있으면 첫 cycle 은 alpha = 1 − exp(−2π f dt) 만큼만 반영된다(500 Hz, 50 Hz → 0.466).
  {
    controllers::JointImpedanceController controller(kPeriodNs);
    JointImpedanceCommand impedance{};
    impedance.target.fill(0.0);
    ControllerConfig::JointImpedanceController gains{};
    gains.stiffness.fill(0.0);
    gains.damping.fill(1.0);
    gains.velocity_from_rpm = true;
    gains.velocity_cutoff_hz = 50.0;
    HandState s = healthy_state();  s.actuators.velocity_rpm[0] = 60;  // 1024 count/s
    canfd::CommandFrames f1{}, f2{};
    controller.compute(impedance, gains, s, max_effort, f1);
    check(std::abs(f1.target_effort[0] - (-477)) <= 1, "velocity filter first cycle = alpha * v");
    controller.compute(impedance, gains, s, max_effort, f2);
    check(std::abs(f2.target_effort[0] - (-732)) <= 1, "velocity filter converges toward v");
    controller.reset();
    canfd::CommandFrames f3{};
    controller.compute(impedance, gains, s, max_effort, f3);
    check(std::abs(f3.target_effort[0] - (-477)) <= 1, "reset clears the velocity filter");
  }
  // 교차 gain: index 쌍(4, 5)에 k 1, kx 0.5. 같은 방향 오차 +100/+100 → 150 씩(공통 모드 k + kx),
  // 반대 방향 +100/−100 → +50/−50(차동 모드 k − kx). 쌍 밖의 actuator 3 은 그대로이고, 교차 damping 은
  // 짝의 rpm 속도에 걸린다(짝만 60 rpm = 1024 count/s, dx 0.1 → −102).
  {
    ControllerConfig::JointImpedanceController gains{};
    gains.stiffness.fill(1.0);
    gains.damping.fill(0.0);
    gains.cross_stiffness[1] = 0.5;
    JointImpedanceCommand impedance{};
    impedance.target.fill(0.0);
    std::array<int, kActuatorCount> home{};
    kinematics::ik_joint_to_actuator(impedance.target, home);

    HandState same = healthy_state();
    for (std::size_t i = 0; i < kActuatorCount; ++i) same.actuators.position_count[i] = home[i];
    same.actuators.position_count[4] = home[4] - 100;  same.actuators.position_count[5] = home[5] - 100;
    same.actuators.position_count[3] = home[3] - 100;
    controllers::JointImpedanceController common(kPeriodNs);
    canfd::CommandFrames fc{};
    common.compute(impedance, gains, same, max_effort, fc);
    check(fc.target_effort[4] == 150 && fc.target_effort[5] == 150, "common-mode error sees k + kx on both actuators");
    check(fc.target_effort[3] == 100, "an actuator outside a pair keeps its own gain only");

    HandState opposite = healthy_state();
    for (std::size_t i = 0; i < kActuatorCount; ++i) opposite.actuators.position_count[i] = home[i];
    opposite.actuators.position_count[4] = home[4] - 100;  opposite.actuators.position_count[5] = home[5] + 100;
    controllers::JointImpedanceController differential(kPeriodNs);
    canfd::CommandFrames fo{};
    differential.compute(impedance, gains, opposite, max_effort, fo);
    check(fo.target_effort[4] == 50 && fo.target_effort[5] == -50, "differential-mode error sees k - kx");

    gains.stiffness.fill(0.0);
    gains.cross_stiffness[1] = 0.0;
    gains.cross_damping[1] = 0.1;
    gains.velocity_from_rpm = true;
    HandState moving = healthy_state();
    for (std::size_t i = 0; i < kActuatorCount; ++i) moving.actuators.position_count[i] = home[i];
    moving.actuators.velocity_rpm[5] = 60;
    controllers::JointImpedanceController damped(kPeriodNs);
    canfd::CommandFrames fm{};
    damped.compute(impedance, gains, moving, max_effort, fm);
    check(fm.target_effort[4] == -102 && fm.target_effort[5] == 0, "cross damping acts on the partner's velocity");
  }
  // 마찰 보상: 움직일 때(60 rpm = 1024 count/s) coulomb+ 50 과 점성 0.01 → +60. 정지 대역에서는 impedance effort 의
  // 방향을 ramp(100)로 따라가므로 오차 +50(effort 50) → 방향 0.5 → +25, 오차 −50 → coulomb− 80 의 절반 −40.
  // scale 0 이면 아무것도 더하지 않는다.
  {
    ControllerConfig::JointImpedanceController gains{};
    gains.stiffness.fill(0.0);
    gains.damping.fill(0.0);
    gains.velocity_from_rpm = true;
    gains.friction_scale = 1.0;
    gains.friction_stick_velocity = 100.0;
    gains.friction_stick_ramp = 100.0;
    gains.friction_coulomb_positive.fill(50.0);
    gains.friction_coulomb_negative.fill(80.0);
    gains.friction_viscous.fill(0.01);
    JointImpedanceCommand impedance{};
    impedance.target.fill(0.0);
    std::array<int, kActuatorCount> home{};
    kinematics::ik_joint_to_actuator(impedance.target, home);

    HandState moving = healthy_state();
    for (std::size_t i = 0; i < kActuatorCount; ++i) moving.actuators.position_count[i] = home[i];
    moving.actuators.velocity_rpm[0] = 60;
    controllers::JointImpedanceController sliding(kPeriodNs);
    canfd::CommandFrames fs{};
    sliding.compute(impedance, gains, moving, max_effort, fs);
    check(fs.target_effort[0] == 60, "moving: coulomb+ plus viscous in the direction of motion");
    check(fs.target_effort[1] == 0, "at rest with zero effort nothing is added");

    gains.stiffness.fill(1.0);
    HandState resting = healthy_state();
    for (std::size_t i = 0; i < kActuatorCount; ++i) resting.actuators.position_count[i] = home[i];
    resting.actuators.position_count[0] = home[0] - 50;  // error +50
    resting.actuators.position_count[1] = home[1] + 50;  // error −50
    controllers::JointImpedanceController sticking(kPeriodNs);
    canfd::CommandFrames fr{};
    sticking.compute(impedance, gains, resting, max_effort, fr);
    check(fr.target_effort[0] == 75, "stick band: half the ramp toward + adds half of coulomb+");
    check(fr.target_effort[1] == -90, "stick band: half the ramp toward - adds half of coulomb-");

    gains.friction_scale = 0.0;
    controllers::JointImpedanceController off(kPeriodNs);
    canfd::CommandFrames fo{};
    off.compute(impedance, gains, resting, max_effort, fo);
    check(fo.target_effort[0] == 50 && fo.target_effort[1] == -50, "friction_scale 0 leaves the law alone");
  }
  // 위치 표 모델: actuator 0 의 표는 count 0~8000 에 8 칸(칸 1000), + 방향 10, 20, …, 80, − 방향 전부 40.
  // count 2500 은 칸 2 의 중심이라 + 30, count 3000 은 칸 2·3 중심 사이라 + 35. 이동 중(+) 이면 c+ 그대로,
  // 범위 밖(count −500)은 첫 칸 값 10. bias = (c+ − c−)/2 는 정지 대역에서 bias_at_rest 면 그대로, 아니면 |d| 배.
  {
    ControllerConfig::JointImpedanceController gains{};
    gains.stiffness.fill(0.0);
    gains.damping.fill(0.0);
    gains.velocity_from_rpm = true;
    gains.friction_scale = 1.0;
    gains.friction_model = ControllerConfig::JointImpedanceController::FrictionModel::PositionTable;
    for (std::size_t k = 0; k < ControllerConfig::JointImpedanceController::kFrictionTableBins; ++k) {
      gains.friction_table_positive[0][k] = 10.0 * static_cast<double>(k + 1);
      gains.friction_table_negative[0][k] = 40.0;
    }
    gains.friction_table_start[0] = 0.0;
    gains.friction_table_end[0] = 8000.0;
    JointImpedanceCommand impedance{};
    impedance.target.fill(0.0);
    std::array<int, kActuatorCount> home{};
    kinematics::ik_joint_to_actuator(impedance.target, home);
    auto effort_at = [&](double count, std::int32_t rpm, double stiffness, bool bias_at_rest) {
      ControllerConfig::JointImpedanceController g = gains;
      g.stiffness.fill(stiffness);
      g.friction_bias_at_rest = bias_at_rest;
      HandState s = healthy_state();
      for (std::size_t i = 0; i < kActuatorCount; ++i) s.actuators.position_count[i] = home[i];
      s.actuators.position_count[0] = static_cast<std::int32_t>(count);
      s.actuators.velocity_rpm[0] = rpm;
      controllers::JointImpedanceController c(kPeriodNs);
      canfd::CommandFrames f{};
      c.compute(impedance, g, s, max_effort, f);
      return static_cast<int>(f.target_effort[0]);
    };
    check(effort_at(2500.0, 60, 0.0, false) == 30, "table: bin centre while moving +");
    check(effort_at(3000.0, 60, 0.0, false) == 35, "table: halfway between two bin centres");
    check(effort_at(-500.0, 60, 0.0, false) == 10, "table: the edge value held outside the range");
    check(effort_at(2500.0, -60, 0.0, false) == -40, "table: moving - gives c- at that count");
    // 정지 대역, 오차 0 (count 2500 = 목표가 home 이므로 오차 −2500 이지만 stiffness 0 이라 effort 0):
    // bias = (30 − 40)/2 = −5. bias_at_rest 면 −5, 아니면 0.
    check(effort_at(2500.0, 0, 0.0, true) == -5, "table: the bias is held at rest with friction_bias_at_rest");
    check(effort_at(2500.0, 0, 0.0, false) == 0, "table: no bias at rest without it");
  }
  // 속도 soft limit: 1200 rpm(20480 count/s)부터 count/s 당 0.01 의 제동. 1500 rpm(25600 count/s)에서는 넘친 5120 ×
  // 0.01 = 51 을 운동 방향 반대로 더하고, 900 rpm 에서는 아무것도 더하지 않는다. 음의 속도면 부호가 반대다.
  {
    ControllerConfig::JointImpedanceController gains{};
    gains.stiffness.fill(0.0);
    gains.damping.fill(0.0);
    gains.velocity_from_rpm = true;
    gains.velocity_soft_limit = 20480.0;
    gains.velocity_brake_gain = 0.01;
    JointImpedanceCommand impedance{};
    impedance.target.fill(0.0);
    HandState s = healthy_state();
    s.actuators.velocity_rpm[0] = 1500;
    s.actuators.velocity_rpm[1] = 900;
    s.actuators.velocity_rpm[2] = -1500;
    controllers::JointImpedanceController controller(kPeriodNs);
    canfd::CommandFrames f{};
    controller.compute(impedance, gains, s, max_effort, f);
    check(f.target_effort[0] == -51, "above the soft limit the brake opposes the motion");
    check(f.target_effort[1] == 0, "below the soft limit nothing is added");
    check(f.target_effort[2] == 51, "the brake follows the sign of the velocity");
  }
}

}  // namespace

int main()
{
  HandCore core(make_config());

  test_joint_position_roundtrip(core);
  test_fault_mask(core);
  test_effort_clamp(core);
  test_impedance_law();

  std::printf("command_to_frames test: %s\n", g_failures == 0 ? "OK" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
