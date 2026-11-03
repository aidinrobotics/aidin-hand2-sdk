// 목적(Tier 1, HW 무관): HandCore 의 command→frames 변환과 FK joint 채움을 host 에서 회귀 점검한다.
//   - JointPosition FK∘IK 왕복: 도달가능 pose(rad) → command_to_frames(IK→encoder) → fill_joint_state(FK)
//     → 복원 rad 가 원본과 일치? index 매핑·IK/FK 정합을 한 번에 검증.
//   - fault-mask: faulted/미-enabled actuator 는 CSP=현재위치 hold, CST=0.
//   - effort clamp: ±max_effort, max_effort=0 → 0.
//   - friction file: Coulomb 모델 채움, 다른 format 거부.
// device-in-the-loop 아님(HW 거동은 별도 HW probe). HandCore 는 생성만(소켓 open 안 함).

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include <aidin_hand2/types/config.hpp>

#include "hand_core/hand_core.hpp"
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

// ── Test 4: friction file ──
void test_friction_file()
{
  const std::string path = "command_to_frames_test_friction.json";
  {
    std::ofstream file(path);
    file << R"({"format": "aidin-hand2-friction/1", "actuators": [)"
         << R"({"index": 1, "coulombPositive": 264.7, "coulombNegative": 166.8, "viscous": 0.0029}]})";
  }
  ControllerConfig::JointImpedanceController config{};
  config.friction_model = ControllerConfig::JointImpedanceController::FrictionModel::PositionTable;
  config.friction_scale = 0.7;
  load_friction_file(path, config);
  check(config.friction_model == ControllerConfig::JointImpedanceController::FrictionModel::Coulomb,
        "friction file selects the Coulomb model");
  check(config.coulomb_friction.positive[1] == 264.7 && config.coulomb_friction.negative[1] == 166.8 &&
            config.coulomb_friction.viscous[1] == 0.0029,
        "friction file fills the listed actuator");
  check(config.coulomb_friction.positive[0] == 0.0, "friction file leaves the others zero");
  check(config.friction_scale == 0.7, "friction file keeps friction_scale");

  {
    std::ofstream file(path);
    file << R"({"format": "something else", "actuators": []})";
  }
  bool rejected = false;
  try {
    load_friction_file(path, config);
  } catch (const Exception& error) {
    rejected = error.code() == ErrorCode::InvalidArgument;
  }
  check(rejected, "friction file with another format is rejected");
  std::remove(path.c_str());
}

}  // namespace

int main()
{
  HandCore core(make_config());

  test_joint_position_roundtrip(core);
  test_fault_mask(core);
  test_effort_clamp(core);
  test_friction_file();

  std::printf("command_to_frames test: %s\n", g_failures == 0 ? "OK" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
