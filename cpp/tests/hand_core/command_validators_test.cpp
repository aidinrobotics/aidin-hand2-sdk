// 목적(Tier 1, HW 무관): HandCore 의 command 입력 검증기(is_command_valid 오버로드)와
// set_max_effort 의 [0, 2000] clamp 를 host 에서 black-box 로 점검한다. 소켓은 열지 않는다.
//   - is_command_valid: NaN/Inf 거부 · ActuatorPosition int32 범위 밖 거부 · 유효 command 수락.
//   - set_controller_config: filter·gain 의 NaN/Inf·음수 거부 · 0 수락.
//   - set_max_effort: 범위 내 유지 · 음수 → 0 · 2000 초과 → 2000 · NaN/Inf 거부.
//
// 접근 seam: is_command_valid 는 hand_core.cpp 의 anonymous namespace 자유함수라 직접 호출 불가.
// 대신 public set_command 이 유효성 검사를 감싸므로, 그 반환 Status 로 수락/거부를 관측한다.
// 단, set_command 은 check_allowed(SetCommand) gate 뒤에 있어 갓 생성한 core(Disconnected·unhomed)
// 에서는 검증기에 닿기 전에 "not homed" 로 거부된다. hand_core.hpp 가 friend 로 선언한
// test::HandCoreTestPeer 를 이 TU 에서 정의해(command_to_frames_test.cpp 와 동일 seam) lifecycle_·
// homing_state_ 를 전제조건까지 세팅한 뒤 검증기 경로만 노출한다. set_max_effort 는 gate 밖이라 그대로 호출.
// (production 코드는 수정하지 않는다 — friend 선언은 header 에 이미 존재한다.)

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/error.hpp>

#include "hand_core/hand_core.hpp"
#include "hand_core/kinematics/hand_kinematics_core.hpp"

using namespace aidin_hand2;

namespace aidin_hand2::test
{
// private 멤버에 접근하는 테스트 peer (hand_core.hpp 가 friend 선언). 각 테스트는 별도 실행 파일이라
// command_to_frames_test.cpp 의 동명 peer 와 ODR 충돌하지 않는다 — 필요한 접근만 노출한다.
struct HandCoreTestPeer {
  // set_command 의 check_allowed(SetCommand) gate 통과 전제조건을 세운다: Running + homed.
  // 이후 set_command 은 is_command_valid 검증기로 진입한다(소켓·comms 접근 없음).
  static void open_command_gate(HandCore& core)
  {
    core.requested_lifecycle_.store(HandLifecycle::Running);
    core.lifecycle_.store(HandLifecycle::Running);
    core.homing_state_.store(HomingState::Succeeded);
  }
  // set_max_effort 가 staging 에 남긴 clamp 결과를 읽는다.
  static std::array<double, kActuatorCount> staged_max_effort(const HandCore& core)
  {
    std::lock_guard<std::mutex> lock(core.staging_mutex_);
    return core.staging_command_.max_effort_pct;
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

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

HandConfig make_config()
{
  HandConfig config{"can0", HandSide::Right};  // 절대 open 안 함 (생성만)
  config.control_rate = 500;
  return config;
}

// set_controller_config·set_max_effort 는 여전히 실패 Status 를 돌려준다.
bool rejected(const Status& status)
{
  return !status.ok() && status.code == ErrorCode::InvalidArgument;
}

// set_command 는 값이 잘못돼도 예외 없이 버리므로 Status 로는 안 보인다.
// nan_command_count 가 1 늘었는지로 관측한다.
template <class Command>
bool dropped(HandCore& core, const Command& command)
{
  const std::uint64_t before = core.diagnostics().nan_command_count;
  const Status status = core.set_command(command);
  return status.ok() && core.diagnostics().nan_command_count == before + 1;
}

// ── Test 1: 유효 command 는 수락 ──
void test_valid_commands_accepted(HandCore& core)
{
  {
    JointPositionCommand cmd{};      // target 0, speed 0 — 유효
    check(core.set_command(cmd).ok(), "valid JointPosition accepted");
  }
  {
    JointImpedanceCommand cmd{};     // 기본 gain(양수), target 0 — 유효
    check(core.set_command(cmd).ok(), "valid JointImpedance accepted");
  }
  {
    ActuatorPositionCommand cmd{};   // target 0 (int32 범위 안) — 유효
    check(core.set_command(cmd).ok(), "valid ActuatorPosition accepted");
  }
  {
    ActuatorEffortCommand cmd{};     // target 0 — 유효
    check(core.set_command(cmd).ok(), "valid ActuatorEffort accepted");
  }
}

// ── Test 2: NaN/Inf 는 거부 ──
void test_non_finite_rejected(HandCore& core)
{
  {
    JointPositionCommand cmd{};  cmd.target[0] = kNaN;
    check(dropped(core, cmd), "JointPosition NaN target rejected");
  }
  {
    JointPositionCommand cmd{};  cmd.target[3] = kInf;
    check(dropped(core, cmd), "JointPosition Inf target rejected");
  }
  {
    JointImpedanceCommand cmd{};  cmd.target[2] = kInf;
    check(dropped(core, cmd), "JointImpedance Inf target rejected");
  }
  {
    ActuatorPositionCommand cmd{};  cmd.target[0] = kNaN;
    check(dropped(core, cmd), "ActuatorPosition NaN target rejected");
  }
  {
    ActuatorEffortCommand cmd{};  cmd.target[4] = kInf;
    check(dropped(core, cmd), "ActuatorEffort Inf target rejected");
  }
}

// ── Test 3: set_controller_config — filter 값은 유한·음수 아님 ──
void test_controller_config_filter_rejected(HandCore& core)
{
  {
    ControllerConfig config{};  config.joint_position_controller.cutoff_freq = kNaN;
    check(rejected(core.set_controller_config(config)), "cutoff_freq NaN rejected");
  }
  {
    ControllerConfig config{};  config.joint_position_controller.cutoff_freq = -1.0;
    check(rejected(core.set_controller_config(config)), "cutoff_freq negative rejected");
  }
  {
    ControllerConfig config{};  config.joint_position_controller.deadband = kInf;
    check(rejected(core.set_controller_config(config)), "deadband Inf rejected");
  }
  {
    ControllerConfig config{};  config.joint_position_controller.deadband = -1e-6;
    check(rejected(core.set_controller_config(config)), "deadband negative rejected");
  }
  {
    ControllerConfig config{};  // 경계: 0 은 해당 단계를 끄는 값이라 허용
    config.joint_position_controller.cutoff_freq = 0.0;
    config.joint_position_controller.deadband    = 0.0;
    check(core.set_controller_config(config).ok(), "zero cutoff_freq and deadband accepted");
  }
}

// ── Test 4: set_controller_config — gain 은 유한·음수 아님 ──
void test_controller_config_gains_rejected(HandCore& core)
{
  {
    ControllerConfig config{};  config.joint_impedance_controller.stiffness[0] = kNaN;
    check(rejected(core.set_controller_config(config)), "stiffness NaN rejected");
  }
  {
    ControllerConfig config{};  config.joint_impedance_controller.damping[1] = kInf;
    check(rejected(core.set_controller_config(config)), "damping Inf rejected");
  }
  {
    ControllerConfig config{};  config.joint_impedance_controller.stiffness[0] = -1.0;
    check(rejected(core.set_controller_config(config)), "stiffness negative rejected");
  }
  {
    ControllerConfig config{};  config.joint_impedance_controller.damping[5] = -1e-6;
    check(rejected(core.set_controller_config(config)), "damping negative rejected");
  }
  {
    ControllerConfig config{};  // 경계: 0 gain 허용
    config.joint_impedance_controller.stiffness.fill(0.0);
    config.joint_impedance_controller.damping.fill(0.0);
    check(core.set_controller_config(config).ok(), "zero gains accepted");
  }
}

// ── Test 5: ActuatorPosition int32 범위 밖은 거부 ──
void test_actuator_position_int32_range(HandCore& core)
{
  constexpr double kMax = static_cast<double>(std::numeric_limits<std::int32_t>::max());
  constexpr double kMin = static_cast<double>(std::numeric_limits<std::int32_t>::min());

  {
    ActuatorPositionCommand cmd{};  cmd.target[0] = kMax + 1e9;  // > INT32_MAX
    check(dropped(core, cmd), "ActuatorPosition above int32 max rejected");
  }
  {
    ActuatorPositionCommand cmd{};  cmd.target[7] = kMin - 1e9;  // < INT32_MIN
    check(dropped(core, cmd), "ActuatorPosition below int32 min rejected");
  }
  {
    ActuatorPositionCommand cmd{};  cmd.target.fill(kMax);  // 경계: INT32_MAX 는 허용
    check(core.set_command(cmd).ok(), "ActuatorPosition at int32 max accepted");
  }
  {
    ActuatorPositionCommand cmd{};  cmd.target.fill(kMin);  // 경계: INT32_MIN 는 허용
    check(core.set_command(cmd).ok(), "ActuatorPosition at int32 min accepted");
  }
}

// ── Test 6: set_max_effort clamp to [0, 2000] (gate 밖 — 직접 호출) ──
void test_set_max_effort_clamp(HandCore& core)
{
  std::array<double, kActuatorCount> limit{};
  limit.fill(1500.0);  // 범위 내 기준값
  limit[0] = -100.0;   // 음수 → 0 으로 clamp
  limit[1] = 5000.0;   // 2000 초과 → 2000 으로 clamp
  limit[2] = 0.0;      // 경계 하한
  limit[3] = 2000.0;   // 경계 상한

  check(core.set_max_effort(limit).ok(), "set_max_effort valid returns ok");

  const auto staged = HandCoreTestPeer::staged_max_effort(core);
  check(staged[0] == 0.0,    "negative max effort clamped to 0");
  check(staged[1] == 2000.0, "above-2000 max effort clamped to 2000");
  check(staged[2] == 0.0,    "max effort lower bound 0 kept");
  check(staged[3] == 2000.0, "max effort upper bound 2000 kept");
  check(staged[4] == 1500.0, "in-range max effort kept");

  // NaN/Inf limit 은 거부(clamp 이전 finite gate).
  {
    std::array<double, kActuatorCount> bad{};  bad.fill(1000.0);  bad[0] = kNaN;
    check(rejected(core.set_max_effort(bad)), "set_max_effort NaN rejected");
  }
  {
    std::array<double, kActuatorCount> bad{};  bad.fill(1000.0);  bad[2] = kInf;
    check(rejected(core.set_max_effort(bad)), "set_max_effort Inf rejected");
  }
}

}  // namespace

int main()
{
  HandCore core(make_config());
  HandCoreTestPeer::open_command_gate(core);  // check_allowed(SetCommand) 전제조건: Running + homed

  test_valid_commands_accepted(core);
  test_non_finite_rejected(core);
  test_controller_config_filter_rejected(core);
  test_controller_config_gains_rejected(core);
  test_actuator_position_int32_range(core);
  test_set_max_effort_clamp(core);

  if (g_failures == 0) std::printf("command_validators_test: PASS\n");
  else                 std::printf("command_validators_test: FAIL (%d)\n", g_failures);
  return g_failures;
}
