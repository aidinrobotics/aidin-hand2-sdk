// 목적(Tier 1, HW 무관): 안전 회귀 가드 3종을 host 에서 black-box 로 점검한다.
//   (a) fault-mask 방어: disabled set 에 없더라도 statusword 가 not-enabled 이거나 error_code 가
//       fault 인 ACTIVE actuator 의 명령은 마스킹된다 — CSP=현재위치 hold(사용자 목표로 구동 안 됨),
//       CST=0 토크. 고장난 모터를 사용자가 disable 하지 않은 경우의 SDK 최후 방어선.
//   (b) FK NaN 비전파: 도달불가/implausible encoder 로 FK 가 NaN 을 내도 그 NaN 이 joint state
//       (컨트롤러 seed 원천)로 새지 않는다. "homing 전 FK NaN 이 컨트롤러로 전파되는 문제" 회귀 가드.
//   (c) homing 중 고장: present/enabled 로 참여한 actuator 가 trigger 에서 fault(bit13)를 보고하면
//       조용히 homed 되지 않고 HomingError 로 실패 보고 + 해당 actuator 를 원인으로 표기.
// device-in-the-loop 아님. HandCore 는 생성만(소켓 open 안 함), HomingSequence 는 합성 StateFrames.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include <aidin_hand2/hand/hand_kinematics.hpp>  // 공개 FK (NaN 발생 확인용)
#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/config.hpp>

#include "hand_core/comms/canfd/protocol.hpp"
#include "hand_core/comms/state_machine/cia402.hpp"
#include "hand_core/comms/state_machine/homing_sequence.hpp"
#include "hand_core/hand_core.hpp"
#include "hand_core/kinematics/hand_kinematics_core.hpp"

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
  config.control_rate = 500;
  return config;
}

HandState healthy_state()
{
  return HandState{};
}

ActuatorHealth all_enabled_health()
{
  ActuatorHealth health{};
  health.enabled.fill(true);  // 기본은 전부 enabled·fault 없음
  return health;
}

// ── (a) fault-mask: disabled set 에 없는 faulted/미-enabled ACTIVE actuator 의 명령이 마스킹된다 ──
// ActuatorHealth 는 statusword(→enabled)·error_code(→fault)의 디코드 형태다. command_to_frames 는
// 이 health 를 직접 소비하므로, "statusword not-enabled" = enabled[i]=false, "error_code fault" =
// fault[i]!=None 로 재현한다.
void test_fault_mask_defends_active_actuator(HandCore& core)
{
  constexpr std::size_t kFaulted   = 3;  // error_code fault (enabled 여도)
  constexpr std::size_t kNotEnabled = 6; // statusword not-enabled
  constexpr std::size_t kHealthy   = 0;  // 대조군 — 정상 구동돼야 함
  constexpr std::int32_t kHoldA    = 1234;  // faulted 모터의 실제 위치
  constexpr std::int32_t kHoldB    = 777;   // not-enabled 모터의 실제 위치
  constexpr std::int32_t kTarget   = 5000;  // 사용자 명령 목표

  // CSP (ActuatorPositionCommand): faulted/not-enabled → 현재 위치 hold, 정상 → 목표.
  {
    HandState state = healthy_state();
    state.actuators.position_count[kFaulted]    = static_cast<double>(kHoldA);
    state.actuators.position_count[kNotEnabled] = static_cast<double>(kHoldB);
    ActuatorHealth health = all_enabled_health();
    health.fault[kFaulted]    = ActuatorFault::OverCurrentError;  // error_code fault, enabled 유지
    health.enabled[kNotEnabled] = false;                          // statusword not-enabled

    HandCommand command{};
    ActuatorPositionCommand actuator_position{};
    actuator_position.target.fill(static_cast<double>(kTarget));
    command.controller = actuator_position;
    command.max_effort_pct.fill(500.0);

    const auto frames = HandCoreTestPeer::command_to_frames(core, command, state, health);

    // 핵심 계약: faulted active actuator 는 사용자 목표로 구동되지 않는다.
    check(frames.target_position[kFaulted] != kTarget,
          "CSP: faulted actuator NOT driven with commanded value");
    check(frames.target_position[kFaulted] == kHoldA,
          "CSP: faulted actuator holds actual position");
    check(frames.target_position[kNotEnabled] != kTarget,
          "CSP: not-enabled actuator NOT driven with commanded value");
    check(frames.target_position[kNotEnabled] == kHoldB,
          "CSP: not-enabled actuator holds actual position");
    // per-actuator 마스킹임을 증명 — 정상 모터는 목표를 따른다.
    check(frames.target_position[kHealthy] == kTarget,
          "CSP: healthy actuator follows commanded value");
    // ActuatorHealth 가 fault 를 반영 (masking 의 key).
    check(health.fault[kFaulted] != ActuatorFault::None,
          "ActuatorHealth reflects the fault");
    check(!health.enabled[kNotEnabled],
          "ActuatorHealth reflects the not-enabled state");
  }

  // CST (ActuatorEffortCommand): faulted/not-enabled → 0 토크, 정상 → 목표.
  {
    HandState state = healthy_state();
    ActuatorHealth health = all_enabled_health();
    health.fault[kFaulted]      = ActuatorFault::HallSensorError;
    health.enabled[kNotEnabled] = false;

    HandCommand command{};
    ActuatorEffortCommand actuator_effort{};
    actuator_effort.target.fill(300.0);
    command.controller = actuator_effort;
    command.max_effort_pct.fill(500.0);

    const auto frames = HandCoreTestPeer::command_to_frames(core, command, state, health);

    check(frames.target_effort[kFaulted] == 0,   "CST: faulted actuator zero torque");
    check(frames.target_effort[kNotEnabled] == 0, "CST: not-enabled actuator zero torque");
    check(frames.target_effort[kHealthy] == 300,  "CST: healthy actuator follows commanded torque");
  }

  // JointImpedance (CST 경로): impedance 법칙이 0 이 아닌 토크를 낼 상황에서도 faulted → 0.
  {
    HandState state = healthy_state();
    state.actuators.position_count[kFaulted] = 2000.0;  // target 0 과 큰 오차 → K항 큰 토크 유발
    ActuatorHealth health = all_enabled_health();
    health.fault[kFaulted] = ActuatorFault::FollowingError;

    HandCommand command{};
    JointImpedanceCommand impedance{};
    impedance.target.fill(0.0);
    command.controller = impedance;
    command.max_effort_pct.fill(30000.0);

    ControllerConfig config{};
    config.joint_impedance_controller.stiffness.fill(1.0);
    config.joint_impedance_controller.damping.fill(0.0);

    const auto frames = HandCoreTestPeer::command_to_frames(core, command, state, health, config);
    check(frames.target_effort[kFaulted] == 0,
          "JointImpedance: faulted actuator masked to zero torque");
  }
}

// ── (b) bad/NaN encoder → FK NaN 이 joint state 로 전파되지 않는다 ──
void test_fk_nan_does_not_propagate()
{
  // 먼저 이 encoder 가 실제로 FK NaN 을 유발함을 공개 API 로 확인(가드가 방어할 위험이 실재).
  constexpr int kImplausibleEncoder = 1000000;  // 도달불가 — thumb/finger FK 가 non-finite 산출
  std::array<int, kActuatorCount> encoder{};
  encoder.fill(kImplausibleEncoder);
  const auto fk_joints = aidin_hand2::fk_actuator_to_joint(encoder);
  bool fk_has_nonfinite = false;
  for (double value : fk_joints) if (!std::isfinite(value)) fk_has_nonfinite = true;
  check(fk_has_nonfinite, "public FK produces non-finite for implausible encoder (hazard is real)");

  // HandCore::fill_joint_state 는 그 NaN 을 joint state 로 내보내지 않아야 한다(직전 유효값 유지).
  HandCore core(make_config());
  HandState state = healthy_state();
  for (std::size_t i = 0; i < kActuatorCount; ++i)
    state.actuators.position_count[i] = static_cast<double>(kImplausibleEncoder);
  HandCoreTestPeer::fill_joint_state(core, state);

  bool all_finite = true;
  for (std::size_t i = 0; i < kJointCount; ++i)
    if (!std::isfinite(state.joints.position_rad[i])) all_finite = false;
  check(all_finite, "fill_joint_state: FK NaN does NOT reach joint state");

  // 이어서 정상 encoder → 정상 각도, 그 다음 다시 NaN encoder → 직전 유효값 유지(0 튐 없음).
  {
    HandState good = healthy_state();
    good.actuators.position_count.fill(20000.0);  // 도달 가능 — 유한 각도
    HandCoreTestPeer::fill_joint_state(core, good);
    std::array<double, kJointCount> last_valid = good.joints.position_rad;
    bool good_finite = true;
    for (double v : last_valid) if (!std::isfinite(v)) good_finite = false;
    check(good_finite, "fill_joint_state: reachable encoder gives finite joints");

    HandState bad = healthy_state();
    bad.actuators.position_count.fill(static_cast<double>(kImplausibleEncoder));
    HandCoreTestPeer::fill_joint_state(core, bad);
    bool bad_finite = true;
    for (double v : bad.joints.position_rad) if (!std::isfinite(v)) bad_finite = false;
    check(bad_finite, "fill_joint_state: FK NaN after valid frame still finite (holds last valid)");
  }
}

// ── (c) homing 중 present-but-faulted actuator → HomingError, 조용히 homed 안 됨 ──

namespace status_word_bits = aidin_hand2::status_word;
namespace canfd = aidin_hand2::canfd;

// CiA402 상태 비트 조합(homing_sequence_test 와 동일 규약).
constexpr std::uint16_t kEnabled    = status_word_bits::kReadyToSwitchOn |
                                      status_word_bits::kSwitchedOn |
                                      status_word_bits::kOperationEnabled;   // 0x07
constexpr std::uint16_t kSwitchedOn = status_word_bits::kReadyToSwitchOn |
                                      status_word_bits::kSwitchedOn;         // 0x03
constexpr std::uint16_t kAttained   = status_word_bits::kHomingAttained;    // bit12
constexpr std::uint16_t kHomingErr  = status_word_bits::kHomingError;       // bit13

canfd::StateFrames make_rx(std::uint16_t status_word_value)
{
  canfd::StateFrames rx{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) rx.status_word[i] = status_word_value;
  return rx;
}

void test_homing_present_but_faulted_actuator()
{
  constexpr std::size_t kFaulty = 5;  // present/enabled 로 참여하지만 trigger 에서 fault 보고

  HomingSequence seq;
  seq.start();  // disabled 없음 — 전 actuator 참여 대상

  canfd::CommandFrames out{};
  canfd::StateFrames rx = make_rx(kEnabled);
  int trigger_cycle = 0;
  int guard = 0;
  while (seq.is_active() && guard++ < 12000) {
    const ControlWordMode mode = seq.step(rx, out);
    std::uint16_t sw = kEnabled;
    if (mode == ControlWordMode::SwitchOn) {
      sw = kSwitchedOn;  // ClearDisable: Operation Enabled 에서 빠져나오게
    } else if (mode == ControlWordMode::Homing) {
      ++trigger_cycle;
      if (trigger_cycle >= 3) sw |= kAttained;  // 정상 모터는 0→1 엣지로 완료
    } else {
      trigger_cycle = 0;
    }
    rx = make_rx(sw);
    // 고장 모터: enable/preload/clear 는 정상 참여(present 로 latch)하되, trigger 에서 fault(bit13) 보고.
    if (mode == ControlWordMode::Homing && trigger_cycle >= 3) {
      rx.status_word[kFaulty] = kEnabled | kHomingErr;  // present 였는데 homing error
    }
    for (std::size_t i = 0; i < kActuatorCount; ++i) rx.actual_position[i] = out.target_position[i];
  }

  check(!seq.is_active(), "homing fault: sequence terminated");
  check(!seq.succeeded(), "homing fault: NOT silently homed (succeeded == false)");
  check(seq.outcome() == HomingOutcome::HomingError, "homing fault: reports HomingError");
  check(seq.failed_actuators()[kFaulty],
        "homing fault: faulted actuator flagged as failure cause");
}

// 보완: enable 창 내내 fault 로 Operation Enabled 에 못 든 actuator 는 present 에서 제외(excluded)되고
// 나머지가 정상 homed → 죽은/고장 모터가 정상 모터를 막지 않는다.
void test_homing_never_enabled_actuator_excluded()
{
  constexpr std::size_t kDead = 8;
  HomingSequence seq;
  seq.start();

  canfd::CommandFrames out{};
  canfd::StateFrames rx = make_rx(kEnabled);
  rx.status_word[kDead] = status_word_bits::kFault;  // 계속 fault — Operation Enabled 못 감
  int trigger_cycle = 0;
  int guard = 0;
  while (seq.is_active() && guard++ < 12000) {
    const ControlWordMode mode = seq.step(rx, out);
    std::uint16_t live = kEnabled;
    if (mode == ControlWordMode::SwitchOn) {
      live = kSwitchedOn;
    } else if (mode == ControlWordMode::Homing) {
      ++trigger_cycle;
      if (trigger_cycle >= 3) live |= kAttained;
    } else {
      trigger_cycle = 0;
    }
    rx = make_rx(live);
    rx.status_word[kDead] = status_word_bits::kFault;  // 항상 fault (present 제외)
    for (std::size_t i = 0; i < kActuatorCount; ++i) rx.actual_position[i] = out.target_position[i];
  }

  check(!seq.is_active(), "homing exclude: sequence terminated");
  check(seq.succeeded() && seq.outcome() == HomingOutcome::Succeeded,
        "homing exclude: rest homes despite one never-enabled actuator");
  check(!seq.failed_actuators()[kDead],
        "homing exclude: never-enabled actuator excluded (not a failure cause)");
}

}  // namespace

int main()
{
  HandCore core(make_config());

  test_fault_mask_defends_active_actuator(core);
  test_fk_nan_does_not_propagate();
  test_homing_present_but_faulted_actuator();
  test_homing_never_enabled_actuator_excluded();

  std::printf("fault_robustness_test: %s\n", g_failures == 0 ? "PASS" : "FAIL");
  return g_failures;
}
