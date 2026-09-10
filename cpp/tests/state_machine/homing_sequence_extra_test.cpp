// 목적: HomingSequence 중 기존 homing_sequence_test 가 다루지 않는 경로를 합성 StateFrames 로 검증.
//   1) ClearTimeout — fault-clear(ClearDisable) 단계가 Operation Enabled 에서 못 빠져나와 타임아웃.
//   2) start(disabled) — 명시적 비활성 mask 로 준 actuator 를 참여 대상에서 배제(암묵 auto-exclude 와 구분).
//   3) cancel() — 진행 중 취소 시 문서화된 종료 상태(InProgress 유지, is_active/succeeded false).
//   4) 실패 후 진단 accessor(current_phase / failed_stage_name / failed_*_snapshot) 가 타당한 값.
// 방법: step() 이 반환한 control_word 모드로 "지금 어느 단계인지"를 읽어 다음 rx(status_word/velocity)를
//   손처럼 먹인다. 기존 test 의 합성 StateFrame 구동 패턴(make_rx + 모드 적응형 rx)을 재사용.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <aidin_hand2/types/description.hpp>  // kActuatorCount

#include "hand_core/comms/canfd/protocol.hpp"
#include "hand_core/comms/state_machine/cia402.hpp"
#include "hand_core/comms/state_machine/homing_sequence.hpp"

namespace
{

int g_failures = 0;
void check(bool ok, const char* what)
{
  if (!ok) { std::printf("FAIL: %s\n", what); ++g_failures; }
}

using aidin_hand2::HomingSequence;
using aidin_hand2::HomingOutcome;
using aidin_hand2::HomingPhase;
using aidin_hand2::ControlWordMode;
using aidin_hand2::kActuatorCount;
namespace canfd = aidin_hand2::canfd;

// CiA402 state 비트 (기존 test 와 동일 pin).
constexpr std::uint16_t kEnabled    = aidin_hand2::status_word::kReadyToSwitchOn |
                                      aidin_hand2::status_word::kSwitchedOn |
                                      aidin_hand2::status_word::kOperationEnabled;   // 0x07
constexpr std::uint16_t kSwitchedOn = aidin_hand2::status_word::kReadyToSwitchOn |
                                      aidin_hand2::status_word::kSwitchedOn;         // 0x03
constexpr std::uint16_t kAttained   = aidin_hand2::status_word::kHomingAttained;    // bit12

canfd::StateFrames make_rx(std::uint16_t status_word, std::int32_t velocity)
{
  canfd::StateFrames rx{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    rx.status_word[i]     = status_word;
    rx.actual_velocity[i] = velocity;
  }
  return rx;
}

// 모드에 따른 "살아있는(정상)" 모터의 다음 status_word — 기존 test 의 성공 정책과 동일.
// SwitchOn(=ClearDisable) → Switched On(Operation Enabled 빠져나옴), Homing(=TriggerWait) → 3 cycle 뒤 attained.
std::uint16_t live_status_for(ControlWordMode mode, int& trigger_cycle)
{
  if (mode == ControlWordMode::SwitchOn) return kSwitchedOn;
  if (mode == ControlWordMode::Homing) {
    ++trigger_cycle;
    return (trigger_cycle >= 3) ? (kEnabled | kAttained) : kEnabled;
  }
  trigger_cycle = 0;  // 트리거 phase 사이 리셋 (다음 phase 의 0→1 엣지 재현)
  return kEnabled;
}

int count_problem(const HomingSequence& seq)
{
  int n = 0;
  for (std::size_t i = 0; i < kActuatorCount; ++i) n += seq.failed_actuators()[i] ? 1 : 0;
  return n;
}

}  // namespace

int main()
{
  // ── (1) ClearTimeout: ClearDisable 에서 present 모터가 Operation Enabled 를 계속 유지 →
  //    no_present_operation_enabled 가 영원히 false → kClearTimeoutCycles 후 ClearTimeout. ──
  //    (4) 그 실패 지점 진단 accessor 도 함께 검증.
  {
    HomingSequence seq; seq.start();
    canfd::CommandFrames out{};
    // Enable/Preload 는 계속 enabled 로 통과시키고, ClearDisable(SwitchOn 모드)에 들어서면
    // 일부러 enabled 를 유지해 fault-clear 가 못 빠져나오게 한다.
    canfd::StateFrames rx = make_rx(kEnabled, 0);
    bool reached_cleardisable = false;
    int guard = 0;
    while (seq.is_active() && guard++ < 12000) {
      const ControlWordMode mode = seq.step(rx, out);
      if (mode == ControlWordMode::SwitchOn) reached_cleardisable = true;
      // 어느 단계든 항상 Operation Enabled 를 유지 → ClearDisable 이 절대 못 빠져나감.
      rx = make_rx(kEnabled, 0);
    }
    check(reached_cleardisable, "ClearTimeout: ClearDisable 단계까지 도달");
    check(!seq.is_active(), "ClearTimeout: 종료");
    check(!seq.succeeded() && seq.outcome() == HomingOutcome::ClearTimeout, "ClearTimeout: ClearTimeout outcome");
    // ClearDisable 에서 안 내려간(Operation Enabled 유지) present 모터가 원인 — 전 actuator.
    check(count_problem(seq) == (int)kActuatorCount, "ClearTimeout: 전 present actuator problem");

    // (4) 진단 accessor.
    check(seq.current_phase() == HomingPhase::Preparing, "진단: ClearTimeout 시 current_phase=Preparing");
    check(std::strcmp(seq.failed_stage_name(), "ClearDisable") == 0,
          "진단: failed_stage_name=ClearDisable");
    check((seq.failed_status_snapshot()[0] & aidin_hand2::status_word::kOperationEnabled) != 0,
          "진단: failed_status_snapshot 이 Operation Enabled 관측 보존");
  }

  // ── (2) 명시적 disabled mask: idx 2,5 를 비활성으로 준다. 이 둘은 항상 kEnabled 를 보고해도
  //    (즉 auto-exclude 로는 present 가 될 자격이 있어도) 명시 mask 로 참여 대상에서 배제된다. ──
  //    present_ 는 public accessor 가 없어, "disabled 모터에는 Preload push effort 가 절대 안 나간다"
  //    (fill_homing_command 이 present_ 아닌 슬롯에 effort 0)로 배제를 간접 검증한다.
  {
    std::array<bool, kActuatorCount> disabled{};
    disabled[2] = true;
    disabled[5] = true;

    HomingSequence seq; seq.start(disabled);
    canfd::CommandFrames out{};
    canfd::StateFrames rx = make_rx(kEnabled, 0);
    int trigger_cycle = 0, guard = 0;
    bool disabled_ever_got_effort = false;  // 배제됐다면 false 여야 함
    bool live_saw_preload_effort  = false;  // 정상 모터는 Preload push effort 관측
    while (seq.is_active() && guard++ < 12000) {
      const ControlWordMode mode = seq.step(rx, out);
      if (out.target_effort[2] != 0 || out.target_effort[5] != 0) disabled_ever_got_effort = true;
      if (out.target_effort[0] != 0) live_saw_preload_effort = true;  // idx0 = 정상 모터
      // 다음 rx: 정상 모터는 성공 정책, disabled 모터는 "항상 kEnabled"(auto-exclude 였다면 present 될 자격).
      const std::uint16_t live = live_status_for(mode, trigger_cycle);
      rx = make_rx(live, 0);
      rx.status_word[2] = kEnabled;
      rx.status_word[5] = kEnabled;
      for (std::size_t i = 0; i < kActuatorCount; ++i) rx.actual_position[i] = out.target_position[i];
    }
    check(!seq.is_active() && seq.succeeded() && seq.outcome() == HomingOutcome::Succeeded,
          "disabled mask: 나머지 모터로 Succeeded");
    check(!disabled_ever_got_effort, "disabled mask: 비활성 모터에는 effort 명령이 절대 안 나감(배제됨)");
    check(live_saw_preload_effort, "disabled mask: 정상 모터는 Preload push effort 관측(sanity)");
    check(count_problem(seq) == 0, "disabled mask: 성공이라 problem 없음");
  }

  // ── (3) cancel(): 진행 중 취소 → 즉시 비활성, outcome 은 InProgress 유지(succeeded false). ──
  {
    HomingSequence seq; seq.start();
    canfd::CommandFrames out{};
    const canfd::StateFrames rx = make_rx(kEnabled, 0);
    for (int i = 0; i < 10; ++i) seq.step(rx, out);  // 몇 cycle 진행 (Enable 창 안)
    check(seq.is_active(), "cancel: 취소 전 active");
    seq.cancel();
    check(!seq.is_active(), "cancel: 취소 후 비활성");
    check(!seq.succeeded(), "cancel: succeeded false");
    check(seq.outcome() == HomingOutcome::InProgress, "cancel: outcome 은 InProgress 유지(완료 아님)");
    check(count_problem(seq) == 0, "cancel: 실패 원인 actuator 없음");
  }

  if (g_failures == 0) std::printf("homing_sequence_extra_test: PASS\n");
  return g_failures;
}
