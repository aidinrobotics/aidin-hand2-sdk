// 목적: HomingSequence 단계 전이(Enable→ClearDisable→ClearReEnable→Preload→Settle→TriggerWait)와
//       완료/실패 판정을 합성 StateFrames 로 검증(Transport·HW 무관 순수 로직).
// 방법: step() 이 반환한 control_word 모드로 "지금 어느 단계인지"를 읽어, 그 단계가 진행하도록
//       손을 흉내낸 rx(status_word/velocity)를 먹인다. 시간 상수·판정식은 homing_sequence.cpp 검증값을 pin.

#include <cstdint>
#include <cstdio>

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
using aidin_hand2::ControlWordMode;
using aidin_hand2::kActuatorCount;
namespace canfd = aidin_hand2::canfd;

// CiA402 state 비트. enabled=Ready|SwitchedOn|OperationEnabled, switched_on=Ready|SwitchedOn(bit2 없음).
constexpr std::uint16_t kEnabled    = aidin_hand2::status_word::kReadyToSwitchOn |
                                      aidin_hand2::status_word::kSwitchedOn |
                                      aidin_hand2::status_word::kOperationEnabled;   // 0x07
constexpr std::uint16_t kSwitchedOn = aidin_hand2::status_word::kReadyToSwitchOn |
                                      aidin_hand2::status_word::kSwitchedOn;         // 0x03
constexpr std::uint16_t kAttained   = aidin_hand2::status_word::kHomingAttained;    // bit12
constexpr std::uint16_t kHomingErr  = aidin_hand2::status_word::kHomingError;       // bit13

canfd::StateFrames make_rx(std::uint16_t status_word, std::int32_t velocity)
{
  canfd::StateFrames rx{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    rx.status_word[i]     = status_word;
    rx.actual_velocity[i] = velocity;
  }
  return rx;
}

// trigger 단계에서 먹일 status_word 를 trigger cycle 수로 정하는 정책.
enum class TriggerPolicy { Succeed, Timeout, Error };

// 시퀀스를 적응형 rx 로 구동. 반환한 모드로 현재 단계를 읽어 다음 rx 를 정한다.
// SwitchOn(=ClearDisable) → Switched On rx, Homing(=TriggerWait) → 정책별 bit12/13, 그 외 → enabled+정지.
void drive(HomingSequence& seq, TriggerPolicy policy)
{
  canfd::CommandFrames out{};
  canfd::StateFrames rx = make_rx(kEnabled, 0);
  int trigger_cycle = 0, guard = 0;
  // 시간 기반 Enable 창(2000)·Preload(2000)·Settle(1000) 을 모두 담도록 넉넉히.
  while (seq.is_active() && guard++ < 12000) {
    const ControlWordMode mode = seq.step(rx, out);
    if (mode == ControlWordMode::SwitchOn) {
      rx = make_rx(kSwitchedOn, 0);  // ClearDisable: Operation Enabled 에서 빠져나오게
    } else if (mode == ControlWordMode::Homing) {
      ++trigger_cycle;
      std::uint16_t sw = kEnabled;  // bit12=0(진행) 부터 시작
      if (policy == TriggerPolicy::Succeed && trigger_cycle >= 3) sw |= kAttained;   // 0→1 엣지
      if (policy == TriggerPolicy::Error   && trigger_cycle >= 3) sw |= kHomingErr;  // 진행 후 에러
      rx = make_rx(sw, 0);
    } else {
      rx = make_rx(kEnabled, 0);  // Enable/Preload/Settle/ClearReEnable: enabled + 정지 + bit12=0
    }
  }
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
  // ── start(): 개시 상태 ──
  {
    HomingSequence seq;
    check(!seq.is_active(), "개시 전 is_active false");
    seq.start();
    check(seq.is_active(), "start() → active");
    check(seq.outcome() == HomingOutcome::InProgress, "start() → InProgress");
    check(!seq.succeeded(), "start() → not succeeded");
  }

  // ── 정상 완료: 전 단계 통과 + bit12 0→1 엣지 → Succeeded ──
  {
    HomingSequence seq; seq.start();
    drive(seq, TriggerPolicy::Succeed);
    check(!seq.is_active(), "성공: 완료(is_active false)");
    check(seq.succeeded() && seq.outcome() == HomingOutcome::Succeeded, "성공: Succeeded");
    check(count_problem(seq) == 0, "성공: problem actuator 없음");
  }

  // ── 1-pass: Homing 트리거는 한 번만 관측된다(재영점 없음). 하드스톱에서 뗀 자리는 encoder
  //    경계의 home offset 이 담당하므로 시퀀스는 영점 한 번으로 끝난다. ──
  {
    HomingSequence seq;
    seq.start();
    canfd::CommandFrames out{};
    canfd::StateFrames rx = make_rx(kEnabled, 0);
    int trigger_cycle = 0, guard = 0, homing_entries = 0;
    bool was_homing = false;
    while (seq.is_active() && guard++ < 12000) {
      const ControlWordMode mode = seq.step(rx, out);
      const bool now_homing = (mode == ControlWordMode::Homing);
      if (now_homing && !was_homing) ++homing_entries;
      was_homing = now_homing;
      if (mode == ControlWordMode::SwitchOn) {
        rx = make_rx(kSwitchedOn, 0);
      } else if (now_homing) {
        ++trigger_cycle;
        std::uint16_t sw = kEnabled;
        if (trigger_cycle >= 3) sw |= kAttained;  // bit12 0→1 엣지
        rx = make_rx(sw, 0);
      } else {
        rx = make_rx(kEnabled, 0);
      }
    }
    check(!seq.is_active() && seq.succeeded(), "1-pass: 최종 Succeeded");
    check(homing_entries == 1, "1-pass: Homing 트리거가 한 번만 관측");
  }

  // ── Enable 타임아웃: 전 모터 계속 not-enabled(present 전무) → EnableTimeout, 전 actuator problem ──
  {
    HomingSequence seq; seq.start();
    canfd::CommandFrames out{};
    const canfd::StateFrames not_enabled = make_rx(0, 0);
    int guard = 0;
    while (seq.is_active() && guard++ < 2100) seq.step(not_enabled, out);
    check(!seq.is_active(), "enable timeout: 종료");
    check(seq.outcome() == HomingOutcome::EnableTimeout, "enable timeout: EnableTimeout");
    check(count_problem(seq) == (int)kActuatorCount, "enable timeout: 전 actuator problem");
  }

  // ── 결측 모터 자동 제외: 모터 4개(idx 7,8,9,10)는 끝까지 not-enabled, 나머지는 정상 →
  //    연결된 모터만으로 preload/homing 진행해 Succeeded, problem 없음 ──
  {
    HomingSequence seq; seq.start();
    // 죽은 모터: Enable 창에서 Operation Enabled 를 못 봄 → present 제외. 항상 status_word=0.
    const bool dead[kActuatorCount] = {false,false,false,false,false,false,false,
                                       true, true, true, true,
                                       false,false,false,false,false};
    canfd::CommandFrames out{};
    canfd::StateFrames rx = make_rx(kEnabled, 0);
    for (std::size_t i = 0; i < kActuatorCount; ++i) if (dead[i]) rx.status_word[i] = 0;
    int trigger_cycle = 0, guard = 0;
    while (seq.is_active() && guard++ < 12000) {
      const ControlWordMode mode = seq.step(rx, out);
      // 반환된 모드로 다음 rx 결정 (drive() 와 동일 정책). 죽은 모터는 언제나 status_word=0.
      std::uint16_t live = kEnabled;
      if (mode == ControlWordMode::SwitchOn) {
        live = kSwitchedOn;  // ClearDisable: Operation Enabled 에서 빠져나오게
      } else if (mode == ControlWordMode::Homing) {
        ++trigger_cycle;
        live = kEnabled;
        if (trigger_cycle >= 3) live |= kAttained;  // 살아있는 모터에 0→1 엣지
      }
      rx = make_rx(live, 0);
      for (std::size_t i = 0; i < kActuatorCount; ++i) if (dead[i]) rx.status_word[i] = 0;
    }
    check(!seq.is_active(), "결측 제외: 종료");
    check(seq.succeeded() && seq.outcome() == HomingOutcome::Succeeded, "결측 제외: 살아있는 모터로 Succeeded");
    check(count_problem(seq) == 0, "결측 제외: problem 없음(죽은 모터는 제외되어 원인 아님)");
  }

  // ── Trigger 타임아웃: bit12 계속 0(진행중만) → TriggerTimeout, 실패 ──
  {
    HomingSequence seq; seq.start();
    drive(seq, TriggerPolicy::Timeout);
    check(!seq.is_active(), "trigger timeout: 종료");
    check(!seq.succeeded() && seq.outcome() == HomingOutcome::TriggerTimeout, "trigger timeout: TriggerTimeout");
    check(count_problem(seq) == (int)kActuatorCount, "trigger timeout: 전 actuator problem(미attained)");
  }

  // ── Homing error: 트리거 후 bit13 → HomingError ──
  {
    HomingSequence seq; seq.start();
    drive(seq, TriggerPolicy::Error);
    check(!seq.is_active(), "homing error: 종료");
    check(!seq.succeeded() && seq.outcome() == HomingOutcome::HomingError, "homing error: HomingError");
    check(count_problem(seq) == (int)kActuatorCount, "homing error: 전 actuator problem");
  }

  if (g_failures == 0) std::printf("homing_sequence: all passed\n");
  return g_failures == 0 ? 0 : 1;
}
