// 목적: compute_control_word 의 CiA402 enable 사다리와 고정 command 모드가 status_word/직전
//       control_word 에 따라 규약대로 결정되는지 검증(Transport·HW 무관 순수 로직).
//       enable_settled 의 판정(run() 이 무엇을 기다리는지)도 같은 방식으로 검증한다.
// 방법: status_word 비트 조합 + 직전 control_word 를 넣어 반환 control_word 를 오라클과 대조.

#include <array>
#include <cstdint>
#include <cstdio>

#include "hand_core/comms/state_machine/cia402.hpp"

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

using aidin_hand2::compute_control_word;
using aidin_hand2::ControlWordMode;

constexpr std::uint16_t kReady   = aidin_hand2::status_word::kReadyToSwitchOn;   // bit0
constexpr std::uint16_t kSwitched = aidin_hand2::status_word::kSwitchedOn;       // bit1
constexpr std::uint16_t kEnabled = aidin_hand2::status_word::kOperationEnabled;  // bit2
constexpr std::uint16_t kFault   = aidin_hand2::status_word::kFault;             // bit3

constexpr std::uint16_t kShutdown = aidin_hand2::control_word::kShutdown;               // 0x06
constexpr std::uint16_t kSwitchOn = aidin_hand2::control_word::kSwitchOn;               // 0x07
constexpr std::uint16_t kEnable   = aidin_hand2::control_word::kEnableOperation;        // 0x0F
constexpr std::uint16_t kHoming   = aidin_hand2::control_word::kEnableOperationHoming;  // 0x1F
constexpr std::uint16_t kReset    = aidin_hand2::control_word::kFaultReset;             // 0x80
constexpr std::uint16_t kQuick    = aidin_hand2::control_word::kQuickStop;              // 0x02

// Enable 모드에서 직전 control_word 는 fault-reset 토글에만 쓰이므로 nofault 케이스는 0 로.
std::uint16_t enable_cw(std::uint16_t status_word, std::uint16_t last = 0)
{
  return compute_control_word(ControlWordMode::Enable, status_word, last);
}

}  // namespace

int main()
{
  // ── Enable 사다리: status_word 로 다음 단계 결정 ──
  check(enable_cw(0) == kShutdown,               "cold(status 0) → Shutdown");
  check(enable_cw(kReady) == kSwitchOn,          "ReadyToSwitchOn → SwitchOn");
  check(enable_cw(kReady | kSwitched) == kEnable, "SwitchedOn → EnableOperation");
  check(enable_cw(kReady | kSwitched | kEnabled) == kEnable, "OperationEnabled → EnableOperation");
  // SwitchedOn/OperationEnabled 는 ReadyToSwitchOn 보다 우선(사다리 상위 단계 유지).
  check(enable_cw(kSwitched) == kEnable,         "SwitchedOn(no ready bit) → EnableOperation");

  // ── Fault: bit7(0x80) 상승엣지 토글 (fault 는 다른 비트보다 우선) ──
  check(enable_cw(kFault, 0) == kReset,          "fault, last!=reset → FaultReset");
  check(enable_cw(kFault, kReset) == kShutdown,  "fault, last==reset → Shutdown (엣지 하강)");
  check(enable_cw(kFault | kReady | kEnabled, 0) == kReset, "fault 는 enable 비트보다 우선");

  // ── 고정 command 모드: status_word 무관 ──
  check(compute_control_word(ControlWordMode::Homing, 0, 0) == kHoming,          "Homing → 0x1F");
  check(compute_control_word(ControlWordMode::Homing, kFault, kReset) == kHoming, "Homing 은 fault 무시");
  check(compute_control_word(ControlWordMode::SwitchOn, kEnabled, 0) == kSwitchOn,   "SwitchOn → 0x07 강제");
  check(compute_control_word(ControlWordMode::Shutdown, kEnabled, 0) == kShutdown,    "Shutdown → 0x06");
  check(compute_control_word(ControlWordMode::QuickStop, kEnabled, 0) == kQuick,      "QuickStop → 0x02 강제");
  check(compute_control_word(ControlWordMode::QuickStop, kFault, kReset) == kQuick,   "QuickStop 은 fault 무시(정지 우선)");

  // ── enable_settled: run() 이 기다리는 조건 ──
  {
    using aidin_hand2::status_word::enable_settled;
    constexpr std::size_t kN = 4;
    constexpr std::uint16_t kOn = 0x27;   // Operation Enabled 비트 패턴
    constexpr std::uint16_t kOff = 0x21;  // 아직 Ready to Switch On

    std::array<std::uint16_t, kN> status{kOn, kOn, kOn, kOn};
    std::array<std::uint16_t, kN> error{0, 0, 0, 0};
    std::array<bool, kN> disabled{false, false, false, false};
    check(enable_settled(status, error, disabled), "전부 enable → 확인");

    status[2] = kOff;
    check(!enable_settled(status, error, disabled), "fault 없이 안 올라온 축 → 미확인(다시 run)");

    error[2] = 0x8611;  // FollowingError
    check(enable_settled(status, error, disabled), "fault 축은 건너뛰고 나머지로 확인");

    disabled[2] = true;
    error[2] = 0;
    check(enable_settled(status, error, disabled), "disabled 축은 원래 제외");

    disabled[2] = false;
    error = {0x8611, 0x8611, 0x8611, 0x8611};
    status = {kOff, kOff, kOff, kOff};
    check(!enable_settled(status, error, disabled), "전부 fault → 아무것도 안 올라옴, 미확인");

    disabled = {true, true, true, true};
    check(enable_settled(status, error, disabled), "present 축이 없으면 기다릴 것도 없음");
  }

  if (g_failures == 0) std::printf("control_word: all passed\n");
  return g_failures == 0 ? 0 : 1;
}