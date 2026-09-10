// 목적: evaluate_stop_confirmation(순수 정지 판정 전이) 검증.
//   (1) 무토크 확인(confirmed) → Confirmed
//   (2) 지속 미확인 → 창 만료 후 NotConfirmed (무한 대기 안 함 — bounded)
//   (3) cycles 는 "Waiting & 미확인"일 때만 증가
//   (4) NotConfirmed 후 confirmed → Confirmed 로 승격 / Confirmed 는 미확인에도 유지
//   (5) 경계: window 에선 아직 Waiting, window+1 번째 미확인에서 NotConfirmed
// 방법: 순수 함수의 계약(반환값·cycles 증가)만 assert — 내부 구현 비의존.

#include <cstdio>

#include "types/stop_confirmation.hpp"

namespace
{
using aidin_hand2::StopConfirmation;
using aidin_hand2::evaluate_stop_confirmation;

int g_failures = 0;
void check(bool ok, const char* what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}

void test_confirmed()
{
  long cycles = 0;
  // 어느 상태에서든 confirmed 신호면 Confirmed. cycles 는 안 건드림.
  check(evaluate_stop_confirmation(StopConfirmation::Waiting, true, cycles, 10) ==
            StopConfirmation::Confirmed, "confirmed: Waiting → Confirmed");
  check(cycles == 0, "confirmed 는 cycles 증가 안 함");
  check(evaluate_stop_confirmation(StopConfirmation::NotConfirmed, true, cycles, 10) ==
            StopConfirmation::Confirmed, "confirmed: NotConfirmed → Confirmed (회복 승격)");
  check(evaluate_stop_confirmation(StopConfirmation::Confirmed, true, cycles, 10) ==
            StopConfirmation::Confirmed, "confirmed: Confirmed 유지");
}

void test_bounded_not_confirmed()
{
  const long window = 3;
  long cycles = 0;
  StopConfirmation s = StopConfirmation::Waiting;
  // window 번째까지는 Waiting (++cycles > window 아직 아님).
  for (long i = 1; i <= window; ++i) {
    s = evaluate_stop_confirmation(s, /*confirmed=*/false, cycles, window);
    check(s == StopConfirmation::Waiting, "미확인: 창 안에서는 Waiting 유지");
  }
  check(cycles == window, "cycles 가 window 까지 증가");
  // window+1 번째 미확인에서 NotConfirmed 로 전이 (무한 대기 안 함).
  s = evaluate_stop_confirmation(s, /*confirmed=*/false, cycles, window);
  check(s == StopConfirmation::NotConfirmed, "미확인: window+1 에서 NotConfirmed");
}

void test_cycles_only_when_waiting_and_unconfirmed()
{
  // 이미 NotConfirmed 면 미확인이어도 cycles 증가 안 하고 상태 유지.
  long cycles = 100;
  StopConfirmation s = evaluate_stop_confirmation(StopConfirmation::NotConfirmed, false, cycles, 3);
  check(s == StopConfirmation::NotConfirmed, "NotConfirmed: 미확인에 상태 유지");
  check(cycles == 100, "NotConfirmed: cycles 증가 안 함");
  // Confirmed 상태에서 미확인이 와도 유지(하강 전이 없음), cycles 불변.
  long cycles2 = 5;
  s = evaluate_stop_confirmation(StopConfirmation::Confirmed, false, cycles2, 3);
  check(s == StopConfirmation::Confirmed, "Confirmed: 미확인에도 유지");
  check(cycles2 == 5, "Confirmed: cycles 증가 안 함");
}

void test_boundary_window_zero_and_one()
{
  // window=0: 첫 미확인(++cycles=1 > 0) 즉시 NotConfirmed.
  long c0 = 0;
  check(evaluate_stop_confirmation(StopConfirmation::Waiting, false, c0, 0) ==
            StopConfirmation::NotConfirmed, "window=0: 첫 미확인에 NotConfirmed");
  // window=1: 1번째 Waiting, 2번째 NotConfirmed.
  long c1 = 0;
  check(evaluate_stop_confirmation(StopConfirmation::Waiting, false, c1, 1) ==
            StopConfirmation::Waiting, "window=1: 1번째 Waiting");
  check(evaluate_stop_confirmation(StopConfirmation::Waiting, false, c1, 1) ==
            StopConfirmation::NotConfirmed, "window=1: 2번째 NotConfirmed");
}

}  // namespace

int main()
{
  test_confirmed();
  test_bounded_not_confirmed();
  test_cycles_only_when_waiting_and_unconfirmed();
  test_boundary_window_zero_and_one();
  if (g_failures == 0) {
    std::printf("stop_confirmation_test: PASS\n");
  }
  return g_failures;
}
