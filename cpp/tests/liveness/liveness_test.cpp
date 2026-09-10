// 목적: stop()/destroy() 및 connect→run→stop→disconnect→destroy 전체 lifecycle 이
//   통신이 "정지 확인(quick-stop-attained)"을 영원히 안 주는 상황에서도 유한 시간에 반환함을
//   HW 없이 증명한다 (liveness). 실행 seam 은 link-time mock transport:
//   mock_transport.cpp 가 실제 comms/canfd/transport.cpp 자리에 링크되어, RT 제어 thread 가
//   진짜 socket 없이 가짜 Transport 를 상대로 돈다.
//
// 핵심 계약: 정지 확인이 절대 안 와도 stop()·manager.destroy() 는 자체 판정-창(window)이
//   만료되며 unblock 한다 — hang 하지 않는다. 각 blocking 호출을 watchdog(별도 thread +
//   _Exit)로 감싸 유한 시간 내 반환을 강제 검증한다.
//
// 시나리오:
//   A. 전체 lifecycle: create→connect→run→stop→disconnect→destroy (frames, 정지확인 없음).
//      stop() 은 창 만료 후 반환(확인 실패라 Exception — 반환했음이 요점).
//   B. Running 중 destroy: create→connect→run→destroy — close() 의 정지-확인 대기가 창
//      만료로 unblock 함을 직접 겨냥.
//   C. 무수신(silence): connect() 가 자체 첫-수신 timeout 으로 유한 반환(Exception).
//   D. 미확인 정지의 재시도: 확인이 안 온 stop() 을 다시 불러도 "이미 Stopped" 로 조용히
//      성공하지 않는다 — lifecycle 이 사실을 담으므로 두 번째 호출도 재확인한다.
//
// 규약: 익명 namespace, check(bool,const char*)+g_failures, main 이 PASS 출력/ g_failures 반환.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <thread>

#include <aidin_hand2/hand/hand.hpp>
#include <aidin_hand2/hand/hand_manager.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/error.hpp>

#include "tests/liveness/mock_transport.hpp"

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

// Watchdog: run `call` on this thread, but a separate thread hard-kills the
// process (_Exit(1)) with a FAIL message if `call` has not returned within
// `budget`. A hang in stop()/destroy()/connect() therefore fails loudly and
// deterministically instead of stalling the test runner forever.
//
// Returns the wall-clock the call took (for reporting). Exceptions thrown by the
// public API (an error Status crossing the boundary) are caught and swallowed —
// for a liveness test only the RETURN matters, not the outcome. `threw` reports the
// outcome for the cases that assert on it.
template <class Callable>
double run_with_watchdog(const char* label, std::chrono::milliseconds budget, Callable&& call,
                         bool* threw = nullptr)
{
  std::atomic<bool> done{false};
  const auto start = std::chrono::steady_clock::now();

  std::thread watchdog([&done, label, budget] {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline) {
      if (done.load(std::memory_order_acquire)) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!done.load(std::memory_order_acquire)) {
      std::printf("FAIL: %s did not return within %lld ms — HUNG (liveness violated)\n",
                  label, static_cast<long long>(budget.count()));
      std::fflush(stdout);
      std::_Exit(1);
    }
  });

  if (threw != nullptr) *threw = false;
  try {
    call();
  } catch (const aidin_hand2::Exception& e) {
    // Expected for stop() (quick stop never confirmed) and connect() (silence):
    // the call RETURNED control by throwing — that is the liveness we assert.
    if (threw != nullptr) *threw = true;
    std::printf("  (%s returned via Exception: %s)\n", label, e.what());
  } catch (...) {
    if (threw != nullptr) *threw = true;
    std::printf("  (%s returned via unknown exception)\n", label);
  }

  done.store(true, std::memory_order_release);
  watchdog.join();

  const auto elapsed = std::chrono::steady_clock::now() - start;
  return std::chrono::duration<double, std::milli>(elapsed).count();
}

using namespace aidin_hand2;

// stop()/close() drive-confirmation windows are 200 ms and the outer waits are
// 500 ms; a 5 s watchdog is comfortably above any bounded return yet far below a
// true hang.
constexpr std::chrono::milliseconds kWatchdog{5000};

HandConfig make_config()
{
  HandConfig config{"mock0", HandSide::Left};
  config.control_rate = 500;
  config.auto_home = false;   // no homing — run() enables directly (per task)
  config.auto_reconnect = false;
  return config;
}

// A. Full lifecycle. Every blocking call is watchdog-wrapped; the whole point is
//    stop() and destroy() unblock even though quick stop is never attained.
void test_full_lifecycle_bounded()
{
  mock::set_mode(mock::Mode::FramesNoQuickStop);
  HandManager manager;
  Hand hand = manager.create(make_config());

  run_with_watchdog("connect()", kWatchdog, [&] { hand.connect(); });
  run_with_watchdog("run()", kWatchdog, [&] { hand.run(); });

  // stop(): frames flow but bit5 never drops, so quick_stop_attained() stays
  // false forever. The stop-confirmation window MUST expire and unblock.
  const double stop_ms = run_with_watchdog("stop()", kWatchdog, [&] { hand.stop(); });
  check(stop_ms < static_cast<double>(kWatchdog.count()),
        "stop() returns within bounded time (window expires, no hang)");

  run_with_watchdog("disconnect()", kWatchdog, [&] { hand.disconnect(); });

  const double destroy_ms =
      run_with_watchdog("destroy()", kWatchdog, [&] { manager.destroy(hand); });
  check(destroy_ms < static_cast<double>(kWatchdog.count()),
        "destroy() returns within bounded time");
}

// B. destroy() while Running — exercises close()'s own stop-confirmation wait
//    (not the already-stopped fast path). This is the sharpest destroy() test:
//    close() latches Stopped, waits for a confirmation that never comes, and must
//    give up on its bounded timeout.
void test_destroy_while_running_bounded()
{
  mock::set_mode(mock::Mode::FramesNoQuickStop);
  HandManager manager;
  Hand hand = manager.create(make_config());

  run_with_watchdog("connect() [B]", kWatchdog, [&] { hand.connect(); });
  run_with_watchdog("run() [B]", kWatchdog, [&] { hand.run(); });

  const double destroy_ms =
      run_with_watchdog("destroy() while Running", kWatchdog, [&] { manager.destroy(hand); });
  check(destroy_ms < static_cast<double>(kWatchdog.count()),
        "destroy() while Running returns within bounded time (stop window unblocks close)");
}

// C. Silence — open() succeeds but no frame ever arrives; connect() must give up
//    on its own first-frame timeout (bounded), not block indefinitely. Then
//    destroy() of the never-connected hand is also bounded.
void test_silence_connect_bounded()
{
  mock::set_mode(mock::Mode::Silence);
  HandManager manager;
  Hand hand = manager.create(make_config());

  const double connect_ms =
      run_with_watchdog("connect() [silence]", kWatchdog, [&] { hand.connect(); });
  check(connect_ms < static_cast<double>(kWatchdog.count()),
        "connect() returns within bounded time under silence (first-frame timeout)");

  const double destroy_ms =
      run_with_watchdog("destroy() [silence]", kWatchdog, [&] { manager.destroy(hand); });
  check(destroy_ms < static_cast<double>(kWatchdog.count()),
        "destroy() after failed connect returns within bounded time");
}

// D. 확인 없는 정지를 다시 요청했을 때. 예전 구현은 lifecycle 이 이미 Stopped 라는 이유로
//    두 번째 stop() 을 무검증 성공시켰다 — 문구는 "retry stop()" 인데 재확인이 없었다.
void test_unconfirmed_stop_retry_reverifies()
{
  mock::set_mode(mock::Mode::FramesNoQuickStop);
  HandManager manager;
  Hand hand = manager.create(make_config());

  run_with_watchdog("connect() [D]", kWatchdog, [&] { hand.connect(); });
  run_with_watchdog("run() [D]", kWatchdog, [&] { hand.run(); });

  bool first_threw = false;
  bool second_threw = false;
  run_with_watchdog("stop() [D]", kWatchdog, [&] { hand.stop(); }, &first_threw);
  run_with_watchdog("stop() retry [D]", kWatchdog, [&] { hand.stop(); }, &second_threw);
  check(first_threw, "확인 없는 stop() 은 예외로 알린다");
  check(second_threw, "재시도한 stop() 도 재확인한다 — 조용한 성공 금지");

  // lifecycle 은 사실만 담으므로, 확인되지 않은 정지는 Stopped 로 보이지 않는다
  check(hand.get_diagnostics().lifecycle != HandLifecycle::Stopped,
        "미확인 정지: lifecycle 은 Stopped 가 아니다");

  run_with_watchdog("destroy() [D]", kWatchdog, [&] { manager.destroy(hand); });
}

}  // namespace

int main()
{
  test_full_lifecycle_bounded();
  test_destroy_while_running_bounded();
  test_silence_connect_bounded();
  test_unconfirmed_stop_retry_reverifies();
  if (g_failures == 0) {
    std::printf("liveness_test: PASS\n");
  }
  return g_failures;
}
