// 목적: RealtimeBuffer<T> 의 (1) 단일스레드 의미론(fresh 게이트·latest-wins·읽은 뒤 비움),
//       (2) 2-스레드 SPSC tearing 부재·단조성(loss 허용)을 검증.
// 방법: 단일스레드는 결정적 시퀀스로 read/write 반환·값을 직접 대조. 2-스레드는 payload 에
//       redundant 필드를 넣어 reader 가 읽은 값의 내부 일관성(=tearing 없음)과 비감소를 확인.

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "types/realtime_buffer.hpp"

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

// tearing 검출용 payload: 두 필드가 항상 같은 값으로 함께 쓰여야 한다.
struct Pair {
  std::uint64_t a;
  std::uint64_t b;
};

void test_single_thread()
{
  aidin_hand2::RealtimeBuffer<int> tb;
  int out = -1;

  // 초기: fresh 없음 → read=false, out 불변.
  check(!tb.read(out), "초기 read 는 false");
  check(out == -1, "실패한 read 는 out 을 안 건드림");

  // write 후 read 1회 성공.
  tb.write(42);
  check(tb.read(out) && out == 42, "write 후 read 가 그 값");
  // 같은 값을 두 번 소비 못 함(fresh 소진).
  check(!tb.read(out), "소비 후 read 는 false");

  // latest-wins: 연속 write 후 read 는 마지막 값(중간값 coalesce).
  tb.write(100);
  tb.write(200);
  tb.write(300);
  check(tb.read(out) && out == 300, "연속 write 후 read 는 최신값");
  check(!tb.read(out), "최신값 소비 후 read 는 false");
}

void test_spsc_no_tearing()
{
  aidin_hand2::RealtimeBuffer<Pair> tb;
  constexpr std::uint64_t kN = 2'000'000;
  std::atomic<bool> go{false};

  std::thread producer([&] {
    while (!go.load(std::memory_order_acquire)) {}
    for (std::uint64_t i = 1; i <= kN; ++i) {
      tb.write(Pair{i, i});  // 두 필드 동일값 — torn read 면 a!=b 로 드러남
    }
  });

  std::uint64_t reads = 0, last = 0;
  bool torn = false, regressed = false;
  go.store(true, std::memory_order_release);
  Pair p{};
  // producer 가 끝까지 도달할 때까지(마지막 값 관측 or 충분히 소진) 폴링.
  while (last < kN) {
    if (tb.read(p)) {
      ++reads;
      if (p.a != p.b) torn = true;          // 슬롯 분리 위반 = tearing
      if (p.a < last) regressed = true;      // latest-wins 면 비감소
      last = p.a;
    }
  }
  producer.join();

  check(!torn, "2-스레드 read 에 tearing 없음 (a==b 항상)");
  check(!regressed, "관측값이 단조 비감소 (latest-wins)");
  check(reads > 0, "reader 가 최소 1회는 값 관측");
  std::printf("  spsc: %llu writes, %llu reads observed (loss 허용)\n",
              (unsigned long long)kN, (unsigned long long)reads);
}

}  // namespace

int main()
{
  test_single_thread();
  test_spsc_no_tearing();
  if (g_failures == 0) {
    std::printf("realtime_buffer_test: PASS\n");
  }
  return g_failures;
}
