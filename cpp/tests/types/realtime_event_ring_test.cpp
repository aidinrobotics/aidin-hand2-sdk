// 목적: RealtimeEventRing 의 (1) 단일스레드 의미론(FIFO·빈 링 pop·overflow drop+count_dropped·
//       wraparound 재사용), (2) 2-스레드 SPSC 무손실·순서 보존을 검증.
// 방법: 단일스레드는 결정적 시퀀스로 push/pop 반환·값을 직접 대조. 2-스레드는 producer 가
//       일련번호(detail)를 재시도-push 로 전부 밀어넣고 consumer 가 연속성(무손실·무중복)을 확인.

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "types/realtime_event_ring.hpp"

namespace
{

using aidin_hand2::RealtimeEvent;
using aidin_hand2::RealtimeEventKind;
using aidin_hand2::RealtimeEventRing;

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}

RealtimeEvent make_event(std::int32_t sequence)
{
  return {RealtimeEventKind::ActuatorFaultSet, static_cast<std::uint64_t>(sequence),
          sequence % 16, sequence};
}

void test_fifo_and_empty()
{
  RealtimeEventRing ring;
  RealtimeEvent out{};
  out.detail = -1;

  // 빈 링: pop=false, out 불변.
  check(!ring.pop(out), "빈 링 pop 은 false");
  check(out.detail == -1, "실패한 pop 은 out 을 안 건드림");

  // FIFO: push 5 → pop 5 가 같은 순서·같은 내용.
  for (std::int32_t i = 0; i < 5; ++i) check(ring.push(make_event(i)), "push 성공(여유 있음)");
  for (std::int32_t i = 0; i < 5; ++i) {
    check(ring.pop(out), "push 만큼 pop 성공");
    check(out.detail == i && out.cycle == static_cast<std::uint64_t>(i) &&
              out.actuator_index == i % 16 && out.kind == RealtimeEventKind::ActuatorFaultSet,
          "pop 이 push 순서·필드 그대로");
  }
  check(!ring.pop(out), "다 꺼낸 뒤 pop 은 false");
}

void test_overflow_drop()
{
  RealtimeEventRing ring;
  const std::size_t capacity = RealtimeEventRing::kCapacity;

  // 용량까지는 성공, 초과분은 drop + count_dropped 증가. producer 는 블로킹하지 않는다.
  for (std::size_t i = 0; i < capacity; ++i)
    check(ring.push(make_event(static_cast<std::int32_t>(i))), "용량 내 push 성공");
  for (int i = 0; i < 6; ++i) check(!ring.push(make_event(999)), "가득 찬 링 push 는 false");
  check(ring.count_dropped() == 6, "count_dropped = 버린 건수");

  // 버려진 사건은 흔적 없이 사라지고, 남은 것은 순서 보존.
  RealtimeEvent out{};
  for (std::size_t i = 0; i < capacity; ++i) {
    check(ring.pop(out), "가득 찼던 링에서 용량만큼 pop");
    check(out.detail == static_cast<std::int32_t>(i), "drop 이후에도 FIFO 순서 유지");
  }
  check(!ring.pop(out), "drop 된 사건은 pop 안 됨");

  // drain 후에는 다시 push 가능 (슬롯 재사용).
  check(ring.push(make_event(7)), "drain 후 push 재개");
  check(ring.pop(out) && out.detail == 7, "재개된 push 도 정상 pop");
}

void test_wraparound()
{
  RealtimeEventRing ring;
  RealtimeEvent out{};
  // 용량의 수십 배를 1-in-1-out 으로 돌려 인덱스 wrap(% kCapacity) 경계 검증.
  for (std::int32_t i = 0; i < 3000; ++i) {
    check(ring.push(make_event(i)), "wraparound push 성공");
    check(ring.pop(out) && out.detail == i, "wraparound pop 값 일치");
  }
  check(ring.count_dropped() == 0, "1-in-1-out 은 drop 없음");
}

void test_spsc_lossless_order()
{
  RealtimeEventRing ring;
  constexpr std::int32_t kTotal = 200000;
  std::atomic<bool> go{false};

  // producer: 가득 차면 재시도(실전의 RT 는 버리지만, 여기선 무손실 경로 자체를 검증).
  std::thread producer([&] {
    while (!go.load(std::memory_order_acquire)) {}
    for (std::int32_t i = 0; i < kTotal; ++i) {
      while (!ring.push(make_event(i))) {}
    }
  });

  std::int32_t received = 0;
  bool out_of_order = false;
  go.store(true, std::memory_order_release);
  RealtimeEvent out{};
  while (received < kTotal) {
    if (ring.pop(out)) {
      if (out.detail != received) out_of_order = true;  // 연속 일련번호 = 무손실·무중복·순서보존
      ++received;
    }
  }
  producer.join();

  check(!out_of_order, "2-스레드 pop 이 push 일련번호를 빠짐·중복 없이 순서대로 관측");
  check(!ring.pop(out), "전부 소비 후 링은 빈 상태");
  std::printf("  spsc: %d events transferred lossless in order\n", kTotal);
}

}  // namespace

int main()
{
  test_fifo_and_empty();
  test_overflow_drop();
  test_wraparound();
  test_spsc_lossless_order();
  if (g_failures == 0) {
    std::printf("realtime_event_ring_test: PASS\n");
  }
  return g_failures;
}
