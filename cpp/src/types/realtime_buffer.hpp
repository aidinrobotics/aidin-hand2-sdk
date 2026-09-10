#pragma once

#include <atomic>
#include <type_traits>

namespace aidin_hand2
{

// Lock-free handoff of the latest value between one producer and one consumer
// The index word orders both directions, so the writer never reuses a slot the reader still holds
template <class T>
class RealtimeBuffer
{
  static_assert(std::is_trivially_copyable_v<T>, "RealtimeBuffer<T>: T must be trivially copyable POD");

 public:
  RealtimeBuffer() : state_(pack(0, 1, 2, false)) {}
  RealtimeBuffer(const RealtimeBuffer&) = delete;
  RealtimeBuffer& operator=(const RealtimeBuffer&) = delete;

  void write(const T& v)
  {
    // acquire pairs with the reader's release, so the slot is free to overwrite
    const unsigned s = state_.load(std::memory_order_acquire);
    slot_[wr(s)] = v;
    unsigned cur = s, next;
    do {
      next = pack(rdy(cur), wr(cur), rd(cur), true);
    } while (!state_.compare_exchange_weak(
        // release publishes the payload before the index
        cur, next, std::memory_order_release,
        std::memory_order_relaxed));
  }

  // Fills out and returns true only when a new value is waiting
  bool read(T& out)
  {
    unsigned cur = state_.load(std::memory_order_acquire);
    if (!fresh(cur)) return false;
    unsigned next;
    do {
      next = pack(wr(cur), rd(cur), rdy(cur), false);
    } while (!state_.compare_exchange_weak(
        // release tells the writer the slot is consumed
        cur, next, std::memory_order_acq_rel,
        std::memory_order_relaxed));
    out = slot_[rd(next)];
    return true;
  }

 private:
  // state word: [1:0] write, [3:2] ready, [5:4] read, [6] fresh
  static unsigned pack(unsigned w, unsigned r, unsigned c, bool f)
  {
    return (w & 3u) | ((r & 3u) << 2) | ((c & 3u) << 4) | (f ? (1u << 6) : 0u);
  }
  static unsigned wr(unsigned s) { return s & 3u; }
  static unsigned rdy(unsigned s) { return (s >> 2) & 3u; }
  static unsigned rd(unsigned s) { return (s >> 4) & 3u; }
  static bool fresh(unsigned s) { return (s >> 6) & 1u; }

  T slot_[3];
  std::atomic<unsigned> state_;
};

}  // namespace aidin_hand2
