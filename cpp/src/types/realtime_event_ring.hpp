#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "types/realtime_event.hpp"

namespace aidin_hand2
{

// Lock-free fixed capacity queue carrying every event in order, RT loop to logging thread
// The producer never waits, a full ring drops the event and counts it
class RealtimeEventRing
{
 public:
  static constexpr std::size_t kCapacity = 64;

  RealtimeEventRing() = default;
  RealtimeEventRing(const RealtimeEventRing&) = delete;
  RealtimeEventRing& operator=(const RealtimeEventRing&) = delete;

  // Returns false when the ring is full
  bool push(const RealtimeEvent& event) noexcept
  {
    const std::uint64_t write = write_index_.load(std::memory_order_relaxed);
    const std::uint64_t read  = read_index_.load(std::memory_order_acquire);
    if (write - read >= kCapacity) {
      count_dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    slots_[write % kCapacity] = event;

    // release publishes the payload before the index
    write_index_.store(write + 1, std::memory_order_release);
    return true;
  }

  // Fills out and returns true only when an event is waiting
  bool pop(RealtimeEvent& out) noexcept
  {
    const std::uint64_t read  = read_index_.load(std::memory_order_relaxed);
    const std::uint64_t write = write_index_.load(std::memory_order_acquire);
    if (read == write) return false;
    out = slots_[read % kCapacity];

    // release tells the producer the slot is consumed
    read_index_.store(read + 1, std::memory_order_release);
    return true;
  }

  [[nodiscard]] std::uint64_t count_dropped() const noexcept
  {
    return count_dropped_.load(std::memory_order_relaxed);
  }

 private:
  // Indices grow monotonically, so full and empty differ by write - read alone
  std::array<RealtimeEvent, kCapacity> slots_{};
  std::atomic<std::uint64_t> write_index_{0};
  std::atomic<std::uint64_t> read_index_{0};
  std::atomic<std::uint64_t> count_dropped_{0};
};

}  // namespace aidin_hand2
