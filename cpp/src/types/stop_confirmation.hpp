#pragma once

namespace aidin_hand2
{

// NotConfirmed means the window expired without every drive reaching quick stop
enum class StopConfirmation {
  Waiting,
  Confirmed,
  NotConfirmed,
};

// Pure transition, called every cycle while the stop latch is up
// cycles advances only on an unconfirmed Waiting cycle
[[nodiscard]] inline StopConfirmation evaluate_stop_confirmation(StopConfirmation current,
                                                                bool confirmed_signal,
                                                                long& cycles, long window) noexcept
{
  if (confirmed_signal) return StopConfirmation::Confirmed;
  if (current == StopConfirmation::Waiting && ++cycles > window) {
    return StopConfirmation::NotConfirmed;
  }
  return current;
}

}  // namespace aidin_hand2
