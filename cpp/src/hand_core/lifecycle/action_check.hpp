#pragma once

#include <cstdint>
#include <string>

#include <aidin_hand2/types/error.hpp>

#include <aidin_hand2/types/state.hpp>
#include "types/realtime_event.hpp"
#include "types/status.hpp"

namespace aidin_hand2
{

// ---------------------------------- Action ----------------------------------

// One row of the rule table in action_check.cpp
enum class HandAction {
  Connect,
  Disconnect,
  Run,
  Stop,
  Home,
  Reconnect,
  SetCommand,
  SetTactileBias,
  ResetTactileBias,
};

// E207, one message whichever call met the destroyed hand, as the caller knows which call it made
[[nodiscard]] Status hand_destroyed();

// Check the action is possible now
// The table is read against requested, so a requested stop refuses a command before the drives
// confirm it. observed only carries the fault, which dominates every request
[[nodiscard]] Status check_action_allowed(HandAction action, HandLifecycle requested,
                                          HandLifecycle observed, bool destroyed,
                                          const StopCause& stop_cause, bool homed);

// ---------------------------- Message rendering -----------------------------

// Builds ", last RX error: conflicting command" from a canfd::DecodeResult, empty for the rest
// The other anomalies do not tell why the frames stopped
[[nodiscard]] std::string receive_anomaly_suffix(std::int32_t decode_result);

// Builds " (errno <number>: <system message>)" from the errno of the failed CAN send
[[nodiscard]] std::string transmit_errno_suffix(std::int32_t transmit_errno);

// Builds "0x" and four uppercase hex digits
[[nodiscard]] std::string to_hex16(std::uint16_t value);

// The number its realtime log line carries: 404, 405 or 702, 0 for a reason that never stops the hand
[[nodiscard]] std::uint16_t stop_cause_code(const StopCause& cause);

// Builds "E<code> <event>", the code and the words of its realtime log line, empty for code 0
[[nodiscard]] std::string stop_cause_phrase(const StopCause& cause);

}  // namespace aidin_hand2