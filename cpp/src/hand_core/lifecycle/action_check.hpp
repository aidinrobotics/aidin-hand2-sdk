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
};

// Check the action is possible now
// The table is read against requested, so a requested stop refuses a command before the drives
// confirm it. observed only carries the fault, which dominates every request
[[nodiscard]] Status check_action_allowed(HandAction action, HandLifecycle requested,
                                          HandLifecycle observed, bool destroyed,
                                          const StopCause& stop_cause, bool homed);

// ---------------------------- Message rendering -----------------------------

// Builds ", last RX error: <reason>" from a canfd::DecodeResult and the CAN_ERR_* class bits
[[nodiscard]] std::string receive_anomaly_suffix(std::int32_t decode_result,
                                                 std::int32_t can_error_class);

// Builds " (errno <number>: <system message>)" from the errno of the failed CAN send
[[nodiscard]] std::string transmit_errno_suffix(std::int32_t transmit_errno);

// Builds "0x" and four uppercase hex digits
[[nodiscard]] std::string to_hex16(std::uint16_t value);

// Builds one phrase per stop reason, with the two suffixes above appended where they apply
[[nodiscard]] std::string stop_cause_phrase(const StopCause& cause);

}  // namespace aidin_hand2