#pragma once

#include <cstdint>

#include <aidin_hand2/types/state.hpp>

namespace aidin_hand2
{

// Recorded once on the transition, not repeated while the condition holds
enum class RealtimeEventKind : std::int32_t {
  // Trigger an automatic stop, also stored as StopCause.reason
  ReceiveSilence,
  TransmitFailure,
  ControlLoopFailure,

  // Outcome of that stop
  StopConfirmed,
  StopNotConfirmed,
  ReceiveRestored,

  // Actuator and homing observations
  ActuatorFaultSet,
  ActuatorFaultCleared,
  HomingPhaseEntered,
  HomingCompleted,
  HomingFailed,
};

// Fixed size POD with no strings, rendered into text by the event logging thread
struct RealtimeEvent {
  RealtimeEventKind kind{};
  std::uint64_t cycle{};

  // -1 for a whole-hand event
  std::int32_t actuator_index{-1};

  // Per kind: DecodeResult, errno, drive error code or HomingPhase
  std::int32_t detail{};

  // ReceiveSilence only, CAN_ERR_* class bits of the last error frame
  std::int32_t can_error_class{};

  // Snapshot taken at push time
  HandLifecycle lifecycle{HandLifecycle::Disconnected};
  bool stop_latched{false};

  // HomingFailed only, bit i is actuator i
  std::int32_t actuator_mask{};
};

// Written by the RT thread before it stores Faulted, so read it only after lifecycle reads Faulted
struct StopCause {
  RealtimeEventKind reason{};
  std::uint64_t cycle{};

  // Same meaning as in RealtimeEvent
  std::int32_t detail{};
  std::int32_t can_error_class{};
};

}  // namespace aidin_hand2
