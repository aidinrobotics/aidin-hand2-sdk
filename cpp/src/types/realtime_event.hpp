#pragma once

#include <cstdint>

#include <aidin_hand2/types/state.hpp>

namespace aidin_hand2
{

// Recorded once on the transition, not repeated while the condition holds
enum class RealtimeEventKind : std::int32_t {
  // Fault the hand, also stored as StopCause.reason
  ReceiveSilence,
  TransmitFailure,
  ControlLoopFailure,

  // Actuator and homing observations
  ActuatorFaultSet,
  ActuatorFaultCleared,
  HomingPhaseEntered,
  HomingCompleted,
  HomingFailed,

  // The cycle a tactile bias request took effect
  TactileBiasSet,
  TactileBiasReset,
};

// Fixed size POD with no strings, rendered into text by the event logging thread
struct RealtimeEvent {
  RealtimeEventKind kind{};
  std::uint64_t cycle{};

  // -1 for a whole-hand event
  std::int32_t actuator_index{-1};

  // Per kind: DecodeResult, errno, drive error code or HomingPhase
  std::int32_t detail{};

  // HomingFailed only, bit i is actuator i
  std::int32_t actuator_mask{};
};

// Written by the RT thread before it stores Faulted, so read it only after lifecycle reads Faulted
struct StopCause {
  RealtimeEventKind reason{};
  std::uint64_t cycle{};

  // Same meaning as in RealtimeEvent
  std::int32_t detail{};
};

}  // namespace aidin_hand2
