#include "hand_core/comms/state_machine/homing_sequence.hpp"

#include <cstddef>
#include <cstdint>

#include "hand_core/comms/state_machine/cia402.hpp"

namespace aidin_hand2
{

namespace
{

// Sequence parameters, confirmed on hardware
// Percent of rated current, 1000 being 100%
constexpr std::int16_t kPreloadEffort = -800;
constexpr std::int16_t kMaxEffort     = 1000;
// Preload ends on a fixed duration rather than a velocity test, which the small vibration
// at the settling point would make unrepeatable
constexpr int          kPreloadCycles    = 2000;
constexpr int          kSettleCycles     = 1000;

// Tail of Settle spent in mode 6, so the next stage can raise bit4
constexpr int          kEdgeCycles       = 1000;
// Timeouts, which only lengthen the wait on failure
constexpr int          kEnableTimeoutCycles  = 2000;  // 4s @500Hz
constexpr int          kClearTimeoutCycles   = 2000;  // 4s @500Hz
constexpr int          kTriggerTimeoutCycles = 2000;  // 4s @500Hz

using ActuatorMask = std::array<bool, kActuatorCount>;

bool any_present(const ActuatorMask& present)
{
  for (bool p : present) if (p) return true;
  return false;
}

// Absent actuators get effort 0, so nothing pushes a motor that is not there
// The control word is filled in afterwards by the caller
void fill_homing_command(canfd::CommandFrames& frames, const ActuatorMask& present,
                         std::int8_t mode_of_operation, std::int16_t target_effort)
{
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index) {
    frames.mode_of_operation[actuator_index] = mode_of_operation;
    frames.target_effort[actuator_index]     = present[actuator_index] ? target_effort : 0;
    frames.target_position[actuator_index]   = 0;
    frames.max_effort[actuator_index]        = kMaxEffort;
    frames.homing_method[actuator_index]     = 0;
  }
}


// Every all_ and no_ test below counts present actuators only
bool all_present_operation_enabled(const canfd::StateFrames& received_frames, const ActuatorMask& present)
{
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index) {
    if (!present[actuator_index]) continue;
    if (!(received_frames.status_word[actuator_index] & status_word::kOperationEnabled)) return false;
  }
  return true;
}

// Whether every present actuator has left Operation Enabled for Switched On
bool no_present_operation_enabled(const canfd::StateFrames& received_frames, const ActuatorMask& present)
{
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index) {
    if (!present[actuator_index]) continue;
    if (received_frames.status_word[actuator_index] & status_word::kOperationEnabled) return false;
  }
  return true;
}

// Whether bit12 has been cleared on every present actuator, giving a known 0 baseline
bool all_present_homing_attained_clear(const canfd::StateFrames& received_frames, const ActuatorMask& present)
{
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index) {
    if (!present[actuator_index]) continue;
    if (received_frames.status_word[actuator_index] & status_word::kHomingAttained) return false;
  }
  return true;
}

}  // namespace

void HomingSequence::start(const std::array<bool, kActuatorCount>& disabled)
{
  active_    = true;
  outcome_   = HomingOutcome::InProgress;
  disabled_  = disabled;
  present_.fill(false);
  in_progress_seen_.fill(false);
  attained_.fill(false);
  failed_.fill(false);
  enter_stage(Stage::Enable);
}

void HomingSequence::cancel()
{
  // outcome_ stays InProgress, so succeeded() reports it as unfinished
  active_ = false;
}

void HomingSequence::enter_stage(Stage next_stage)
{
  stage_           = next_stage;
  stage_cycle_     = 0;
  stop_hold_cycle_ = 0;
}

HomingPhase HomingSequence::current_phase() const noexcept
{
  switch (stage_) {
    case Stage::Enable:
    case Stage::ClearDisable:
    case Stage::ClearReEnable:  return HomingPhase::Preparing;
    case Stage::Preload:        return HomingPhase::PreloadingToHardStop;
    case Stage::Settle:
    case Stage::TriggerWait:    return HomingPhase::SettingOrigin;
  }
  return HomingPhase::Preparing;
}

const char* HomingSequence::failed_stage_name() const noexcept
{
  switch (failed_stage_) {
    case Stage::Enable:         return "Enable";
    case Stage::ClearDisable:   return "ClearDisable";
    case Stage::ClearReEnable:  return "ClearReEnable";
    case Stage::Preload:        return "Preload";
    case Stage::Settle:         return "Settle";
    case Stage::TriggerWait:    return "TriggerWait";
  }
  return "Unknown";
}

ControlWordMode HomingSequence::step(const canfd::StateFrames& rx, canfd::CommandFrames& out)
{
  ControlWordMode control_word_mode = ControlWordMode::Enable;
  ++stage_cycle_;
  // Kept to tell whether this cycle ended the sequence, and in which stage
  const bool was_active = active_;
  const Stage entry_stage = stage_;
  switch (stage_) {
    case Stage::Enable:
      // present is decided here: each cycle latches whoever reached Operation Enabled, so the
      // window only has to run out when an actuator never answers and must be left out
      {
        ActuatorMask enable_targets{};
        for (std::size_t i = 0; i < kActuatorCount; ++i) enable_targets[i] = !disabled_[i];
        fill_homing_command(out, enable_targets, canfd::mode_of_operation::kCST, 0);
      }
      {
        bool all_latched = true;
        for (std::size_t i = 0; i < kActuatorCount; ++i) {
          if (disabled_[i]) continue;
          if (rx.status_word[i] & status_word::kOperationEnabled) present_[i] = true;
          if (!present_[i]) all_latched = false;
        }
        // any_present also rules out an all-disabled mask, which must still time out
        if (all_latched && any_present(present_)) {
          enter_stage(Stage::ClearDisable);
        } else if (stage_cycle_ >= kEnableTimeoutCycles) {
          if (any_present(present_)) {
            enter_stage(Stage::ClearDisable);
          } else {
            // Nothing responded, so every actuator is reported as the cause
            failed_.fill(true);
            outcome_ = HomingOutcome::EnableTimeout;
            active_  = false;
          }
        }
      }
      break;

    case Stage::ClearDisable:
      // Runs before the push, because 0x07 drops the power stage and pressed fingers would slip
      fill_homing_command(out, present_, canfd::mode_of_operation::kHoming, 0);
      control_word_mode = ControlWordMode::SwitchOn;
      if (no_present_operation_enabled(rx, present_)) {
        enter_stage(Stage::ClearReEnable);
      } else if (stage_cycle_ >= kClearTimeoutCycles) {
        for (std::size_t i = 0; i < kActuatorCount; ++i)
          failed_[i] = present_[i] && (rx.status_word[i] & status_word::kOperationEnabled) != 0;
        outcome_ = HomingOutcome::ClearTimeout;
        active_  = false;
      }
      break;

    case Stage::ClearReEnable:
      // Re-entering Operation Enabled clears bit12
      // Only valid in mode 6, since that is where bit12 means homing attained
      fill_homing_command(out, present_, canfd::mode_of_operation::kHoming, 0);
      control_word_mode = ControlWordMode::Enable;
      if (all_present_operation_enabled(rx, present_) && all_present_homing_attained_clear(rx, present_)) {
        enter_stage(Stage::Preload);
      } else if (stage_cycle_ >= kClearTimeoutCycles) {
        for (std::size_t i = 0; i < kActuatorCount; ++i)
          failed_[i] = present_[i] && (!(rx.status_word[i] & status_word::kOperationEnabled) ||
                        (rx.status_word[i] & status_word::kHomingAttained));
        outcome_ = HomingOutcome::ClearTimeout;
        active_  = false;
      }
      break;

    case Stage::Preload:
      // Pushes to the hard stop in CST, which puts the physical zero in place
      fill_homing_command(out, present_, canfd::mode_of_operation::kCST, kPreloadEffort);
      if (stage_cycle_ >= kPreloadCycles) {
        enter_stage(Stage::Settle);
      }
      break;

    case Stage::Settle:
      // Holds the CST push so nothing slips off the hard stop, switching to mode 6 only at the end
      fill_homing_command(
        out, present_,
        (stage_cycle_ > kSettleCycles - kEdgeCycles) ? canfd::mode_of_operation::kHoming
                                                     : canfd::mode_of_operation::kCST,
        kPreloadEffort);
      control_word_mode = ControlWordMode::Enable;
      if (stage_cycle_ >= kSettleCycles) {
        enter_stage(Stage::TriggerWait);
      }
      break;

    case Stage::TriggerWait:
      // Completion means the drive origin is the hard stop, and kHomeOffsetCount backs off from it
      fill_homing_command(out, present_, canfd::mode_of_operation::kHoming, kPreloadEffort);
      control_word_mode = ControlWordMode::Homing;
      if (evaluate_homing_trigger(rx)) {
        outcome_ = HomingOutcome::Succeeded;
        active_  = false;
      }
      break;
  }
  // Snapshot the stage and the observed values when this cycle ends in a failure
  if (was_active && !active_ && outcome_ != HomingOutcome::Succeeded) {
    failed_stage_ = entry_stage;
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      failed_status_snapshot_[i]   = rx.status_word[i];
      failed_position_snapshot_[i] = rx.actual_position[i];
    }
  }
  return control_word_mode;
}

// Only updates state, the command and the mode change belong to the caller
bool HomingSequence::evaluate_homing_trigger(const canfd::StateFrames& rx)
{
  bool any_error = false;
  bool all_attained = true;
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (!present_[i]) continue;
    const std::uint16_t status = rx.status_word[i];
    if (status & status_word::kHomingError) {
      failed_[i] = true;
      any_error   = true;
    }
    if (!(status & status_word::kHomingAttained)) {
      // bit12 is 0, so homing is running
      in_progress_seen_[i] = true;
    } else if (in_progress_seen_[i] && !(status & status_word::kHomingError)) {
      // The rising edge completes it
      attained_[i] = true;
    }
    if (!attained_[i]) all_attained = false;
  }
  if (any_error) {
    outcome_ = HomingOutcome::HomingError;
    active_  = false;
    return false;
  }
  if (stage_cycle_ >= kTriggerTimeoutCycles && !all_attained) {
    for (std::size_t i = 0; i < kActuatorCount; ++i)
      if (present_[i] && !attained_[i]) failed_[i] = true;
    outcome_ = HomingOutcome::TriggerTimeout;
    active_  = false;
    return false;
  }
  return all_attained;
}

}  // namespace aidin_hand2
