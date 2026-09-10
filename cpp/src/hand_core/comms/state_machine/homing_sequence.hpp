#pragma once

#include <array>
#include <cstdint>

#include <aidin_hand2/types/description.hpp>

#include "hand_core/comms/canfd/protocol.hpp"
#include "hand_core/comms/state_machine/cia402.hpp"

namespace aidin_hand2
{

// Why the sequence ended, InProgress meaning it has not
enum class HomingOutcome {
  InProgress,

  // Every present actuator showed the bit12 rising edge with bit13 clear
  Succeeded,

  // The enable window closed without a single actuator reaching Operation Enabled
  EnableTimeout,

  // Re-entering the state machine or clearing bit12 ran out of cycles
  ClearTimeout,

  // A drive reported a homing error in status word bit13
  HomingError,

  // Some present actuator never attained after the trigger
  TriggerTimeout,
};

// Failure cause in public wording, keeping the CiA402 and stage names internal
[[nodiscard]] constexpr const char* homing_failure_reason(HomingOutcome outcome) noexcept
{
  switch (outcome) {
    case HomingOutcome::EnableTimeout:  return "no actuator could be brought under control";
    case HomingOutcome::ClearTimeout:   return "some actuators could not be prepared for homing";
    case HomingOutcome::HomingError:    return "some actuators reported a homing error";
    case HomingOutcome::TriggerTimeout: return "some actuators did not finish homing";
    case HomingOutcome::InProgress:
    case HomingOutcome::Succeeded:      break;
  }
  return "homing did not complete";
}

// The six internal stages grouped into the three phases the user is shown
enum class HomingPhase {
  // Enable through ClearReEnable
  Preparing,

  // Preload
  PreloadingToHardStop,

  // Settle and TriggerWait
  SettingOrigin,
};

// Preload and Settle are time based and cannot fail, so the outcome alone names the phase
[[nodiscard]] constexpr HomingPhase homing_failure_phase(HomingOutcome outcome) noexcept
{
  switch (outcome) {
    case HomingOutcome::EnableTimeout:
    case HomingOutcome::ClearTimeout:   return HomingPhase::Preparing;
    case HomingOutcome::TriggerTimeout:
    case HomingOutcome::HomingError:    return HomingPhase::SettingOrigin;
    case HomingOutcome::InProgress:
    case HomingOutcome::Succeeded:      break;
  }
  return HomingPhase::Preparing;
}

// Homing state machine, pure logic with no transport
// home() starts it and every RT cycle drives it through step()
//
// An actuator that never reaches Operation Enabled inside the enable window counts as absent,
// so a dead motor cannot block the connected ones
//
// Stages
//   Enable         Walk the control word ladder to Operation Enabled, then latch who is present
//   ClearDisable   Disable Operation, preparing to clear the homing attained bit
//   ClearReEnable  Enable Operation again and confirm bit12 is 0 on every actuator
//   Preload        Push to the hard stop in CST, which is where the true origin is
//   Settle         Hold mode 6 with 0x0F, so the next stage can raise bit4
//   TriggerWait    Raise bit4 to trigger the drive homing and wait for the bit12 rising edge
//
// Clearing bit12 runs before the push because 0x07 drops the power stage: doing it afterwards
// would let the pressed fingers slip and set the origin away from the hard stop
// The cost is that the mode changes twice while enabled, and Clear has to run in mode 6, since
// bit12 only means homing attained there
//
// step() fills out with this cycle's command and returns the control word mode for the caller
class HomingSequence {
 public:
  // Restarts from Enable and clears the outcome and the latches
  // disabled marks actuators to keep out of the enable window and the present latch
  void start(const std::array<bool, kActuatorCount>& disabled = {});

  // Releases a running sequence at once, which is how stop() interrupts homing
  void cancel();

  // Advances one cycle from the last decoded state, returning this cycle's control word mode
  ControlWordMode step(const canfd::StateFrames& rx, canfd::CommandFrames& out);

  [[nodiscard]] bool is_active() const noexcept { return active_; }
  [[nodiscard]] bool succeeded() const noexcept { return outcome_ == HomingOutcome::Succeeded; }
  [[nodiscard]] HomingOutcome outcome() const noexcept { return outcome_; }

  // Internal stage mapped to the phase the user is shown
  [[nodiscard]] HomingPhase current_phase() const noexcept;

  // Actuators that caused the failure, all false on success
  [[nodiscard]] const std::array<bool, kActuatorCount>& failed_actuators() const noexcept { return failed_; }

  // Stage the sequence failed in, or the last stage when it did not fail
  [[nodiscard]] const char* failed_stage_name() const noexcept;

  // Status words seen at that moment
  [[nodiscard]] const std::array<std::uint16_t, kActuatorCount>& failed_status_snapshot() const noexcept
  {
    return failed_status_snapshot_;
  }

  // Positions seen at that moment, for analysing a failure to reach the hard stop
  [[nodiscard]] const std::array<std::int32_t, kActuatorCount>& failed_position_snapshot() const noexcept
  {
    return failed_position_snapshot_;
  }

 private:
  enum class Stage {
    Enable, ClearDisable, ClearReEnable, Preload, Settle, TriggerWait,
  };

  // Moves to the next stage and resets the cycle counter
  void enter_stage(Stage next_stage);

  // Updates attained_, failed_ and outcome_ from the bit12 rising edge, a bit13 error or a timeout
  // Only failures are decided here, the success outcome belongs to the caller
  // Returns true once every present actuator has attained
  bool evaluate_homing_trigger(const canfd::StateFrames& rx);

  bool          active_{false};
  Stage         stage_{Stage::Enable};

  // Cycles spent in the current stage
  long          stage_cycle_{0};

  // Preload: cycles the velocity has stayed near zero
  long          stop_hold_cycle_{0};

  HomingOutcome outcome_{HomingOutcome::InProgress};

  // Latched by start(), and never part of the sequence
  std::array<bool, kActuatorCount>         disabled_{};

  // Reached Operation Enabled inside the enable window, so it takes part
  std::array<bool, kActuatorCount>         present_{};

  // Saw bit12 at 0 after the trigger, which is homing in progress
  std::array<bool, kActuatorCount>         in_progress_seen_{};

  // Saw the bit12 rising edge with bit13 clear
  std::array<bool, kActuatorCount>         attained_{};

  std::array<bool, kActuatorCount>         failed_{};

  // Recorded when the sequence fails
  Stage                                    failed_stage_{Stage::Enable};
  std::array<std::uint16_t, kActuatorCount> failed_status_snapshot_{};
  std::array<std::int32_t, kActuatorCount>  failed_position_snapshot_{};
};

}  // namespace aidin_hand2
