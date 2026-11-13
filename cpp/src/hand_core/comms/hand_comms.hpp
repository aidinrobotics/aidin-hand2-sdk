#pragma once

#include <array>
#include <cstdint>

#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>

#include "hand_core/comms/canfd/protocol.hpp"
#include "hand_core/comms/canfd/transport.hpp"
#include "hand_core/comms/state_machine/homing_sequence.hpp"
#include "types/status.hpp"

namespace aidin_hand2
{

// Owns the Transport that moves frames and the Protocol that gives them meaning
// It activates the communication, while HandCore owns the typed commands and the loop
class HandComms {
 public:
  // -------------------------------- Lifetime --------------------------------

  explicit HandComms(HandConfig config);
  ~HandComms();

  // Transport cannot move and HandCore holds this by value, so neither copy nor move is allowed
  HandComms(const HandComms&) = delete;
  HandComms& operator=(const HandComms&) = delete;
  HandComms(HandComms&&) = delete;
  HandComms& operator=(HandComms&&) = delete;

  // ----------------------------- Communication ------------------------------

  // Makes frames flow, without enabling any drive: opens the socket, installs the filters, checks
  // that no other master commands the hand and sets the TX headers, rolling back a partial open
  // An interface_name of "auto" scans the CAN interfaces for the one this side is wired to
  [[nodiscard]] Status activate();

  // Safe to call repeatedly
  void deactivate() noexcept;

  // The interface actually activated, which after an "auto" scan is the resolved canX
  [[nodiscard]] const std::string& interface_name() const noexcept { return config_.interface_name; }

  // Actuators in config_.disabled_actuators, each counted once
  [[nodiscard]] std::size_t disabled_actuator_count() const noexcept;

  // -------------------------------- RT path ---------------------------------

  // Called every cycle, so these return false instead of a Status: building a message allocates
  // Turning a failure into a sentence is the non-RT side's job

  // Starts the homing sequence, which write() then drives, false while not active
  [[nodiscard]] bool start_homing();

  // Drains RX, decodes it and fills the observation, false while not active
  [[nodiscard]] bool read(HandState& hand_state, ActuatorHealth& actuator_health);

  // The control word comes from the internal state machine
  // quick_stop forces 0x02, which stops the drives themselves and cancels a running homing
  // False while not active or when a frame was not sent, with last_transmit_errno() telling why
  [[nodiscard]] bool write(const canfd::CommandFrames& command_frames, bool quick_stop = false);

  // Whether every enabled actuator reached quick stop, which is what stop() waits on
  [[nodiscard]] bool quick_stop_attained() const noexcept;

  // Whether the enable settled: every fault-free actuator reached Operation Enabled and at least
  // one did. A faulted actuator is left to the fault reset ladder instead of being waited for
  [[nodiscard]] bool drives_enabled() const noexcept;

  // read() calls since the last frame arrived, which is how a lost link is spotted
  [[nodiscard]] std::uint32_t cycles_since_last_frame() const noexcept { return cycles_since_last_frame_; }

  // Whether this session ever saw a valid frame, so silence is never reported before the first one
  [[nodiscard]] bool any_frame_received() const noexcept { return any_frame_received_; }

  // ConflictingCommand once another master's command arrived in this session, otherwise Decoded
  [[nodiscard]] canfd::DecodeResult last_receive_anomaly() const noexcept { return last_receive_anomaly_; }

  // errno of the last failed TX, 0 when the reason is unknown
  [[nodiscard]] int last_transmit_errno() const noexcept { return last_transmit_errno_; }

  // --------------------------------- Homing ---------------------------------

  // start_homing() starts the sequence, write() drives it, and is_homing() drops to false when it ends
  [[nodiscard]] bool is_homing() const noexcept;

  // Why it ended, and how far it had got
  [[nodiscard]] HomingOutcome last_homing_outcome() const noexcept;
  [[nodiscard]] HomingPhase homing_phase() const noexcept;

  // Actuators that caused the last failure, all false on success
  [[nodiscard]] const std::array<bool, kActuatorCount>& last_homing_failed_actuators() const noexcept;

  // Stage it stopped in and the values seen there, for the debug log
  [[nodiscard]] const char* last_homing_failure_stage() const noexcept;
  [[nodiscard]] const std::array<std::uint16_t, kActuatorCount>& last_homing_status_snapshot() const noexcept;
  [[nodiscard]] const std::array<std::int32_t, kActuatorCount>& last_homing_position_snapshot() const noexcept;

 private:
  // -------------------------------- Internal --------------------------------

  // Guards start_homing, read and write
  [[nodiscard]] bool is_active() const noexcept { return transport_.is_open(); }

  // Walks the CAN interfaces for one carrying this side's state frames and pins
  // config_.interface_name to it, failing with InterfaceUnavailable when none does
  [[nodiscard]] Status discover_interface();

  // ------------------------------- Variables --------------------------------

  // Always set in the constructor init list, since HandConfig has no default constructor
  HandConfig config_;

  // config_.disabled_actuators spread into a per-actuator lookup, fixed after construction
  // A disabled actuator is never enabled, never reported as faulted and never homed
  std::array<bool, kActuatorCount> actuator_disabled_{};

  canfd::Protocol protocol_;
  canfd::Transport transport_;
  canfd::TxFrames tx_frames_{};

  // Latest decode, which is what the control word state machine reads
  canfd::StateFrames rx_frames_{};

  // Previous control word per actuator, needed for the fault reset rising edge
  std::array<std::uint16_t, kActuatorCount> last_control_word_{};

  // Only an actuator seen with bit5 high may later count a bit5 low as quick stop reached,
  // so a zeroed or never received status word cannot be mistaken for one
  std::array<bool, kActuatorCount> quick_stop_seen_high_{};

  HomingSequence homing_;

  // Reset to 0 by a frame, otherwise incremented up to saturation
  std::uint32_t cycles_since_last_frame_{0};

  bool any_frame_received_{false};
  canfd::DecodeResult last_receive_anomaly_{canfd::DecodeResult::Decoded};
  int last_transmit_errno_{0};
};

}  // namespace aidin_hand2
