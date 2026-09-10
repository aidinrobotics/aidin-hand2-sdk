#pragma once

#include <linux/can.h>

#include <array>
#include <cstdint>
#include <vector>

#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>

// This hand's CAN rules: it gives meaning to the frames transport moved, and builds the ones to send
//   RX: canfd_frame --decode--> StateFrames --frames_to_hand_state--> HandState
//   TX: command --(command_to_frames)--> CommandFrames --encode--> TxFrames --> bus
// One object holds the left and right CAN ids and the payload endianness and scaling
// Wire slot order is SDK actuator order, so decode and encode gather and scatter in sequence

namespace aidin_hand2::canfd
{

// -------------------------------- Constants ---------------------------------

// Homing takes the hard stop as drive 0, and commanding 0 there would keep pressing into it,
// so the SDK treats this far off the hard stop as its own 0
// The conversion happens only at the wire boundary, in encode and decode
inline constexpr std::int32_t kHomeOffsetCount = 1000;

// CiA402 mode_of_operation values
namespace mode_of_operation
{
constexpr std::int8_t kHoming = 6;

// Cyclic Synchronous Position
constexpr std::int8_t kCSP    = 8;

// Cyclic Synchronous Torque
constexpr std::int8_t kCST    = 10;
}  // namespace mode_of_operation

// --------------------------------- Signals ----------------------------------

// Decoded from what the hand sends, actuator arrays in SDK index order
struct StateFrames {
  std::array<std::int32_t,  kActuatorCount> actual_position{};
  std::array<std::int32_t,  kActuatorCount> actual_velocity{};
  std::array<std::int16_t,  kActuatorCount> actual_current{};
  std::array<std::uint16_t, kActuatorCount> status_word{};
  std::array<std::uint16_t, kActuatorCount> error_code{};

  std::array<std::uint16_t, kTactileTaxelsPerFinger> tactile_thumb{};
  std::array<std::uint16_t, kTactileTaxelsPerFinger> tactile_index{};
  std::array<std::uint16_t, kTactileTaxelsPerFinger> tactile_middle{};
  std::array<std::uint16_t, kTactileTaxelsPerFinger> tactile_ring{};
  std::array<std::uint16_t, kTactileTaxelsPerFinger> tactile_baby{};

  std::array<std::uint16_t, kPalm1UpperCount> palm1_upper{};
  std::array<std::uint16_t, kPalm1LowerCount> palm1_lower{};
  std::array<std::uint16_t, kPalm2Count>      palm2{};
};

// What the host will send, actuator arrays in SDK index order
// target_effort is TargetTorque in the firmware
struct CommandFrames {
  std::array<std::int32_t,  kActuatorCount> target_position{};
  std::array<std::int16_t,  kActuatorCount> target_effort{};
  std::array<std::int16_t,  kActuatorCount> max_effort{};
  std::array<std::uint16_t, kActuatorCount> control_word{};
  std::array<std::int8_t,   kActuatorCount> mode_of_operation{};
  std::array<std::int8_t,   kActuatorCount> homing_method{};
};

// The five CAN-FD frames actually sent, their headers set once by init_tx
struct TxFrames {
  canfd_frame target_position{};
  canfd_frame target_effort{};
  canfd_frame control_word{};
  canfd_frame mode_of_operation{};
  canfd_frame homing_method{};
};

// --------------------------------- Protocol ---------------------------------

// ConflictingCommand means another program is commanding the same hand, since a command id
// only ever travels host to hand and a sender does not receive its own frames
enum class DecodeResult : int {
  Unknown            = 0,
  Decoded            = 1,
  ConflictingCommand = 2,

  // CAN_ERR_FLAG, such as bus-off
  Error              = -1,

  // A known id whose payload is shorter than the protocol says
  LengthError        = -2,
};

// Computed once per side in the Protocol constructor
struct HandCanIds {
  std::uint32_t state_position;
  std::uint32_t state_velocity;
  std::uint32_t state_current;
  std::uint32_t state_status_word;
  std::array<std::uint32_t, kFingerCount> tactile_finger;
  std::uint32_t tactile_palm1_upper;
  std::uint32_t tactile_palm1_lower;
  std::uint32_t tactile_palm2;

  std::uint32_t cmd_target_position;
  std::uint32_t cmd_target_effort;
  std::uint32_t cmd_control_word;
  std::uint32_t cmd_mode_of_operation;
  std::uint32_t cmd_homing_method;
};

class Protocol {
 public:
  explicit Protocol(HandSide side);

  // The 12 state ids plus the 5 command ids, the latter to notice a second master
  [[nodiscard]] std::vector<can_filter> rx_filters() const;

  // Sets the five TX frame headers, once after open
  void init_tx(TxFrames& tx) const;

  // Frame to signal
  DecodeResult decode(const canfd_frame& rx, StateFrames& frames) const;

  // Signal to frame payload
  void encode(const CommandFrames& frames, TxFrames& tx) const;

  // Signal to observation
  void frames_to_hand_state(const StateFrames& frames, HandState& state) const;

  // Signal to per-actuator enable and fault
  void frames_to_actuator_health(const StateFrames& frames, ActuatorHealth& health) const;

 private:
  // Cached, so decode and encode do not rebuild them every cycle
  HandCanIds ids_;
};

}  // namespace aidin_hand2::canfd
