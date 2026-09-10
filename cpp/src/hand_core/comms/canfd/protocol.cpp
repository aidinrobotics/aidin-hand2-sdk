#include "hand_core/comms/canfd/protocol.hpp"

#include <cstddef>
#include <cstring>

#include "hand_core/comms/state_machine/cia402.hpp"  // status_word::is_operation_enabled

namespace aidin_hand2::canfd
{

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
              "canfd serdes assumes a little-endian host");

namespace
{

// Little-endian scalar access through memcpy, because canfd_frame::data may be misaligned
std::int32_t  read_int32 (const std::uint8_t* b) { std::int32_t  v; std::memcpy(&v, b, sizeof v); return v; }
std::int16_t  read_int16 (const std::uint8_t* b) { std::int16_t  v; std::memcpy(&v, b, sizeof v); return v; }
std::uint16_t read_uint16(const std::uint8_t* b) { std::uint16_t v; std::memcpy(&v, b, sizeof v); return v; }
void write_int32 (std::uint8_t* b, std::int32_t  v) { std::memcpy(b, &v, sizeof v); }
void write_int16 (std::uint8_t* b, std::int16_t  v) { std::memcpy(b, &v, sizeof v); }
void write_uint16(std::uint8_t* b, std::uint16_t v) { std::memcpy(b, &v, sizeof v); }

// N values from offset b, wire slot order matching SDK actuator order
template <std::size_t N>
void read_int32_array(std::array<std::int32_t, N>& out, const std::uint8_t* b) {
  for (std::size_t i = 0; i < N; ++i) out[i] = read_int32(b + i * 4);
}
template <std::size_t N>
void read_int16_array(std::array<std::int16_t, N>& out, const std::uint8_t* b) {
  for (std::size_t i = 0; i < N; ++i) out[i] = read_int16(b + i * 2);
}
template <std::size_t N>
void read_uint16_array(std::array<std::uint16_t, N>& out, const std::uint8_t* b) {
  for (std::size_t i = 0; i < N; ++i) out[i] = read_uint16(b + i * 2);
}
template <std::size_t N>
void write_int32_array(std::uint8_t* b, const std::array<std::int32_t, N>& v) {
  for (std::size_t i = 0; i < N; ++i) write_int32(b + i * 4, v[i]);
}
template <std::size_t N>
void write_int16_array(std::uint8_t* b, const std::array<std::int16_t, N>& v) {
  for (std::size_t i = 0; i < N; ++i) write_int16(b + i * 2, v[i]);
}
template <std::size_t N>
void write_uint16_array(std::uint8_t* b, const std::array<std::uint16_t, N>& v) {
  for (std::size_t i = 0; i < N; ++i) write_uint16(b + i * 2, v[i]);
}
template <std::size_t N>
void write_int8_array(std::uint8_t* b, const std::array<std::int8_t, N>& v) {
  for (std::size_t i = 0; i < N; ++i) b[i] = static_cast<std::uint8_t>(v[i]);
}

template <std::size_t N>
void cast_to_double(std::array<double, N>& dst, const std::array<std::uint16_t, N>& src) {
  for (std::size_t i = 0; i < N; ++i) dst[i] = static_cast<double>(src[i]);
}

// Minimum payload the protocol promises for each state frame, in bytes
// CAN-FD pads to a valid DLC, so decode rejects only what is shorter, never what is longer
constexpr std::size_t kLenPosition = kActuatorCount * sizeof(std::int32_t);             // 64
constexpr std::size_t kLenVelocity = kActuatorCount * sizeof(std::int32_t);             // 64
constexpr std::size_t kLenCurrent  = kActuatorCount * sizeof(std::int16_t);             // 32
constexpr std::size_t kLenStatus   = 2 * kActuatorCount * sizeof(std::uint16_t);        // 64 (status+error)
constexpr std::size_t kLenFinger   = kTactileTaxelsPerFinger * sizeof(std::uint16_t);   // 34
constexpr std::size_t kLenPalmUp   = kPalm1UpperCount * sizeof(std::uint16_t);          // 40
constexpr std::size_t kLenPalmLow  = kPalm1LowerCount * sizeof(std::uint16_t);          // 40
constexpr std::size_t kLenPalm2    = kPalm2Count * sizeof(std::uint16_t);               // 36

// CAN id table, 0x2xx for the left hand and 0x1xx for the right
constexpr HandCanIds kLeftCanIds{
  // state (device → host)
  .state_position    = 0x221,  // INT32×16
  .state_velocity    = 0x222,  // INT32×16
  .state_current     = 0x223,  // INT16×16  (firmware: Current)
  .state_status_word = 0x224,  // UINT16×32 (status 0~31B + error 32~63B)
  // tactile (device → host) — thumb, index, middle, ring, baby
  .tactile_finger     = {0x231, 0x232, 0x233, 0x234, 0x235},
  .tactile_palm1_upper = 0x236,
  .tactile_palm1_lower = 0x237,
  .tactile_palm2       = 0x238,
  // command (host → device)
  .cmd_target_position   = 0x211,  // INT32×16  (DLC 64)
  .cmd_target_effort     = 0x212,  // firmware: TargetTorque, INT16×32 target+max
  .cmd_control_word      = 0x213,  // UINT16×16 (DLC 32)
  .cmd_mode_of_operation = 0x214,  // INT8×16
  .cmd_homing_method     = 0x215,  // INT8×16
};

constexpr HandCanIds kRightCanIds{
  .state_position    = 0x121,
  .state_velocity    = 0x122,
  .state_current     = 0x123,
  .state_status_word = 0x124,
  .tactile_finger     = {0x131, 0x132, 0x133, 0x134, 0x135},
  .tactile_palm1_upper = 0x136,
  .tactile_palm1_lower = 0x137,
  .tactile_palm2       = 0x138,
  .cmd_target_position   = 0x111,
  .cmd_target_effort     = 0x112,
  .cmd_control_word      = 0x113,
  .cmd_mode_of_operation = 0x114,
  .cmd_homing_method     = 0x115,
};

const HandCanIds& hand_can_ids(HandSide side)
{
  return (side == HandSide::Left) ? kLeftCanIds : kRightCanIds;
}

}  // namespace

Protocol::Protocol(HandSide side)
: ids_(hand_can_ids(side))
{
}

// --------------------------------- Protocol ---------------------------------

std::vector<can_filter> Protocol::rx_filters() const
{
  // Order: position, velocity, current, status_word, finger0..4, palm upper, lower, palm2
  return {
    {ids_.state_position,    CAN_SFF_MASK},
    {ids_.state_velocity,    CAN_SFF_MASK},
    {ids_.state_current,     CAN_SFF_MASK},
    {ids_.state_status_word, CAN_SFF_MASK},
    {ids_.tactile_finger[0], CAN_SFF_MASK},
    {ids_.tactile_finger[1], CAN_SFF_MASK},
    {ids_.tactile_finger[2], CAN_SFF_MASK},
    {ids_.tactile_finger[3], CAN_SFF_MASK},
    {ids_.tactile_finger[4], CAN_SFF_MASK},
    {ids_.tactile_palm1_upper, CAN_SFF_MASK},
    {ids_.tactile_palm1_lower, CAN_SFF_MASK},
    {ids_.tactile_palm2,       CAN_SFF_MASK},
    // recv_own_msgs is off, so receiving a command id means another program is commanding
    {ids_.cmd_target_position,   CAN_SFF_MASK},
    {ids_.cmd_target_effort,     CAN_SFF_MASK},
    {ids_.cmd_control_word,      CAN_SFF_MASK},
    {ids_.cmd_mode_of_operation, CAN_SFF_MASK},
    {ids_.cmd_homing_method,     CAN_SFF_MASK},
  };
}

void Protocol::init_tx(TxFrames& tx) const
{
  // Without CANFD_BRS the data phase would send at the nominal 1 Mbit and saturate the bus
  tx = TxFrames{};
  tx.target_position.can_id   = ids_.cmd_target_position;   tx.target_position.len   = 64; tx.target_position.flags   = CANFD_BRS;
  tx.target_effort.can_id     = ids_.cmd_target_effort;     tx.target_effort.len     = 64; tx.target_effort.flags     = CANFD_BRS;
  tx.control_word.can_id      = ids_.cmd_control_word;      tx.control_word.len      = 32; tx.control_word.flags      = CANFD_BRS;
  tx.mode_of_operation.can_id = ids_.cmd_mode_of_operation; tx.mode_of_operation.len = 16; tx.mode_of_operation.flags = CANFD_BRS;
  tx.homing_method.can_id     = ids_.cmd_homing_method;     tx.homing_method.len     = 16; tx.homing_method.flags     = CANFD_BRS;
}

DecodeResult Protocol::decode(const canfd_frame& rx, StateFrames& frames) const
{
  if (rx.can_id & CAN_ERR_FLAG) {
    return DecodeResult::Error;
  }
  const std::uint32_t id = rx.can_id & CAN_EFF_MASK;

  // The CAN-FD CRC rules out truncation, so a short payload means firmware and SDK disagree
  if (id == ids_.state_position) {
    if (rx.len < kLenPosition) return DecodeResult::LengthError;
    read_int32_array(frames.actual_position, rx.data);
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      // Drive origin to SDK origin
      frames.actual_position[i] -= kHomeOffsetCount;
    }
  } else if (id == ids_.state_velocity) {
    if (rx.len < kLenVelocity) return DecodeResult::LengthError;
    read_int32_array(frames.actual_velocity, rx.data);
  } else if (id == ids_.state_current) {
    if (rx.len < kLenCurrent) return DecodeResult::LengthError;
    read_int16_array(frames.actual_current, rx.data);
  } else if (id == ids_.state_status_word) {
    // statusword frame: status(0~31B) + error(32~63B).
    if (rx.len < kLenStatus) return DecodeResult::LengthError;
    read_uint16_array(frames.status_word, rx.data);
    read_uint16_array(frames.error_code,  rx.data + 32);
  } else if (id == ids_.tactile_finger[0]) {
    if (rx.len < kLenFinger) return DecodeResult::LengthError;
    read_uint16_array(frames.tactile_thumb, rx.data);
  } else if (id == ids_.tactile_finger[1]) {
    if (rx.len < kLenFinger) return DecodeResult::LengthError;
    read_uint16_array(frames.tactile_index, rx.data);
  } else if (id == ids_.tactile_finger[2]) {
    if (rx.len < kLenFinger) return DecodeResult::LengthError;
    read_uint16_array(frames.tactile_middle, rx.data);
  } else if (id == ids_.tactile_finger[3]) {
    if (rx.len < kLenFinger) return DecodeResult::LengthError;
    read_uint16_array(frames.tactile_ring, rx.data);
  } else if (id == ids_.tactile_finger[4]) {
    if (rx.len < kLenFinger) return DecodeResult::LengthError;
    read_uint16_array(frames.tactile_baby, rx.data);
  } else if (id == ids_.tactile_palm1_upper) {
    if (rx.len < kLenPalmUp) return DecodeResult::LengthError;
    read_uint16_array(frames.palm1_upper, rx.data);
  } else if (id == ids_.tactile_palm1_lower) {
    if (rx.len < kLenPalmLow) return DecodeResult::LengthError;
    read_uint16_array(frames.palm1_lower, rx.data);
  } else if (id == ids_.tactile_palm2) {
    if (rx.len < kLenPalm2) return DecodeResult::LengthError;
    read_uint16_array(frames.palm2, rx.data);
  } else if (id == ids_.cmd_target_position || id == ids_.cmd_target_effort ||
             id == ids_.cmd_control_word || id == ids_.cmd_mode_of_operation ||
             id == ids_.cmd_homing_method) {
    // A command id can only have come from another program
    return DecodeResult::ConflictingCommand;
  } else {
    return DecodeResult::Unknown;
  }
  return DecodeResult::Decoded;
}

void Protocol::encode(const CommandFrames& frames, TxFrames& tx) const
{
  // Serialises SDK actuator i into wire slot i, mirroring decode
  std::array<std::int32_t, kActuatorCount> wire_target{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    // SDK origin to drive origin
    wire_target[i] = frames.target_position[i] + kHomeOffsetCount;
  }
  write_int32_array(tx.target_position.data, wire_target);
  // effort frame: target(0~31B) + max(32~63B). (firmware: TargetTorque)
  write_int16_array(tx.target_effort.data,      frames.target_effort);
  write_int16_array(tx.target_effort.data + 32, frames.max_effort);
  write_uint16_array(tx.control_word.data,      frames.control_word);
  write_int8_array  (tx.mode_of_operation.data, frames.mode_of_operation);
  write_int8_array  (tx.homing_method.data,     frames.homing_method);
}

void Protocol::frames_to_hand_state(const StateFrames& frames, HandState& state) const
{
  // decode already put these in SDK index order, so this only converts types
  auto& act = state.actuators;
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    act.position_count[i] = static_cast<double>(frames.actual_position[i]);
    act.velocity_rpm[i] = static_cast<double>(frames.actual_velocity[i]);
    act.current_mA[i]   = static_cast<double>(frames.actual_current[i]);
  }

  // Five fingers of 17 taxels, in order
  cast_to_double(state.tactile.fingers[0], frames.tactile_thumb);
  cast_to_double(state.tactile.fingers[1], frames.tactile_index);
  cast_to_double(state.tactile.fingers[2], frames.tactile_middle);
  cast_to_double(state.tactile.fingers[3], frames.tactile_ring);
  cast_to_double(state.tactile.fingers[4], frames.tactile_baby);

  // Three palm pads flattened into 58 cells: 20 upper, 20 lower, 18 palm2
  for (std::size_t i = 0; i < kPalm1UpperCount; ++i)
    state.tactile.palm[i] = static_cast<double>(frames.palm1_upper[i]);
  for (std::size_t i = 0; i < kPalm1LowerCount; ++i)
    state.tactile.palm[kPalm1UpperCount + i] = static_cast<double>(frames.palm1_lower[i]);
  for (std::size_t i = 0; i < kPalm2Count; ++i)
    state.tactile.palm[kPalm1UpperCount + kPalm1LowerCount + i] = static_cast<double>(frames.palm2[i]);
}

void Protocol::frames_to_actuator_health(const StateFrames& frames, ActuatorHealth& health) const
{
  // Derived from statusword and error_code, already in SDK index order
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    health.enabled[i] = status_word::is_operation_enabled(frames.status_word[i]);
    health.fault[i]   = static_cast<ActuatorFault>(frames.error_code[i]);
  }
}

}  // namespace aidin_hand2::canfd
