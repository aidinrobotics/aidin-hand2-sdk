#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// CiA402 status word tests and control word rules, used to pick each cycle's control word
// Pure logic with no transport, so it is unit tested without hardware
// CiA402 is implementation vocabulary and is never exposed on the public API

namespace aidin_hand2
{

// -------------------------------- Status word -------------------------------

namespace status_word
{
constexpr std::uint16_t kReadyToSwitchOn  = 1u << 0;
constexpr std::uint16_t kSwitchedOn       = 1u << 1;
constexpr std::uint16_t kOperationEnabled = 1u << 2;
constexpr std::uint16_t kFault            = 1u << 3;

// Active low: 1 normally, 0 while quick stop is running
constexpr std::uint16_t kQuickStopActive  = 1u << 5;

// Homing bits, meaningful only in mode 6
// Welcon Servo Drive Reference Manual 5.4.2: bit13 0 with bit12 0 is running, bit13 0 with
// bit12 1 is complete, bit13 1 is an error
// bit12 stays 1 after a homing, so completion is read as a rising edge, and re-entering
// Operation Enabled beforehand resets it to a known 0
constexpr std::uint16_t kHomingAttained = 1u << 12;
constexpr std::uint16_t kHomingError    = 1u << 13;

// A CiA402 state is a bit pattern, not one bit, so the state bits are matched as a whole
constexpr std::uint16_t kStateBits            = 0x6F;  // bit 0,1,2,3,5,6
constexpr std::uint16_t kOperationEnabledBits = 0x27;  // bit 0,1,2,5 (3,6 = 0)

// The one Operation Enabled test: ActuatorHealth::enabled and the run() enable wait share it
inline bool is_operation_enabled(std::uint16_t status)
{
  return (status & kStateBits) == kOperationEnabledBits;
}

// Whether an enable request has settled, which is what run() waits on
// A faulted actuator is skipped: the control word ladder keeps resetting it, so waiting for it
// would only turn one dead motor into a failed run(). An actuator that is merely not enabled yet
// is waited for, and the request fails if it never arrives
// Disabled actuators are held at Shutdown, and all faulted means nothing came up at all
template <std::size_t N>
inline bool enable_settled(const std::array<std::uint16_t, N>& status,
                           const std::array<std::uint16_t, N>& error_code,
                           const std::array<bool, N>& disabled)
{
  bool any_present = false;
  bool any_enabled = false;
  for (std::size_t actuator_index = 0; actuator_index < N; ++actuator_index) {
    if (disabled[actuator_index]) continue;
    any_present = true;
    if (error_code[actuator_index] != 0) continue;
    if (!is_operation_enabled(status[actuator_index])) return false;
    any_enabled = true;
  }
  return !any_present || any_enabled;
}
}  // namespace status_word

// ------------------------------- Control word -------------------------------

namespace control_word
{
// bit2 is active low, so clearing it requests the quick stop
constexpr std::uint16_t kQuickStop             = 0x02;

constexpr std::uint16_t kShutdown              = 0x06;
constexpr std::uint16_t kSwitchOn              = 0x07;
constexpr std::uint16_t kEnableOperation       = 0x0F;

// 0x0F plus bit4, which starts the mode 6 homing
constexpr std::uint16_t kEnableOperationHoming = 0x1F;

// Applied on a rising edge of bit7
constexpr std::uint16_t kFaultReset            = 0x80;
}  // namespace control_word

// Enable walks the CiA402 ladder, every other mode sends a fixed command
// SwitchOn sent while Operation Enabled becomes a Disable Operation, which is how the state
// machine is re-entered to clear the homing attained bit
// QuickStop stops the drive itself and leaves it torque-free in Switch On Disabled
enum class ControlWordMode { Enable, SwitchOn, Homing, Shutdown, QuickStop };

// Picks one actuator's control word for this cycle
inline std::uint16_t compute_control_word(ControlWordMode mode, std::uint16_t status,
                                          std::uint16_t last_control_word)
{
  if (mode == ControlWordMode::Homing)    return control_word::kEnableOperationHoming;
  if (mode == ControlWordMode::SwitchOn)  return control_word::kSwitchOn;
  if (mode == ControlWordMode::Shutdown)  return control_word::kShutdown;
  if (mode == ControlWordMode::QuickStop) return control_word::kQuickStop;

  // A fault alternates 0x80 and 0x06 to make the bit7 rising edge, otherwise the ladder follows status
  if (status & status_word::kFault) {
    return (last_control_word == control_word::kFaultReset) ? control_word::kShutdown
                                                            : control_word::kFaultReset;
  }
  if (status & (status_word::kOperationEnabled | status_word::kSwitchedOn)) return control_word::kEnableOperation;
  if (status & status_word::kReadyToSwitchOn)                               return control_word::kSwitchOn;
  return control_word::kShutdown;
}

}  // namespace aidin_hand2
