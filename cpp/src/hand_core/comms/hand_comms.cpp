#include "hand_core/comms/hand_comms.hpp"

#include <linux/can.h>
// if_nameindex, used to enumerate the CAN interfaces for the "auto" scan
#include <net/if.h>
#include <time.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "hand_core/comms/state_machine/cia402.hpp"  // ControlWordMode, compute_control_word
#include "logging/log.hpp"

namespace aidin_hand2
{

namespace
{

// The channel carrying this side's state ids is the one this hand is wired to,
// 0x2xx for the left hand and 0x1xx for the right
constexpr char kAutoInterface[] = "auto";

// Lists the "can*" interfaces in whatever order the kernel gives them
std::vector<std::string> list_can_interfaces()
{
  std::vector<std::string> interfaces;
  struct if_nameindex* name_index = if_nameindex();
  if (name_index == nullptr) return interfaces;
  for (struct if_nameindex* entry = name_index; entry->if_index != 0 || entry->if_name != nullptr; ++entry) {
    if (entry->if_name != nullptr && std::strncmp(entry->if_name, "can", 3) == 0) {
      interfaces.emplace_back(entry->if_name);
    }
  }
  if_freenameindex(name_index);
  return interfaces;
}

}  // namespace

HandComms::HandComms(HandConfig config)
: config_(std::move(config)), protocol_(config_.hand_side)
{
  // Out of range indices are warned about and dropped
  for (const int actuator_index : config_.disabled_actuators) {
    if (actuator_index < 0 || actuator_index >= static_cast<int>(kActuatorCount)) {
      log_warn(config_.hand_side, "disabled_actuators: ignoring out-of-range index " + std::to_string(actuator_index));
      continue;
    }
    actuator_disabled_[static_cast<std::size_t>(actuator_index)] = true;
  }
}

HandComms::~HandComms() = default;

std::size_t HandComms::disabled_actuator_count() const noexcept
{
  std::size_t count = 0;
  for (const bool disabled : actuator_disabled_) {
    if (disabled) ++count;
  }
  return count;
}

// Opens each candidate briefly and watches through this side's filter, taking the first
// channel that decodes, and skipping the ones that stay silent or will not open
Status HandComms::discover_interface()
{
  const std::vector<std::string> candidates = list_can_interfaces();
  if (candidates.empty()) {
    return {error_code::kAutoNoInterface, "no CAN interface found for auto-discovery"};
  }
  constexpr int  kObserveIterations       = 15;
  constexpr long kObserveSleepNanoseconds = 10000000L;  // 10 ms × 15 = 150 ms
  const char* side_name = config_.hand_side == HandSide::Left ? "left" : "right";
  for (const std::string& candidate : candidates) {
    canfd::Transport probe;
    if (!probe.open(candidate).ok()) {
      log_debug(config_.hand_side, "auto-discovery: cannot open " + candidate + " — skipping");
      continue;
    }
    if (!probe.set_id_filters(protocol_.rx_filters())) {
      log_debug(config_.hand_side, "auto-discovery: cannot filter " + candidate + " — skipping");
      continue;
    }
    bool observed = false;
    canfd::StateFrames scratch{};
    canfd_frame frame{};
    for (int iteration = 0; iteration < kObserveIterations && !observed; ++iteration) {
      while (probe.recv(frame)) {
        // A state id from this side can only mean the hand is on this channel
        if (protocol_.decode(frame, scratch) == canfd::DecodeResult::Decoded) { observed = true; break; }
      }
      timespec pause{0, kObserveSleepNanoseconds};
      nanosleep(&pause, nullptr);
    }
    if (observed) {
      config_.interface_name = candidate;
      log_info(config_.hand_side,
               std::string("auto-discovery: ") + side_name + " hand found on " + candidate);
      return {};
    }
    log_debug(config_.hand_side, std::string("auto-discovery: no ") + side_name + " hand on " + candidate);
  }
  return {error_code::kAutoNoHand,
          std::string("no ") + side_name + " hand found on any CAN interface for auto-discovery"};
}

Status HandComms::activate()
{
  if (config_.interface_name == kAutoInterface) {
    if (const Status discovered = discover_interface(); !discovered.ok()) {
      return discovered;
    }
  }
  Status st = transport_.open(config_.interface_name);
  if (!st.ok()) {
    return st;
  }
  // The kernel passes only this hand's ids
  if (!transport_.set_id_filters(protocol_.rx_filters())) {
    const int filter_errno = errno;
    // Rolls back the partial open, so is_active() stays false
    transport_.close();
    return {error_code::kInterfaceOpenFailed, "CAN interface '" + config_.interface_name +
                                                  "' could not be opened (" +
                                                  std::generic_category().message(filter_errno) + ")"};
  }
  // An active master transmits every 2 ms, so a 150 ms window is enough to notice one
  constexpr int  kProbeIterations       = 15;
  constexpr long kProbeSleepNanoseconds = 10000000L;  // 10 ms × 15 = 150 ms
  log_debug(config_.hand_side, "probing " + config_.interface_name + " for a conflicting master");
  canfd_frame probe_frame{};
  for (int iteration = 0; iteration < kProbeIterations; ++iteration) {
    while (transport_.recv(probe_frame)) {
      if (protocol_.decode(probe_frame, rx_frames_) == canfd::DecodeResult::ConflictingCommand) {
        transport_.close();
        return {error_code::kOtherMaster,
                "another master is commanding this hand on " + config_.interface_name};
      }
    }
    timespec probe_sleep{0, kProbeSleepNanoseconds};
    nanosleep(&probe_sleep, nullptr);
  }
  // So a reconnect does not carry the previous socket's observations into the new session
  rx_frames_ = {};
  quick_stop_seen_high_ = {};
  last_control_word_ = {};
  cycles_since_last_frame_ = 0;
  any_frame_received_ = false;
  last_receive_anomaly_ = canfd::DecodeResult::Decoded;
  last_transmit_errno_ = 0;
  homing_.cancel();

  // Set once, so encode only has to fill the payload afterwards
  protocol_.init_tx(tx_frames_);
  log_debug(config_.hand_side, "comms ready on " + config_.interface_name +
            " — RX filters, TX headers set; no conflicting master");
  return {};
}

void HandComms::deactivate() noexcept
{
  // Only log when it was actually open, since this is safe to call repeatedly
  const bool was_open = transport_.is_open();
  transport_.close();
  if (was_open) log_debug(config_.hand_side, "CAN socket closed on " + config_.interface_name);
}

bool HandComms::start_homing()
{
  if (!is_active()) {
    return false;
  }
  // write() drives the stages from here, and disabled actuators take no part
  homing_.start(actuator_disabled_);
  return true;
}

bool HandComms::quick_stop_attained() const noexcept
{
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index) {
    // A disabled actuator is held at Shutdown and never receives a quick stop, so counting it
    // would leave the stop unconfirmed forever
    if (actuator_disabled_[actuator_index]) continue;
    // Never seen operating, so it was never enabled and cannot hold torque
    if (!quick_stop_seen_high_[actuator_index]) continue;
    // bit5 is active low, so 1 means the quick stop has not happened yet
    if (rx_frames_.status_word[actuator_index] & status_word::kQuickStopActive) return false;
  }
  return true;
}

bool HandComms::drives_enabled() const noexcept
{
  return status_word::enable_settled(rx_frames_.status_word, rx_frames_.error_code,
                                     actuator_disabled_);
}

bool HandComms::read(HandState& hand_state, ActuatorHealth& actuator_health)
{
  if (!is_active()) {
    return false;
  }
  // Drains the RX queue, keeping the latest frame per id
  // Anything other than Decoded leaves received_any false, so the comm-loss timeout still fires,
  // and another master's command is kept for the event detail
  canfd_frame received_frame{};
  bool received_any = false;
  while (transport_.recv(received_frame)) {
    const canfd::DecodeResult decode_result = protocol_.decode(received_frame, rx_frames_);
    if (decode_result == canfd::DecodeResult::Decoded) {
      received_any = true;
    } else if (decode_result == canfd::DecodeResult::ConflictingCommand) {
      last_receive_anomaly_ = decode_result;
    }
  }
  if (received_any) {
    cycles_since_last_frame_ = 0;
    any_frame_received_ = true;
  } else if (cycles_since_last_frame_ < UINT32_MAX) {
    ++cycles_since_last_frame_;
  }
  // Records which actuators have been seen with bit5 high
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index) {
    if (rx_frames_.status_word[actuator_index] & status_word::kQuickStopActive) quick_stop_seen_high_[actuator_index] = true;
  }
  protocol_.frames_to_hand_state(rx_frames_, hand_state);
  protocol_.frames_to_actuator_health(rx_frames_, actuator_health);
  // A disabled actuator is never enabled, so its status word must not reach the logs
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index) {
    if (!actuator_disabled_[actuator_index]) continue;
    actuator_health.fault[actuator_index]   = ActuatorFault::None;
    actuator_health.enabled[actuator_index] = false;
  }
  return true;
}

bool HandComms::write(const canfd::CommandFrames& command_frames, bool quick_stop)
{
  if (!is_active()) {
    return false;
  }

  // Quick stop wins over everything, cancelling a running homing without resuming it
  ControlWordMode control_word_mode = ControlWordMode::Enable;
  canfd::CommandFrames outgoing_frames = command_frames;
  if (quick_stop) {
    homing_.cancel();
    control_word_mode = ControlWordMode::QuickStop;
  } else if (homing_.is_active()) {
    control_word_mode = homing_.step(rx_frames_, outgoing_frames);
  }

  // Only the control word is added here, and a disabled actuator is held at Shutdown so its
  // drive never powers up and never reports a Hall error
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index) {
    const ControlWordMode actuator_mode =
        actuator_disabled_[actuator_index] ? ControlWordMode::Shutdown : control_word_mode;
    const std::uint16_t control_word = compute_control_word(
        actuator_mode, rx_frames_.status_word[actuator_index], last_control_word_[actuator_index]);
    outgoing_frames.control_word[actuator_index] = control_word;
    last_control_word_[actuator_index]           = control_word;
  }
  protocol_.encode(outgoing_frames, tx_frames_);
  // The gen2 order: mode, homing, position, effort, control word
  bool all_sent = true;
  all_sent &= transport_.send(tx_frames_.mode_of_operation);
  all_sent &= transport_.send(tx_frames_.homing_method);
  all_sent &= transport_.send(tx_frames_.target_position);
  all_sent &= transport_.send(tx_frames_.target_effort);
  all_sent &= transport_.send(tx_frames_.control_word);
  if (!all_sent) {
    last_transmit_errno_ = transport_.last_send_errno();
    return false;
  }
  return true;
}

bool HandComms::is_homing() const noexcept
{
  return homing_.is_active();
}

HomingOutcome HandComms::last_homing_outcome() const noexcept
{
  return homing_.outcome();
}

HomingPhase HandComms::homing_phase() const noexcept
{
  return homing_.current_phase();
}

const std::array<bool, kActuatorCount>& HandComms::last_homing_failed_actuators() const noexcept
{
  return homing_.failed_actuators();
}

const char* HandComms::last_homing_failure_stage() const noexcept
{
  return homing_.failed_stage_name();
}

const std::array<std::uint16_t, kActuatorCount>& HandComms::last_homing_status_snapshot() const noexcept
{
  return homing_.failed_status_snapshot();
}

const std::array<std::int32_t, kActuatorCount>& HandComms::last_homing_position_snapshot() const noexcept
{
  return homing_.failed_position_snapshot();
}


}  // namespace aidin_hand2