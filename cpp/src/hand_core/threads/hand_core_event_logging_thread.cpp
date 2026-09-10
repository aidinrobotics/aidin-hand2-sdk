#include "hand_core/hand_core.hpp"

#include <time.h>

#include <cstdint>
#include <cstdio>
#include <string>
#include <system_error>

// flush_log
#include <aidin_hand2/logging/logging.hpp>

// receive_anomaly_suffix, transmit_errno_suffix, to_hex16
#include "hand_core/lifecycle/action_check.hpp"
#include "logging/log.hpp"

namespace aidin_hand2
{

namespace
{

// Not tied to the RT period, because the ring absorbs bursts
constexpr long kEventLoggingPeriodNanoseconds = 50000000L;

// One flush per second, since spdlog already flushes warn and above on its own
constexpr int kLoggingCyclesPerFlush = 20;

// Builds "<name> 0x<code>", the name for people and the hex for the drive manual
std::string fault_description(std::int32_t fault_code);

// Builds "<index>, <index>" from the bitmask, empty when no bit is set
std::string actuator_mask_text(std::int32_t mask);

// Builds "step (<n>/3) <what it is doing>", keeping the CiA402 stage names out of user messages
std::string homing_phase_text(HomingPhase phase);

}  // namespace

// ----------------------------- Event ring drain -----------------------------

// Turns the POD events the RT loop left in the ring into log lines
// It never touches control or comms, so falling behind only delays the log
void HandCore::run_event_logging_loop()
{
  // A silent exit would lose every later RT event, so leave at least one line behind
  try {
    int cycles_until_flush = kLoggingCyclesPerFlush;
    while (event_logging_running_.load(std::memory_order_relaxed)) {
      log_pending_events();
      if (--cycles_until_flush <= 0) {
        flush_log();
        cycles_until_flush = kLoggingCyclesPerFlush;
      }
      timespec pause{0, kEventLoggingPeriodNanoseconds};
      nanosleep(&pause, nullptr);
    }

    // Drain what the RT thread left behind before ending
    log_pending_events();
    flush_log();
  } catch (const std::exception& thread_exception) {
    log_critical(hand_side_, std::string("event logging thread terminated: ") + thread_exception.what() +
                 " — realtime events will no longer be logged");
    flush_log();
  } catch (...) {
    log_critical(hand_side_, "event logging thread terminated: unknown exception"
                 " — realtime events will no longer be logged");
    flush_log();
  }
}

void HandCore::log_pending_events()
{
  RealtimeEvent event{};
  while (event_ring_.pop(event)) {
    const std::string cycle_prefix = "[cycle " + std::to_string(event.cycle) + "] ";
    switch (event.kind) {
      case RealtimeEventKind::ActuatorFaultSet:
        // The fault mask keeps the other actuators running, so this is a warning
        log_warn(hand_side_, cycle_prefix + "actuator " + std::to_string(event.actuator_index) + " fault set (" +
                 fault_description(event.detail) + ")");
        break;
      case RealtimeEventKind::ActuatorFaultCleared:
        log_info(hand_side_, cycle_prefix + "actuator " + std::to_string(event.actuator_index) + " fault cleared");
        break;
      case RealtimeEventKind::ReceiveSilence: {
        const std::string base =
            cycle_prefix + "communication lost (no RX" +
            receive_anomaly_suffix(event.detail, event.can_error_class) + ")";
        if (event.stop_latched) {
          // Only RX died, so the quick stop is still reaching the wire and only its confirmation is lost
          log_critical(hand_side_, base + " — sending quick stop; awaiting confirmation");
        } else if (event.lifecycle == HandLifecycle::Stopped) {
          log_critical(hand_side_, base +
                       " while stopped — hand is no longer observable; call reconnect() to recover");
        } else if (event.lifecycle == HandLifecycle::Connected) {
          log_critical(hand_side_, base +
                       " while connected — hand is no longer observable; call reconnect() to recover");
        } else {
          log_warn(hand_side_, base + " — check CAN link and hand power");
        }
        break;
      }
      case RealtimeEventKind::ReceiveRestored:
        log_info(hand_side_, cycle_prefix + "communication restored (RX resumed)" +
                 (event.lifecycle == HandLifecycle::Faulted  ? " — call reconnect() to recover"
                  : event.lifecycle == HandLifecycle::Stopped ? " — call run() to resume"
                                                              : ""));
        break;
      case RealtimeEventKind::TransmitFailure:
        // Reported at once rather than waiting out the verdict window, because the quick stop
        // cannot be delivered at all
        log_critical(hand_side_, cycle_prefix + "CAN TX failed" + transmit_errno_suffix(event.detail) +
                     " — cannot deliver quick stop, retrying; drives may hold last command");
        break;
      case RealtimeEventKind::StopConfirmed:
        log_info(hand_side_, cycle_prefix + "quick stop attained — drives torque-free" +
                 (event.lifecycle == HandLifecycle::Faulted ? "; call reconnect() to recover" : ""));
        break;
      case RealtimeEventKind::StopNotConfirmed:
        log_critical(hand_side_, cycle_prefix + "quick stop NOT confirmed" +
                     " — drives may hold last torque command; restore CAN link or power off the hand");
        break;
      case RealtimeEventKind::ControlLoopFailure:
        // The exception text cannot travel as POD, so the dying thread logs it itself
        break;
      case RealtimeEventKind::HomingPhaseEntered:
        log_info(hand_side_, cycle_prefix + "homing: " +
                 homing_phase_text(static_cast<HomingPhase>(event.detail)));
        break;
      case RealtimeEventKind::HomingCompleted:
        log_info(hand_side_, cycle_prefix + "homing complete — home position established");
        break;
      case RealtimeEventKind::HomingFailed:
        // Index -1 is the whole hand, index 0 and up is one actuator's status word
        if (event.actuator_index >= 0) {
          log_debug(hand_side_, cycle_prefix + "homing failed: actuator " +
                    std::to_string(event.actuator_index) + " status " +
                    to_hex16(static_cast<std::uint16_t>(event.detail)));
        } else {
          const auto outcome = static_cast<HomingOutcome>(event.detail);
          const std::string actuators = actuator_mask_text(event.actuator_mask);
          log_warn(hand_side_, cycle_prefix + "homing failed at " +
                   homing_phase_text(homing_failure_phase(outcome)) + ": " +
                   homing_failure_reason(outcome) +
                   (actuators.empty() ? "" : " (actuators " + actuators + ")") +
                   " — home position NOT established; retry home()");
        }
        break;
    }
  }
  const std::uint64_t dropped = event_ring_.count_dropped();
  if (dropped != count_reported_dropped_) {
    log_warn(hand_side_, std::to_string(dropped - count_reported_dropped_) +
             " realtime events dropped (event ring full)");
    count_reported_dropped_ = dropped;
  }
}

// --------------------------------- Helpers ----------------------------------

namespace
{

const char* fault_name(ActuatorFault fault)
{
  switch (fault) {
    case ActuatorFault::None: return "None";
    case ActuatorFault::OverCurrentError: return "OverCurrentError";
    case ActuatorFault::OverVoltageError: return "OverVoltageError";
    case ActuatorFault::UnderVoltageError: return "UnderVoltageError";
    case ActuatorFault::OverTemperatureError: return "OverTemperatureError";
    case ActuatorFault::CurrentDetectionError: return "CurrentDetectionError";
    case ActuatorFault::SpeedError: return "SpeedError";
    case ActuatorFault::CommunicationError: return "CommunicationError";
    case ActuatorFault::FollowingError: return "FollowingError";
    case ActuatorFault::HallSensorError: return "HallSensorError";
    case ActuatorFault::OverLoadError: return "OverLoadError";
    case ActuatorFault::PositiveLimitSwitchError: return "PositiveLimitSwitchError";
    case ActuatorFault::NegativeLimitSwitchError: return "NegativeLimitSwitchError";
    case ActuatorFault::EmergencySwitchError: return "EmergencySwitchError";
    case ActuatorFault::Sto1Error: return "Sto1Error";
    case ActuatorFault::Sto2Error: return "Sto2Error";
    case ActuatorFault::SerialEncoderChannelAError: return "SerialEncoderChannelAError";
    case ActuatorFault::SerialEncoderChannelADisconnectedError: return "SerialEncoderChannelADisconnectedError";
    case ActuatorFault::SerialEncoderChannelBError: return "SerialEncoderChannelBError";
    case ActuatorFault::SerialEncoderChannelBDisconnectedError: return "SerialEncoderChannelBDisconnectedError";
  }
  return "UnknownError";
}

std::string fault_description(std::int32_t fault_code)
{
  char hex_text[8];
  std::snprintf(hex_text, sizeof(hex_text), "0x%04X", static_cast<unsigned>(fault_code) & 0xFFFFu);
  return std::string(fault_name(static_cast<ActuatorFault>(fault_code))) + " " + hex_text;
}

std::string actuator_mask_text(std::int32_t mask)
{
  std::string list;
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (!(mask & (1 << i))) continue;
    if (!list.empty()) list += ", ";
    list += std::to_string(i);
  }
  return list;
}

std::string homing_phase_text(HomingPhase phase)
{
  switch (phase) {
    case HomingPhase::Preparing:            return "step (1/3) preparing actuators";
    case HomingPhase::PreloadingToHardStop: return "step (2/3) pressing to hard stop";
    case HomingPhase::SettingOrigin:        return "step (3/3) setting origin";
  }
  return "step";
}

}  // namespace

}  // namespace aidin_hand2
