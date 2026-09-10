#include "hand_core/lifecycle/action_check.hpp"

// CAN_ERR_* class bits, used to tell a bus error apart from a missing ACK
#include <linux/can/error.h>

#include <system_error>

#include "hand_core/comms/canfd/protocol.hpp"

namespace aidin_hand2
{

namespace
{

// ------------------------------- Action table -------------------------------

// One bit per lifecycle value, so allowed_states is an OR of them and the test is one AND
constexpr unsigned bit(HandLifecycle state) { return 1U << static_cast<unsigned>(state); }

// kActionTable and kActionPhrase are fixed to this size
constexpr std::size_t kActionCount = static_cast<std::size_t>(HandAction::SetCommand) + 1;

// One rule per action
// The reason strings are appended to "Cannot <action>: " and reach the user as the message
struct ActionRule {
  // Lifecycle bits that allow this action
  unsigned    allowed_states;

  // Fragment starting with " — ", appended after the recorded cause
  const char* faulted_reason;

  // Complete sentences
  const char* disconnected_reason;
  const char* other_reason;
};

// Lifecycle bits
constexpr unsigned kDisc  = bit(HandLifecycle::Disconnected);
constexpr unsigned kConn  = bit(HandLifecycle::Connected);
constexpr unsigned kRun   = bit(HandLifecycle::Running);
constexpr unsigned kStop  = bit(HandLifecycle::Stopped);
constexpr unsigned kFault = bit(HandLifecycle::Faulted);

// Rejection reasons, named only so the table columns line up
constexpr auto kAfterFaultRebuild   = " — call reconnect() to rebuild";
constexpr auto kAfterFaultOrDestroy = " — use reconnect() or destroy";
constexpr auto kAfterFaultThenRun   = " — call reconnect(), then run()";
constexpr auto kAfterFault          = " — call reconnect()";
constexpr auto kAfterFaultFirst     = " — call reconnect() first";
constexpr auto kAlreadyConnected    = "already connected — call disconnect() first to rebuild the link";
constexpr auto kCallConnectFirst    = "not connected — call connect() first";
constexpr auto kCallConnect         = "not connected yet — call connect()";
constexpr auto kCallConnectThenRun  = "not connected yet — call connect(), then run()";
constexpr auto kCallRunFirst        = "control not running — call run() first";
constexpr auto kNotFaulted          = "not faulted — reconnect() recovers from Faulted only";
constexpr auto kNotRunning          = "not running — call run() first";

// Row order is HandAction order
// reconnect() is allowed only in Faulted, so that a failure cannot be cleared without the cause
// being seen. Connect, Disconnect, Run and Stop also accept the state that already satisfies their
// postcondition, where the method no-ops instead of rejecting a caller who asked for what it has.
constexpr ActionRule kActionTable[kActionCount] = {
    // action    allowed_states                 faulted_reason         disconnected_reason  other_reason
    /*Connect*/  {kDisc | kConn,                kAfterFaultRebuild,    nullptr,             kAlreadyConnected},
    /*Disconn*/  {kDisc | kConn | kRun | kStop, kAfterFaultOrDestroy,  nullptr,             nullptr},
    /*Run*/      {kConn | kRun | kStop,         kAfterFaultThenRun,    kCallConnectFirst,   nullptr},
    /*Stop*/     {kRun | kStop,                 kAfterFault,           kCallConnectFirst,   kCallRunFirst},
    /*Home*/     {kConn | kRun | kStop,         kAfterFaultFirst,      kCallConnectFirst,   nullptr},
    /*Reconn*/   {kFault,                       nullptr,               kCallConnect,        kNotFaulted},
    /*SetCmd*/   {kRun,                         kAfterFault,           kCallConnectThenRun, kNotRunning},
};

// Same index as kActionTable
constexpr const char* kActionPhrase[kActionCount] = {
    "connect hand", 
    "disconnect hand", 
    "run hand", 
    "stop hand",
    "home hand",    
    "reconnect hand",  
    "set command"
  };

// Builds "Cannot <action>: <reason>"
Status reject(HandAction action, ErrorCode code, const std::string& reason)
{
  return {code,
          std::string("Cannot ") + kActionPhrase[static_cast<std::size_t>(action)] + ": " + reason};
}

}  // namespace

// ---------------------------------- Action ----------------------------------

Status check_action_allowed(HandAction action, HandLifecycle requested, HandLifecycle observed,
                            bool destroyed, const StopCause& stop_cause, bool homed)
{
  if (destroyed) {
    return reject(action, ErrorCode::WrongCallOrder, "hand is destroyed — create a hand");
  }

  const ActionRule& rule = kActionTable[static_cast<std::size_t>(action)];

  // A fault is observed, never requested, and only an action that lists Faulted survives it
  if (observed == HandLifecycle::Faulted) {
    if ((rule.allowed_states & bit(HandLifecycle::Faulted)) != 0U) {
      return {};
    }
    // A Faulted rejection takes its ErrorCode from the recorded cause
    const bool communication = stop_cause.reason == RealtimeEventKind::ReceiveSilence ||
                               stop_cause.reason == RealtimeEventKind::TransmitFailure;
    return reject(action,
                  communication ? ErrorCode::CommunicationLost : ErrorCode::ControlLoopFault,
                  "hand faulted (" + stop_cause_phrase(stop_cause) + ")" + rule.faulted_reason);
  }

  if ((rule.allowed_states & bit(requested)) == 0U) {
    return reject(action, ErrorCode::WrongCallOrder,
                  requested == HandLifecycle::Disconnected ? rule.disconnected_reason
                                                           : rule.other_reason);
  }

  // A command needs a confirmed enable, not just a request: the drives may not have followed
  if (action == HandAction::SetCommand && observed != HandLifecycle::Running) {
    return reject(action, ErrorCode::WrongCallOrder, rule.other_reason);
  }

  // A target has no physical meaning before homing, and Idle passes require_homed false
  if (action == HandAction::SetCommand && !homed) {
    return reject(action, ErrorCode::WrongCallOrder, "not homed — call home() first");
  }
  return {};
}

// ---------------------------- Message rendering -----------------------------

std::string receive_anomaly_suffix(std::int32_t decode_result, std::int32_t can_error_class)
{
  switch (static_cast<canfd::DecodeResult>(decode_result)) {
    case canfd::DecodeResult::Error:
      // A missing ACK means power off or an unplugged hand, bus off needs a different fix
      if (can_error_class & CAN_ERR_BUSOFF) return ", last RX error: bus off";
      if (can_error_class & CAN_ERR_ACK)    return ", last RX error: bus error (no ACK)";
      return ", last RX error: bus error";
    case canfd::DecodeResult::LengthError:        return ", last RX error: malformed data";
    case canfd::DecodeResult::ConflictingCommand: return ", last RX error: conflicting command";
    case canfd::DecodeResult::Unknown:            return ", last RX error: unknown data";
    case canfd::DecodeResult::Decoded:            break;
  }
  return "";
}

std::string transmit_errno_suffix(std::int32_t transmit_errno)
{
  if (transmit_errno <= 0) return "";
  return " (errno " + std::to_string(transmit_errno) + ": " +
         std::generic_category().message(transmit_errno) + ")";
}

std::string to_hex16(std::uint16_t value)
{
  static const char digits[] = "0123456789ABCDEF";
  std::string out = "0x0000";
  for (int nibble = 0; nibble < 4; ++nibble) out[5 - nibble] = digits[(value >> (nibble * 4)) & 0xF];
  return out;
}

std::string stop_cause_phrase(const StopCause& cause)
{
  switch (cause.reason) {
    case RealtimeEventKind::ReceiveSilence:
      return "no RX since cycle " + std::to_string(cause.cycle) +
             receive_anomaly_suffix(cause.detail, cause.can_error_class);
    case RealtimeEventKind::TransmitFailure:
      return "CAN TX failed at cycle " + std::to_string(cause.cycle) +
             transmit_errno_suffix(cause.detail);
    case RealtimeEventKind::ControlLoopFailure:
      return "control loop terminated at cycle " + std::to_string(cause.cycle);
    default:
      return "cause not recorded";
  }
}

}  // namespace aidin_hand2
