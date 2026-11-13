#include "hand_core/lifecycle/action_check.hpp"

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
constexpr std::size_t kActionCount = static_cast<std::size_t>(HandAction::ResetTactileBias) + 1;

// A rejection's number and the text appended to "cannot <action>: ", empty where never used
struct Reason {
  std::uint16_t code;
  const char*   text;
};

// One rule per action
struct ActionRule {
  // Lifecycle bits that allow this action
  unsigned allowed_states;

  Reason disconnected_reason;
  Reason other_reason;
};

// Lifecycle bits
constexpr unsigned kDisc  = bit(HandLifecycle::Disconnected);
constexpr unsigned kConn  = bit(HandLifecycle::Connected);
constexpr unsigned kRun   = bit(HandLifecycle::Running);
constexpr unsigned kStop  = bit(HandLifecycle::Stopped);
constexpr unsigned kFault = bit(HandLifecycle::Faulted);

// Rejection reasons, named only so the table columns line up
constexpr Reason kNever            {0, nullptr};
constexpr Reason kAlreadyConnected {error_code::kAlreadyConnected, "already connected"};
constexpr Reason kNotConnected     {error_code::kNotConnected, "not connected"};
constexpr Reason kNotFaulted       {error_code::kNotFaulted, "not faulted"};
constexpr Reason kNotRunning       {error_code::kNotRunning, "not running"};

// Row order is HandAction order
// reconnect() is allowed only in Faulted, so that a failure cannot be cleared without the cause
// being seen. Connect, Disconnect, Run and Stop also accept the state that already satisfies their
// postcondition, where the method no-ops instead of rejecting a caller who asked for what it has.
constexpr ActionRule kActionTable[kActionCount] = {
    // action    allowed_states                 disconnected_reason  other_reason
    /*Connect*/  {kDisc | kConn,                kNever,              kAlreadyConnected},
    /*Disconn*/  {kDisc | kConn | kRun | kStop, kNever,              kNever},
    /*Run*/      {kConn | kRun | kStop,         kNotConnected,       kNever},
    /*Stop*/     {kRun | kStop,                 kNotConnected,       kNotRunning},
    /*Home*/     {kConn | kRun | kStop,         kNotConnected,       kNever},
    /*Reconn*/   {kFault,                       kNotConnected,       kNotFaulted},
    /*SetCmd*/   {kRun,                         kNotConnected,       kNotRunning},
    /*SetBias*/  {kConn | kRun | kStop,         kNotConnected,       kNever},
    /*RstBias*/  {kConn | kRun | kStop,         kNotConnected,       kNever},
};

// Same index as kActionTable
constexpr const char* kActionPhrase[kActionCount] = {
    "connect hand", 
    "disconnect hand", 
    "run hand", 
    "stop hand",
    "home hand",    
    "reconnect hand",
    "set command",
    "set tactile bias",
    "reset tactile bias"
  };

// Builds "cannot <action>: <reason>"
Status reject(HandAction action, std::uint16_t code, const std::string& reason)
{
  return {code,
          std::string("cannot ") + kActionPhrase[static_cast<std::size_t>(action)] + ": " + reason};
}

Status reject(HandAction action, const Reason& reason)
{
  return reject(action, reason.code, reason.text);
}

}  // namespace

// ---------------------------------- Action ----------------------------------

Status hand_destroyed()
{
  return {error_code::kHandDestroyed, "cannot use hand: hand is destroyed"};
}

Status check_action_allowed(HandAction action, HandLifecycle requested, HandLifecycle observed,
                            bool destroyed, const StopCause& stop_cause, bool homed)
{
  if (destroyed) {
    return hand_destroyed();
  }

  const ActionRule& rule = kActionTable[static_cast<std::size_t>(action)];

  // A fault is observed, never requested, and only an action that lists Faulted survives it
  if (observed == HandLifecycle::Faulted) {
    if ((rule.allowed_states & bit(HandLifecycle::Faulted)) != 0U) {
      return {};
    }
    const std::string cause = stop_cause_phrase(stop_cause);
    return reject(action, error_code::kHandFaulted,
                  cause.empty() ? "hand faulted" : "hand faulted (" + cause + ")");
  }

  if ((rule.allowed_states & bit(requested)) == 0U) {
    return reject(action, requested == HandLifecycle::Disconnected ? rule.disconnected_reason
                                                                   : rule.other_reason);
  }

  // A command needs a confirmed enable, not just a request: the drives may not have followed
  if (action == HandAction::SetCommand && observed != HandLifecycle::Running) {
    return reject(action, rule.other_reason);
  }

  // A target has no physical meaning before homing, and Idle passes require_homed false
  if (action == HandAction::SetCommand && !homed) {
    return reject(action, error_code::kNotHomed, "not homed");
  }
  return {};
}

// ---------------------------- Message rendering -----------------------------

std::string receive_anomaly_suffix(std::int32_t decode_result)
{
  if (static_cast<canfd::DecodeResult>(decode_result) == canfd::DecodeResult::ConflictingCommand) {
    return ", last RX error: conflicting command";
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

std::uint16_t stop_cause_code(const StopCause& cause)
{
  switch (cause.reason) {
    case RealtimeEventKind::ReceiveSilence:     return error_code::kReceiveLost;
    case RealtimeEventKind::TransmitFailure:    return error_code::kTransmitFailed;
    case RealtimeEventKind::ControlLoopFailure: return error_code::kControlLoopTerminated;
    default:                                    return 0;
  }
}

std::string stop_cause_phrase(const StopCause& cause)
{
  const std::string code = "E" + std::to_string(stop_cause_code(cause)) + " ";
  switch (cause.reason) {
    case RealtimeEventKind::ReceiveSilence:     return code + "communication lost";
    case RealtimeEventKind::TransmitFailure:    return code + "CAN TX failed";
    case RealtimeEventKind::ControlLoopFailure: return code + "control loop terminated";
    default:                                    return "";
  }
}

}  // namespace aidin_hand2
