#include "hand_core/hand_core.hpp"

// if_nametoindex, used to tell a gone interface from a re-created one
#include <net/if.h>
#include <time.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <system_error>

#include "hand_core/lifecycle/action_check.hpp"
#include "hand_core/util/monotonic_clock.hpp"
#include "logging/log.hpp"

// HandCore is split by the thread that runs the code
//   hand_core.cpp                               the user thread
//   threads/hand_core_control_thread.cpp        the RT thread
//   threads/hand_core_event_logging_thread.cpp  the event logging thread

namespace aidin_hand2
{

namespace
{

// Percent of rated current, 1000 = 100%
constexpr double kDefaultMaxEffort = 1000.0;
constexpr double kMaxEffortLimit   = 2000.0;

// Off only under AIDIN_HAND2_WORKSPACE_CLAMP=0, for measuring the reachable limits
bool workspace_clamp_enabled();

// A non-finite target would reach a static_cast below, which is undefined behaviour
template <std::size_t Size>
bool all_finite(const std::array<double, Size>& values);

bool is_command_valid(const JointPositionCommand& command);
bool is_command_valid(const JointImpedanceCommand& command);
bool is_command_valid(const ActuatorPositionCommand& command);
bool is_command_valid(const ActuatorEffortCommand& command);

// Drops the command without throwing, and logs once on the accept to reject change so 500 Hz
// cannot flood the log
Status drop_invalid_command(HandSide side, std::atomic<std::uint64_t>& count,
                            std::atomic<bool>& rejection_active);

// Builds "<index>,<index>" from the actuators that failed homing, or "none"
std::string failed_actuator_list(const std::array<bool, kActuatorCount>& failed);

// Builds "homing failed at stage <stage>:" plus " <index>={status <hex>, pos <count>}" per actuator
std::string homing_debug_detail(const char* stage,
                                const std::array<bool, kActuatorCount>& failed,
                                const std::array<std::uint16_t, kActuatorCount>& status,
                                const std::array<std::int32_t, kActuatorCount>& position);

}  // namespace

// ------------------------------- Construction -------------------------------

HandCore::HandCore(const HandConfig& config)
: comms_(config),
  period_nanoseconds_(1000000000L / config.control_rate),
  rt_cpu_affinity_(config.rt_cpu_affinity),
  auto_reconnect_(config.auto_reconnect),
  auto_reconnect_timeout_ms_(config.auto_reconnect_timeout_ms),
  auto_reconnect_home_(config.auto_reconnect_home),
  auto_home_(config.auto_home),
  workspace_clamp_(workspace_clamp_enabled()),
  interface_name_(config.interface_name),
  hand_side_(config.hand_side),
  joint_position_controller_(period_nanoseconds_),
  joint_impedance_controller_(period_nanoseconds_)
{
  // Seed the buffer so the RT first cycle reads Idle at the default effort
  staging_command_.max_effort_pct.fill(kDefaultMaxEffort);
  command_buffer_.write(staging_command_);
}

HandCore::~HandCore()
{
  close();
}

// -------------------------------- Connection --------------------------------

Status HandCore::connect()
{
  if (Status allowed = check_allowed(HandAction::Connect); !allowed.ok()) {
    return allowed;
  }
  // Only a confirmed link is a no-op, so a retry after one that never observed a frame reopens
  if (requested_lifecycle_.load() == HandLifecycle::Connected &&
      lifecycle_.load() == HandLifecycle::Connected) {
    log_info(hand_side_, "connect() ignored — already connected");
    return {};
  }
  const Status connected = comms_.connect();
  if (!connected.ok()) {
    return connected;
  }

  // "auto" may have resolved to a real channel
  interface_name_ = comms_.interface_name();

  const Status session = comms_.run();
  if (!session.ok()) {
    comms_.disconnect();
    return session;
  }
  check_first_frame_.store(false);
  check_communication_.store(false);

  // A new link may mean a rebooted drive
  homing_state_.store(HomingState::NotRun);

  republish_staged();

  // The RT loop only stays alive while the request is not Disconnected, so set it before the start
  requested_lifecycle_.store(HandLifecycle::Connected);
  try {
    if (!event_logging_thread_.joinable()) {
      event_logging_running_.store(true);
      event_logging_thread_ = std::thread(&HandCore::run_event_logging_loop, this);
    }
    rt_thread_ = std::thread(&HandCore::run_control_loop, this);
  } catch (const std::system_error& thread_failure) {
    requested_lifecycle_.store(HandLifecycle::Disconnected);
    lifecycle_.store(HandLifecycle::Disconnected);
    event_logging_running_.store(false);
    if (event_logging_thread_.joinable()) event_logging_thread_.join();
    comms_.disconnect();
    return {ErrorCode::ControlLoopFault,
            std::string("Cannot connect hand: failed to start SDK threads (") + thread_failure.what() + ")"};
  }

  // Succeed only once observation has started, so state() never returns a blank snapshot
  if (const Status observed = wait_first_frame("connect hand"); !observed.ok()) {
    stop_control_loop();
    comms_.disconnect();
    return observed;
  }
  transition_to(HandLifecycle::Connected);
  return {};
}

Status HandCore::disconnect()
{
  if (Status allowed = check_allowed(HandAction::Disconnect); !allowed.ok()) {
    return allowed;
  }
  if (lifecycle_.load() == HandLifecycle::Disconnected) {
    log_info(hand_side_, "disconnect() ignored — already disconnected");
    return {};
  }

  // set_command() only lands in Running, so leaving it voids what was staged. Clearing on the way
  // out spares every way back in its own reset, and holds even if the stop below is not confirmed
  reset_command_to_idle();

  const HandLifecycle control = requested_lifecycle_.load();
  const bool control_requested =
      control == HandLifecycle::Running || control == HandLifecycle::Stopped;
  if (control_requested && lifecycle_.load() != HandLifecycle::Stopped) {
    // The drives hold the last command when the link drops, so confirm the stop before cutting TX
    requested_lifecycle_.store(HandLifecycle::Stopped);
    constexpr long kDisconnectStopTimeoutNanoseconds = 500000000L;
    if (!wait_observed(HandLifecycle::Stopped, kDisconnectStopTimeoutNanoseconds)) {
      // A fault ends the wait early, so the fault cause is the honest answer
      if (lifecycle_.load() == HandLifecycle::Faulted) {
        if (Status faulted = check_allowed(HandAction::Disconnect); !faulted.ok()) return faulted;
      }
      if (!check_communication_.load()) {
        return {ErrorCode::CommunicationLost,
                "Cannot disconnect hand: cannot confirm stop — no RX on " + interface_name_ +
                    "; drives may hold last command. Use reconnect() or destroy"};
      }
      return {ErrorCode::HardwareFault,
              "Cannot disconnect hand: drives did not reach quick stop within 500 ms — power off the "
              "hand, or destroy the hand to disconnect anyway"};
    }
  }
  stop_control_loop();
  (void)comms_.stop();
  comms_.disconnect();
  check_first_frame_.store(false);
  check_communication_.store(false);
  transition_to(HandLifecycle::Disconnected);
  return {};
}

Status HandCore::reconnect()
{
  if (Status allowed = check_allowed(HandAction::Reconnect); !allowed.ok()) {
    return allowed;
  }
  stop_control_loop();
  (void)comms_.stop();
  comms_.disconnect();

  // connect() does not touch the stop verdict or the homing channels
  const StopCause stop_cause_before_reset = last_stop_cause_;
  reset_fault_state();

  // So the command from before the fault cannot move the hand on recovery, as auto-reconnect does
  // enable_control() only clears it when coming from Stopped, and this path ends in Connected
  reset_command_to_idle();

  const Status connected = connect();
  if (!connected.ok()) {
    // Stay Faulted so the retry is again reconnect(), and keep the cause that put it there:
    // a default StopCause renders as "no RX since cycle 0", which never happened
    last_stop_cause_ = stop_cause_before_reset;
    lifecycle_.store(HandLifecycle::Faulted);
  }
  return connected;
}

void HandCore::close()
{
  if (request_destroy_.exchange(true)) {
    return;
  }
  const HandLifecycle control = requested_lifecycle_.load();
  const bool control_requested =
      control == HandLifecycle::Running || control == HandLifecycle::Stopped;
  if (control_requested && lifecycle_.load() != HandLifecycle::Stopped) {
    // The drives hold the last command when the link drops, so confirm the stop before cutting TX
    requested_lifecycle_.store(HandLifecycle::Stopped);
    constexpr long kCloseStopTimeoutNanoseconds = 500000000L;
    if (!wait_observed(HandLifecycle::Stopped, kCloseStopTimeoutNanoseconds)) {
      log_critical(hand_side_, "drives did not confirm quick stop within " +
                   std::to_string(kCloseStopTimeoutNanoseconds / 1000000L) +
                   " ms — destroying anyway; drives may hold last command; "
                   "power off the hand, or reconnect and stop again to retry the stop");
    }
  }
  stop_control_loop();

  // After the RT thread has ended, so the logging thread can drain what is left
  event_logging_running_.store(false);
  if (event_logging_thread_.joinable()) {
    event_logging_thread_.join();
  }
  (void)comms_.stop();
  comms_.disconnect();

  // Only a session that actually observed something
  if (check_first_frame_.load()) {
    log_info(hand_side_, "hand destroyed");
  }
}

// -------------------------------- Operation ---------------------------------

Status HandCore::run()
{
  if (Status allowed = check_allowed(HandAction::Run); !allowed.ok()) {
    return allowed;
  }
  // auto_home makes run() home first, and home() completes enable, homing and Running on its own
  // Without it the hand runs unhomed, and set_command stays refused until home() succeeds
  if (auto_home_ && homing_state_.load() != HomingState::Succeeded) {
    return home();
  }
  if (requested_lifecycle_.load() == HandLifecycle::Running &&
      lifecycle_.load() == HandLifecycle::Running) {
    log_info(hand_side_, "run() ignored — already running");
    return {};
  }
  return enable_control();
}

Status HandCore::stop()
{
  if (Status allowed = check_allowed(HandAction::Stop); !allowed.ok()) {
    return allowed;
  }
  // Only a confirmed stop is a no-op, so a retry after an unconfirmed one waits again
  if (requested_lifecycle_.load() == HandLifecycle::Stopped &&
      lifecycle_.load() == HandLifecycle::Stopped) {
    log_info(hand_side_, "stop() ignored — already stopped");
    return {};
  }
  requested_lifecycle_.store(HandLifecycle::Stopped);
  reset_command_to_idle();
  constexpr long kStopTimeoutNanoseconds = 500000000L;
  if (!wait_observed(HandLifecycle::Stopped, kStopTimeoutNanoseconds)) {
    // A fault ends the wait early, so the fault cause is the honest answer
    if (lifecycle_.load() == HandLifecycle::Faulted) {
      if (Status faulted = check_allowed(HandAction::Stop); !faulted.ok()) return faulted;
    }
    // No RX means the stop could not be confirmed, a live link means the drives did not reach it
    if (!check_communication_.load()) {
      return {ErrorCode::CommunicationLost,
              "Cannot stop hand: cannot confirm quick stop — no RX on " + interface_name_ +
                  "; drives may hold last command. Restore CAN link and hand power"};
    }
    return {ErrorCode::HardwareFault,
            "Cannot stop hand: drives did not reach quick stop within " +
                std::to_string(kStopTimeoutNanoseconds / 1000000L) + " ms — retry stop() or power off the hand"};
  }
  log_info(hand_side_, "stop() succeeded — control stopped");
  return {};
}

Status HandCore::home()
{
  if (Status started = start_homing(); !started.ok()) {
    return started;
  }
  while (requested_lifecycle_.load() == HandLifecycle::Running &&
         lifecycle_.load() != HandLifecycle::Faulted && is_homing()) {
    timespec poll{0, 1000000L};
    nanosleep(&poll, nullptr);
  }
  if (requested_lifecycle_.load() != HandLifecycle::Running ||
      lifecycle_.load() == HandLifecycle::Faulted) {
    // Re-run the table so the message matches why it stopped
    if (Status allowed = check_allowed(HandAction::Home); !allowed.ok()) {
      return allowed;
    }
    return {ErrorCode::WrongCallOrder,
            "Cannot home hand: interrupted during homing — call run() and home() again"};
  }
  if (result_homing_.load()) {
    // The RT completion point already stored the homing state and logged it
    return {};
  }

  // CiA402 wording stays in the debug log, the returned message uses public vocabulary
  log_debug(hand_side_, homing_debug_detail(comms_.last_homing_failure_stage(),
                                            comms_.last_homing_failed_actuators(),
                                            comms_.last_homing_status_snapshot(),
                                            comms_.last_homing_position_snapshot()));
  const std::string reason = homing_failure_reason(comms_.last_homing_outcome());
  const std::string actuators = failed_actuator_list(comms_.last_homing_failed_actuators());
  return {ErrorCode::HardwareFault,
          "Cannot home hand: " + reason + " (actuators " + actuators +
              ") — retry home(), or check actuator faults via diagnostics()"};
}

Status HandCore::start_homing()
{
  if (Status allowed = check_allowed(HandAction::Home); !allowed.ok()) {
    return allowed;
  }
  homing_state_.store(HomingState::InProgress);

  // Homing moves the hand, so the previous command is void
  reset_command_to_idle();

  // Homing is active motion, so control has to be requested first. The sequence enables the
  // drives itself, so this does not wait for them
  if (Status requested = request_control(); !requested.ok()) {
    return requested;
  }

  // comms_ belongs to the RT thread, so the sequence is requested rather than called here
  log_info(hand_side_, "homing started");
  result_homing_.store(false);
  request_homing_.store(true);
  return {};
}

bool HandCore::is_homing() const
{
  return request_homing_.load() || active_homing_.load();
}

// --------------------------------- Command ----------------------------------

Status HandCore::set_command(const Idle& command)
{
  if (Status allowed = check_allowed(HandAction::SetCommand, false); !allowed.ok()) return allowed;
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_command_.controller = command;
  command_buffer_.write(staging_command_);
  return {};
}

Status HandCore::set_command(const JointPositionCommand& command)
{
  if (Status allowed = check_allowed(HandAction::SetCommand); !allowed.ok()) return allowed;
  if (!is_command_valid(command)) {
    return drop_invalid_command(hand_side_, count_nan_command_, last_command_was_nan_);
  }
  last_command_was_nan_.store(false, std::memory_order_relaxed);

  // Pull the target into the reachable workspace before storing
  JointPositionCommand clamped_command = command;
  if (workspace_clamp_) clamped_command.clamp();
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_command_.controller = clamped_command;
  command_buffer_.write(staging_command_);
  return {};
}

Status HandCore::set_command(const JointImpedanceCommand& command)
{
  if (Status allowed = check_allowed(HandAction::SetCommand); !allowed.ok()) return allowed;
  if (!is_command_valid(command)) {
    return drop_invalid_command(hand_side_, count_nan_command_, last_command_was_nan_);
  }
  last_command_was_nan_.store(false, std::memory_order_relaxed);

  JointImpedanceCommand clamped_command = command;
  if (workspace_clamp_) clamped_command.clamp();
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_command_.controller = clamped_command;
  command_buffer_.write(staging_command_);
  return {};
}

Status HandCore::set_command(const ActuatorPositionCommand& command)
{
  if (Status allowed = check_allowed(HandAction::SetCommand); !allowed.ok()) return allowed;
  if (!is_command_valid(command)) {
    return drop_invalid_command(hand_side_, count_nan_command_, last_command_was_nan_);
  }
  last_command_was_nan_.store(false, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_command_.controller = command;
  command_buffer_.write(staging_command_);
  return {};
}

Status HandCore::set_command(const ActuatorEffortCommand& command)
{
  if (Status allowed = check_allowed(HandAction::SetCommand); !allowed.ok()) return allowed;
  if (!is_command_valid(command)) {
    return drop_invalid_command(hand_side_, count_nan_command_, last_command_was_nan_);
  }
  last_command_was_nan_.store(false, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_command_.controller = command;
  command_buffer_.write(staging_command_);
  return {};
}

// ---------------------------------- Config ----------------------------------

Status HandCore::set_max_effort(const std::array<double, kActuatorCount>& limit)
{
  if (request_destroy_.load()) {
    return {ErrorCode::WrongCallOrder, "Cannot set max effort: hand is destroyed — create a hand"};
  }
  if (!all_finite(limit)) {
    return {ErrorCode::InvalidArgument, "Cannot set max effort: limit contains NaN/Inf"};
  }

  // Bound here so everything downstream only ever sees [0, kMaxEffortLimit]
  std::array<double, kActuatorCount> clamped = limit;
  for (double& value : clamped) value = std::clamp(value, 0.0, kMaxEffortLimit);
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_command_.max_effort_pct = clamped;
  command_buffer_.write(staging_command_);
  return {};
}

Status HandCore::set_controller_config(const ControllerConfig& config)
{
  if (request_destroy_.load()) {
    return {ErrorCode::WrongCallOrder, "Cannot set controller config: hand is destroyed — create a hand"};
  }
  const ControllerConfig::JointPositionController& position = config.joint_position_controller;
  if (!std::isfinite(position.cutoff_freq) || position.cutoff_freq < 0.0 ||
      !std::isfinite(position.deadband) || position.deadband < 0.0) {
    return {ErrorCode::InvalidArgument,
            "Cannot set controller config: joint position cutoff_freq and deadband must be finite and >= 0"};
  }
  const ControllerConfig::JointImpedanceController& impedance = config.joint_impedance_controller;
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (!std::isfinite(impedance.stiffness[i]) || impedance.stiffness[i] < 0.0 ||
        !std::isfinite(impedance.damping[i]) || impedance.damping[i] < 0.0) {
      return {ErrorCode::InvalidArgument,
              "Cannot set controller config: joint impedance stiffness and damping must be finite and >= 0"};
    }
  }
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_controller_config_ = config;
  controller_config_buffer_.write(config);
  return {};
}

// ------------------------------- Observation --------------------------------

HandState HandCore::state() const
{
  if (request_destroy_.load()) {
    throw_error(hand_side_, ErrorCode::WrongCallOrder, "Cannot read state: hand is destroyed — create a hand");
  }
  HandState latest{};
  if (state_buffer_.read(latest)) {
    last_state_ = latest;
  }
  return last_state_;
}

Diagnostics HandCore::diagnostics() const
{
  if (request_destroy_.load()) {
    throw_error(hand_side_, ErrorCode::WrongCallOrder, "Cannot read diagnostics: hand is destroyed — create a hand");
  }
  Diagnostics latest{};
  if (diagnostics_buffer_.read(latest)) last_diagnostics_ = latest;
  Diagnostics diagnostics = last_diagnostics_;
  diagnostics.lifecycle = lifecycle_.load();
  diagnostics.homing_state = homing_state_.load();
  diagnostics.nan_command_count = count_nan_command_.load(std::memory_order_relaxed);
  return diagnostics;
}

CommandMode HandCore::command_mode() const
{
  std::lock_guard<std::mutex> lock(staging_mutex_);
  return to_command_mode(staging_command_.controller);
}

// ----------------------------- State transition -----------------------------

// Load the observed value first, so last_stop_cause_ is read only after Faulted has been seen
Status HandCore::check_allowed(HandAction action, bool require_homed) const
{
  const HandLifecycle observed = lifecycle_.load();
  const HandLifecycle requested = requested_lifecycle_.load();
  const bool homed = require_homed ? homing_state_.load() == HomingState::Succeeded : true;
  return check_action_allowed(action, requested, observed, request_destroy_.load(),
                              last_stop_cause_, homed);
}

Status HandCore::request_control()
{
  const HandLifecycle requested = requested_lifecycle_.load();
  if (requested == HandLifecycle::Running && lifecycle_.load() == HandLifecycle::Running) {
    return {};
  }
  // A gone or re-created interface cannot resume receiving on this socket
  if (!check_communication_.load()) {
    const unsigned int current_interface_index = if_nametoindex(interface_name_.c_str());
    if (current_interface_index == 0) {
      return {ErrorCode::CommunicationLost,
              "Cannot enable hand: interface " + interface_name_ +
                  " is gone — reconnect the CAN adapter, then recover with reconnect()"};
    }
    if (static_cast<int>(current_interface_index) != comms_.bound_interface_index()) {
      return {ErrorCode::CommunicationLost,
              "Cannot enable hand: interface " + interface_name_ +
                  " was re-created — recover with reconnect() once the hand faults"};
    }
    return {ErrorCode::CommunicationLost,
            "Cannot enable hand: communication not restored (no RX on " + interface_name_ +
                ") — restore CAN link and hand power, then recover with reconnect()"};
  }
  if (requested == HandLifecycle::Stopped) {
    // Idle first, so the worst case is torque-free until the RT loop fills the hold
    reset_command_to_idle();
    request_hold_.store(true);
  }
  requested_lifecycle_.store(HandLifecycle::Running);
  return {};
}

Status HandCore::enable_control()
{
  if (Status requested = request_control(); !requested.ok()) {
    return requested;
  }
  // The same budget the homing enable stage uses, because it is the same drive handshake
  constexpr long kEnableTimeoutNanoseconds = 4000000000L;
  if (!wait_observed(HandLifecycle::Running, kEnableTimeoutNanoseconds)) {
    // Some actuators may have come up, so the hand is asked to stop instead of being left with the
    // staged command on the wire. Requesting Connected would cut TX and leave whatever came up
    // enabled, with no quick stop to bring it down
    requested_lifecycle_.store(HandLifecycle::Stopped);
    reset_command_to_idle();

    // No RX means the enable could not be confirmed, a live link means the drives did not follow
    if (!check_communication_.load()) {
      return {ErrorCode::CommunicationLost,
              "Cannot run hand: cannot confirm drive enable — no RX on " + interface_name_ +
                  "; restore CAN link and hand power, then reconnect()"};
    }
    return {ErrorCode::HardwareFault,
            "Cannot run hand: drives did not reach operation enabled within " +
                std::to_string(kEnableTimeoutNanoseconds / 1000000L) +
                " ms — retry run(), or check actuator faults via diagnostics()"};
  }
  transition_to(HandLifecycle::Running);
  return {};
}

// Pairs a confirmed transition with its "<api>() succeeded" log
// stop() logs after its own confirmation, and Faulted is logged from the event ring
void HandCore::transition_to(HandLifecycle next)
{
  lifecycle_.store(next);
  switch (next) {
    case HandLifecycle::Connected:
      log_info(hand_side_, "connect() succeeded — " + interface_name_ + " at " +
               std::to_string(1000000000L / period_nanoseconds_) + " Hz, control not running");
      break;
    case HandLifecycle::Running:
      log_info(hand_side_, "run() succeeded — control active");
      break;
    case HandLifecycle::Disconnected:
      log_info(hand_side_, "disconnect() succeeded");
      break;
    case HandLifecycle::Stopped:
    case HandLifecycle::Faulted:
      break;
  }
}

// A fault ends the wait early, because the loop will not reach the target after one
bool HandCore::wait_observed(HandLifecycle target, long timeout_nanoseconds)
{
  const long deadline = now_nanoseconds() + timeout_nanoseconds;
  while (lifecycle_.load() != HandLifecycle::Faulted && now_nanoseconds() < deadline) {
    if (lifecycle_.load() == target) return true;
    timespec poll{0, 1000000L};
    nanosleep(&poll, nullptr);
  }
  return lifecycle_.load() == target;
}

// The drives send state frames without being enabled, so silence means the hand is not responding
Status HandCore::wait_first_frame(const char* operation)
{
  constexpr long kFirstFrameTimeoutNanoseconds = 300000000L;
  const long first_frame_deadline = now_nanoseconds() + kFirstFrameTimeoutNanoseconds;
  while (!check_first_frame_.load() && now_nanoseconds() < first_frame_deadline) {
    timespec poll{0, 1000000L};
    nanosleep(&poll, nullptr);
  }
  if (!check_first_frame_.load()) {
    return {ErrorCode::CommunicationLost,
            std::string("Cannot ") + operation + ": no data received from " + interface_name_ +
                " within 300 ms — check hand power and CAN wiring"};
  }
  return {};
}

// Requesting Disconnected is the loop's exit signal, and the closed socket makes it a fact
void HandCore::stop_control_loop()
{
  requested_lifecycle_.store(HandLifecycle::Disconnected);
  if (rt_thread_.joinable()) {
    rt_thread_.join();
  }
  lifecycle_.store(HandLifecycle::Disconnected);
}

void HandCore::reset_fault_state()
{
  stop_confirmation_.store(StopConfirmation::Waiting);
  last_stop_cause_ = {};
  request_homing_.store(false);
  active_homing_.store(false);
  request_hold_.store(false);
}

// ------------------------------ Command write -------------------------------

// A RealtimeBuffer hands over a value once, and the loop that consumed the last one is gone with
// its thread. A newly started loop begins from a default HandCommand, whose effort cap is 0, and
// from a default ControllerConfig, so connect() puts the staged state back on the wire
void HandCore::republish_staged()
{
  std::lock_guard<std::mutex> lock(staging_mutex_);
  command_buffer_.write(staging_command_);
  controller_config_buffer_.write(staging_controller_config_);
}

void HandCore::reset_command_to_idle()
{
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_command_.controller = Idle{};
  command_buffer_.write(staging_command_);
}

// The home offset at the encoder boundary puts 0 off the hard stop, so the hand backs out of the
// push pose and holds there
void HandCore::set_command_to_home_hold()
{
  ActuatorPositionCommand hold{};
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_command_.controller = hold;
  command_buffer_.write(staging_command_);
}

void HandCore::set_command_to_position_hold(const HandState& hand_state)
{
  ActuatorPositionCommand hold{};
  hold.target = hand_state.actuators.position_count;
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_command_.controller = hold;
  command_buffer_.write(staging_command_);
}

// --------------------------------- Helpers ----------------------------------

namespace
{

bool workspace_clamp_enabled()
{
  const char* value = std::getenv("AIDIN_HAND2_WORKSPACE_CLAMP");
  if (value == nullptr || std::string(value) != "0") return true;
  log_warn("workspace clamp DISABLED by AIDIN_HAND2_WORKSPACE_CLAMP=0 "
           "— targets bypass the reachable-workspace limits (limit measurement only)");
  return false;
}

template <std::size_t Size>
bool all_finite(const std::array<double, Size>& values)
{
  for (const double value : values) if (!std::isfinite(value)) return false;
  return true;
}

bool is_command_valid(const JointPositionCommand& command)
{
  return all_finite(command.target);
}

bool is_command_valid(const JointImpedanceCommand& command)
{
  return all_finite(command.target);
}

bool is_command_valid(const ActuatorPositionCommand& command)
{
  constexpr double kMin = static_cast<double>(std::numeric_limits<std::int32_t>::min());
  constexpr double kMax = static_cast<double>(std::numeric_limits<std::int32_t>::max());
  if (!all_finite(command.target)) return false;
  for (const double target : command.target) if (target < kMin || target > kMax) return false;
  return true;
}

bool is_command_valid(const ActuatorEffortCommand& command)
{
  return all_finite(command.target);
}

Status drop_invalid_command(HandSide side, std::atomic<std::uint64_t>& count,
                            std::atomic<bool>& rejection_active)
{
  count.fetch_add(1, std::memory_order_relaxed);
  if (!rejection_active.exchange(true)) {
    log_warn(side, "set_command: command contains an invalid value — holding previous command; "
                   "watch nan_command_count");
  }
  return {};
}

std::string failed_actuator_list(const std::array<bool, kActuatorCount>& failed)
{
  std::string list;
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index) {
    if (!failed[actuator_index]) continue;
    if (!list.empty()) list += ",";
    list += std::to_string(actuator_index);
  }
  return list.empty() ? "none" : list;
}

std::string homing_debug_detail(const char* stage,
                                const std::array<bool, kActuatorCount>& failed,
                                const std::array<std::uint16_t, kActuatorCount>& status,
                                const std::array<std::int32_t, kActuatorCount>& position)
{
  std::string detail = std::string("homing failed at stage ") + stage + ":";
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index) {
    if (!failed[actuator_index]) continue;
    detail += " " + std::to_string(actuator_index) + "={status " + to_hex16(status[actuator_index]) +
              ", pos " + std::to_string(position[actuator_index]) + "}";
  }
  return detail;
}

}  // namespace

}  // namespace aidin_hand2
