#include "hand_core/hand_core.hpp"

#include <time.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <system_error>

#include <aidin_hand2/version.hpp>

#include "hand_core/lifecycle/action_check.hpp"
#include "hand_core/model/model_composition.hpp"
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

// count, where every actuator holds after homing, off the hard stop at 0
constexpr double kHomeHoldCount = 1000.0;

// Characters, the rules and the key column of the Connection Info block
constexpr std::size_t kInfoRuleWidth = 68;
constexpr std::size_t kInfoKeyWidth  = 12;

// Off only under AIDIN_HAND2_WORKSPACE_CLAMP=0, for measuring the reachable limits
bool workspace_clamp_enabled(HandSide side);

// A non-finite target would reach a static_cast below, which is undefined behaviour
template <std::size_t Size>
bool all_finite(const std::array<double, Size>& values);

// Joint stiffness in [0, 1] and cutoff_freq >= 0, all finite
Status check_joint_impedance(const ControllerConfig::JointImpedanceController& config);

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
: HandCore(config, composition_of(config.hand_model))
{
}

HandCore::HandCore(const HandConfig& config, const ModelComposition& composition)
: comms_(config),
  rt_cpu_affinity_(config.rt_cpu_affinity),
  auto_reconnect_(config.auto_reconnect),
  auto_reconnect_timeout_ms_(config.auto_reconnect_timeout_ms),
  auto_reconnect_home_(config.auto_reconnect_home),
  auto_home_(config.auto_home),
  workspace_clamp_(workspace_clamp_enabled(config.hand_side)),
  interface_name_(config.interface_name),
  hand_side_(config.hand_side),
  hand_model_(config.hand_model),
  solver_(composition.solver),
  actuator_(composition.actuator),
  joint_position_controller_(kPeriodNanoseconds, composition.solver),
  joint_impedance_controller_(kPeriodNanoseconds, composition.solver, composition.hard_stiffness,
                              composition.actuator)
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
  // The CAN layer names the interface and the failure, so the call goes in front here
  const Status activated = comms_.activate();
  if (!activated.ok()) {
    return {activated.code, "cannot connect hand: " + activated.message};
  }

  // "auto" may have resolved to a real channel
  interface_name_ = comms_.interface_name();

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
    comms_.deactivate();
    return {error_code::kThreadStartFailed,
            std::string("cannot connect hand: failed to start SDK threads (") + thread_failure.what() + ")"};
  }

  // Succeed only once observation has started, so state() never returns a blank snapshot
  if (const Status observed = wait_first_frame("connect hand"); !observed.ok()) {
    stop_control_loop();
    comms_.deactivate();
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
        return {error_code::kQuickStopUnconfirmed,
                "cannot disconnect hand: quick stop not confirmed (CAN communication failed)"};
      }
      return {error_code::kQuickStopTimeout,
              "cannot disconnect hand: drives did not reach quick stop within 500 ms"};
    }
  }
  stop_control_loop();
  comms_.deactivate();
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
  comms_.deactivate();

  // connect() does not touch the stop verdict or the homing channels
  const StopCause stop_cause_before_reset = last_stop_cause_;
  reset_fault_state();

  // So the command from before the fault cannot move the hand on recovery, as auto-reconnect does
  // enable_control() only clears it when coming from Stopped, and this path ends in Connected
  reset_command_to_idle();

  const Status connected = connect();
  if (!connected.ok()) {
    // Stay Faulted so the retry is again reconnect(), and keep the cause that put it there:
    // a default StopCause renders as "E404 communication lost", which never happened
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
  // A faulted hand is not sent anything, and its fault was logged when it happened
  const HandLifecycle observed = lifecycle_.load();
  if (control_requested && observed != HandLifecycle::Stopped && observed != HandLifecycle::Faulted) {
    // The drives hold the last command when the link drops, so confirm the stop before cutting TX
    requested_lifecycle_.store(HandLifecycle::Stopped);
    constexpr long kCloseStopTimeoutNanoseconds = 500000000L;
    if (!wait_observed(HandLifecycle::Stopped, kCloseStopTimeoutNanoseconds) &&
        lifecycle_.load() != HandLifecycle::Faulted) {
      if (!check_communication_.load()) {
        // The same words stop() throws, as destroy() stops the hand first and has no caller to throw to
        log_critical(hand_side_, error_code::kQuickStopUnconfirmed,
                     "cannot stop hand: quick stop not confirmed (CAN communication failed)");
      } else {
        log_critical(hand_side_, error_code::kQuickStopTimeout,
                     "cannot stop hand: drives did not reach quick stop within " +
                         std::to_string(kCloseStopTimeoutNanoseconds / 1000000L) + " ms");
      }
    }
  }
  stop_control_loop();

  // After the RT thread has ended, so the logging thread can drain what is left
  event_logging_running_.store(false);
  if (event_logging_thread_.joinable()) {
    event_logging_thread_.join();
  }
  comms_.deactivate();

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
    // A failing link means the stop could not be confirmed, a live link means the drives did not
    // reach it. A lost RX faults within the wait, so a failing link here is a failing TX
    if (!check_communication_.load()) {
      return {error_code::kQuickStopUnconfirmed,
              "cannot stop hand: quick stop not confirmed (CAN communication failed)"};
    }
    return {error_code::kQuickStopTimeout,
            "cannot stop hand: drives did not reach quick stop within " +
                std::to_string(kStopTimeoutNanoseconds / 1000000L) + " ms"};
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
    return {error_code::kHomingInterrupted, "cannot home hand: interrupted during homing"};
  }
  if (result_homing_.load()) {
    // The RT completion point already stored the homing state and logged it
    return {};
  }

  // CiA402 wording stays in the debug log, and the step and reason in the RT log of the failure
  log_debug(hand_side_, homing_debug_detail(comms_.last_homing_failure_stage(),
                                            comms_.last_homing_failed_actuators(),
                                            comms_.last_homing_status_snapshot(),
                                            comms_.last_homing_position_snapshot()));
  const std::string actuators = failed_actuator_list(comms_.last_homing_failed_actuators());
  return {error_code::kHomingFailed, "cannot home hand: homing failed (actuators " + actuators + ")"};
}

Status HandCore::start_homing()
{
  if (Status allowed = check_allowed(HandAction::Home); !allowed.ok()) {
    return allowed;
  }
  const HomingState previous_homing_state = homing_state_.exchange(HomingState::InProgress);

  // Homing moves the hand, so the previous command is void
  reset_command_to_idle();

  // Homing is active motion, so control has to be requested first. The sequence enables the
  // drives itself, so this does not wait for them
  if (Status requested = request_control("home hand"); !requested.ok()) {
    homing_state_.store(previous_homing_state);
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
    return hand_destroyed();
  }
  if (!all_finite(limit)) {
    return {error_code::kMaxEffortInvalid, "cannot set max effort: limit contains NaN/Inf"};
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
    return hand_destroyed();
  }
  const ControllerConfig::JointPositionController& position = config.joint_position_controller;
  if (!std::isfinite(position.cutoff_freq) || position.cutoff_freq < 0.0 ||
      !std::isfinite(position.deadband) || position.deadband < 0.0) {
    return {error_code::kPositionControllerInvalid,
            "cannot set controller config: joint position cutoff_freq and deadband must be finite and >= 0"};
  }
  const ControllerConfig::JointImpedanceController& impedance = config.joint_impedance_controller;
  if (Status joint = check_joint_impedance(impedance); !joint.ok()) return joint;
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_controller_config_ = config;
  controller_config_buffer_.write(config);
  return {};
}

// --------------------------------- Tactile ----------------------------------

Status HandCore::set_tactile_bias()
{
  return request_tactile_bias(TactileBiasRequest::Set, HandAction::SetTactileBias);
}

Status HandCore::reset_tactile_bias()
{
  return request_tactile_bias(TactileBiasRequest::Reset, HandAction::ResetTactileBias);
}

// ------------------------------- Observation --------------------------------

HandState HandCore::state() const
{
  if (request_destroy_.load()) {
    throw_if_error(hand_side_, hand_destroyed());
  }
  HandState latest{};
  if (state_buffer_.read(latest)) {
    last_state_ = latest;
  }
  return last_state_;
}

TactileState HandCore::tactile_bias() const
{
  if (request_destroy_.load()) {
    throw_if_error(hand_side_, hand_destroyed());
  }
  TactileState latest{};
  if (tactile_bias_buffer_.read(latest)) last_tactile_bias_ = latest;
  return last_tactile_bias_;
}

Diagnostics HandCore::diagnostics() const
{
  if (request_destroy_.load()) {
    throw_if_error(hand_side_, hand_destroyed());
  }
  Diagnostics latest{};
  if (diagnostics_buffer_.read(latest)) last_diagnostics_ = latest;
  Diagnostics diagnostics = last_diagnostics_;
  // Loaded first, so last_stop_cause_ is read only after Faulted has been seen
  diagnostics.lifecycle = lifecycle_.load();
  diagnostics.fault_code =
      diagnostics.lifecycle == HandLifecycle::Faulted ? stop_cause_code(last_stop_cause_) : 0;
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

Status HandCore::request_control(const char* operation)
{
  const HandLifecycle requested = requested_lifecycle_.load();
  if (requested == HandLifecycle::Running && lifecycle_.load() == HandLifecycle::Running) {
    return {};
  }
  // Enabling drives that cannot be observed or reached would leave them uncontrolled
  if (!check_communication_.load()) {
    return {error_code::kLinkFailed, std::string("cannot ") + operation + ": CAN communication failed"};
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
  if (Status requested = request_control("run hand"); !requested.ok()) {
    return requested;
  }
  // The same budget the homing enable stage uses, because it is the same drive handshake
  constexpr long kEnableTimeoutNanoseconds = 4000000000L;
  if (!wait_observed(HandLifecycle::Running, kEnableTimeoutNanoseconds)) {
    // A fault ends the wait early, so the fault cause is the honest answer
    if (lifecycle_.load() == HandLifecycle::Faulted) {
      if (Status faulted = check_allowed(HandAction::Run); !faulted.ok()) return faulted;
    }

    // Some actuators may have come up, so the hand is asked to stop instead of being left with the
    // staged command on the wire. Requesting Connected would cut TX and leave whatever came up
    // enabled, with no quick stop to bring it down
    requested_lifecycle_.store(HandLifecycle::Stopped);
    reset_command_to_idle();

    // A failing link means the enable could not be confirmed, a live link means the drives did not
    // follow
    if (!check_communication_.load()) {
      return {error_code::kLinkFailed, "cannot run hand: CAN communication failed"};
    }
    return {error_code::kEnableTimeout,
            "cannot run hand: drives did not reach operation enabled within " +
                std::to_string(kEnableTimeoutNanoseconds / 1000000L) + " ms"};
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
    case HandLifecycle::Connected: {
      // A Connection Info block between two rules, under the succeeded line and from the first column
      const auto row = [](std::string key, const std::string& value) {
        key.resize(kInfoKeyWidth, ' ');
        return key + value + "\n";
      };
      const std::string title  = " Connection Info ";
      const std::size_t dashes = kInfoRuleWidth - title.size();
      std::size_t faulted = 0;
      for (const ActuatorFault fault : diagnostics().actuator_health.fault) {
        if (fault != ActuatorFault::None) ++faulted;
      }
      const RealtimeReport& realtime = realtime_report_;

      std::string text = "connect() succeeded\n" + std::string(dashes - dashes / 2, '-') + title +
                         std::string(dashes / 2, '-') + "\n";
      text += row("side", hand_side_ == HandSide::Left ? "left" : "right");
      text += row("model", to_string(hand_model_));
      text += row("interface", interface_name_);
      text += row("rate", std::to_string(1000000000L / kPeriodNanoseconds) + " Hz");
      text += row("auto_home", auto_home_ ? "on" : "off");
      text += row("scheduler", realtime.fifo_priority > 0 ? "SCHED_FIFO " + std::to_string(realtime.fifo_priority)
                                                          : std::string("SCHED_OTHER"));
      text += row("cpu", std::to_string(realtime.running_cpu));
      text += row("affinity", rt_cpu_affinity_ >= 0 ? std::to_string(rt_cpu_affinity_) : std::string("none"));
      text += row("memory", realtime.memory_locked ? "locked" : "not locked");
      text += row("actuators", std::to_string(kActuatorCount));
      text += row("disabled", std::to_string(comms_.disabled_actuator_count()));
      text += row("faulted", std::to_string(faulted));
      text += row("sdk", kVersionString);
      text += std::string(kInfoRuleWidth, '-');
      log_info(hand_side_, text);
      break;
    }
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
    return {error_code::kNoFirstFrame,
            std::string("cannot ") + operation + ": no data received from " + interface_name_ +
                " within 300 ms"};
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

// The RT loop logs the cycle it takes the request in, so nothing is logged here
Status HandCore::request_tactile_bias(TactileBiasRequest request, HandAction action)
{
  if (Status allowed = check_allowed(action); !allowed.ok()) return allowed;

  // Without a live link the reading is the last frame before the link dropped
  if (request == TactileBiasRequest::Set && !check_communication_.load()) {
    return {error_code::kLinkFailed, "cannot set tactile bias: CAN communication failed"};
  }
  request_tactile_bias_.store(request);
  return {};
}

void HandCore::reset_fault_state()
{
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

// The hand backs out of the push pose to kHomeHoldCount and holds there, off the stop at count 0
void HandCore::set_command_to_home_hold()
{
  ActuatorPositionCommand hold{};
  hold.target.fill(kHomeHoldCount);
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_command_.controller = hold;
  command_buffer_.write(staging_command_);
}

void HandCore::set_command_to_position_hold(const HandState& hand_state)
{
  ActuatorPositionCommand hold{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    hold.target[i] = hand_state.actuators.position_count[i];
  }
  std::lock_guard<std::mutex> lock(staging_mutex_);
  staging_command_.controller = hold;
  command_buffer_.write(staging_command_);
}

// --------------------------------- Helpers ----------------------------------

namespace
{

bool workspace_clamp_enabled(HandSide side)
{
  const char* value = std::getenv("AIDIN_HAND2_WORKSPACE_CLAMP");
  if (value == nullptr || std::string(value) != "0") return true;
  log_warn(side, "workspace clamp DISABLED by AIDIN_HAND2_WORKSPACE_CLAMP=0 "
                 "— targets bypass the reachable-workspace limits (limit measurement only)");
  return false;
}

template <std::size_t Size>
bool all_finite(const std::array<double, Size>& values)
{
  for (const double value : values) if (!std::isfinite(value)) return false;
  return true;
}

Status check_joint_impedance(const ControllerConfig::JointImpedanceController& config)
{
  const auto fail = [](std::uint16_t code, const char* what) {
    return Status{code, std::string("cannot set controller config: joint impedance ") + what};
  };
  for (const double stiffness : config.stiffness) {
    if (!std::isfinite(stiffness) || stiffness < 0.0 || stiffness > 1.0) {
      return fail(error_code::kImpedanceStiffnessInvalid, "stiffness must be finite and in [0, 1]");
    }
  }
  if (!std::isfinite(config.cutoff_freq) || config.cutoff_freq < 0.0) {
    return fail(error_code::kImpedanceCutoffInvalid, "cutoff_freq must be finite and >= 0");
  }
  return {};
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
    log_warn(side, "set_command: command with an invalid value dropped, previous command kept");
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
