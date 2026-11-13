#include "hand_core/hand_core.hpp"

#include <link.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <string>
#include <variant>

#include "hand_core/kinematics/kinematics.hpp"
#include "hand_core/util/monotonic_clock.hpp"
#include "logging/log.hpp"

namespace aidin_hand2
{

namespace
{

// Silence longer than this is a lost link, converted to cycles when the loop starts
constexpr long kCommunicationLossTimeoutMs = 100;

// Below the kernel critical threads at 99, above the PREEMPT_RT IRQ threads at 50
constexpr int kThreadPriority = 90;

// Stack kept in RAM below the loop's frame, far more than the loop uses
constexpr std::uintptr_t kLockedStackBytes = 512 * 1024;

// Lowest stack address to lock for the calling thread: kLockedStackBytes below `top`, above the
// guard page
std::uintptr_t locked_stack_floor(std::uintptr_t top);

// Locks the loaded segments (code, constants, static data) of the object holding `address` for
// the rest of the process; errno of the first failure, 0 when all are locked
int lock_segments_holding(const void* address);

// Fills HandState.commanded with the controller input and the setpoint it produced
void fill_commanded(CommandedState& echo, const HandCommand& command,
                    const canfd::CommandFrames& frames, bool output_available);

// Joint index of an active joint: thumb j0..j3, then j1..j3 of each long finger, skipping each j4
std::size_t joint_of_active(std::size_t active);

}  // namespace

// ------------------------------- Control loop -------------------------------

void HandCore::run_control_loop()
{
  // This frame and the ones below it stay in RAM while the loop runs
  const auto stack_top = reinterpret_cast<std::uintptr_t>(__builtin_frame_address(0));
  const std::uintptr_t stack_floor = locked_stack_floor(stack_top);
  apply_realtime_scheduling(stack_floor, stack_top);

  // Outside the try, so the failure handler can record the cycle it died on
  std::uint64_t cycle_count = 0;
  try {

  LoopState loop{};
  
  const long communication_loss_cycles =
      std::max<long>(1, kCommunicationLossTimeoutMs * 1000000L / kPeriodNanoseconds);

  long next_wakeup = now_nanoseconds();

  // ------------------------------ loop start ----------------------------------

  // The loop lives while the request is not Disconnected
  // An observed Faulted ends it too, unless auto_reconnect keeps it retrying the link
  while (true) {
    const HandLifecycle requested = requested_lifecycle_.load(std::memory_order_relaxed);
    if (requested == HandLifecycle::Disconnected) break;
    HandLifecycle observed = lifecycle_.load(std::memory_order_relaxed);
    if (observed == HandLifecycle::Faulted) {
      // An RT exception is an SDK bug, so re-establishing the link would not help
      if (!auto_reconnect_ || last_stop_cause_.reason == RealtimeEventKind::ControlLoopFailure) break;
      if (!loop.auto_reconnect_started) {
        log_info(hand_side_, "auto-reconnect started — retrying communication until it recovers");
        loop.auto_reconnect_started = true;
        if (auto_reconnect_timeout_ms_ > 0) {
          loop.auto_reconnect_deadline = now_nanoseconds() +
              static_cast<long>(auto_reconnect_timeout_ms_) * 1000000L;
        }
      }
      if (loop.auto_reconnect_deadline != 0 && now_nanoseconds() > loop.auto_reconnect_deadline) {
        log_critical(hand_side_, error_code::kAutoReconnectTimeout, "auto-reconnect timed out");
        break;
      }
      if (try_reconnect_once()) {
        // The new session starts from scratch, so the change detectors have to as well
        loop.previous_receiving = true;
        loop.first_frame_reported = false;
        loop.homing_was_active = false;
        loop.previous_fault.fill(ActuatorFault::None);
      }
      next_wakeup = now_nanoseconds();
      continue;
    }
    loop.auto_reconnect_deadline = 0;
    loop.auto_reconnect_started = false;

    const long cycle_start = now_nanoseconds();

    // Actual gap between cycle starts, 0 on the first cycle
    const double last_period_ms =
        loop.previous_cycle_start == 0 ? 0.0 : static_cast<double>(cycle_start - loop.previous_cycle_start) / 1000000.0;
    loop.previous_cycle_start = cycle_start;

    // 1) Take the input
    // Only Running reaches the wire: a latch is dominated by quick stop, Connected has no TX
    command_buffer_.read(loop.active_command);
    controller_config_buffer_.read(loop.active_controller_config);
    const HandCommand command =
        (requested == HandLifecycle::Running) ? loop.active_command : HandCommand{};

    // Outside Running the homing request is dropped rather than started, closing the race with stop()
    // active_homing_ rises before request_homing_ falls, so is_homing() never reads both false
    if (request_homing_.load()) {
      if (requested == HandLifecycle::Running) {
        active_homing_.store(true);
        (void)comms_.start_homing();
      } else {
        homing_state_.store(HomingState::Failed, std::memory_order_relaxed);
      }
      request_homing_.store(false);
    }

    // 2) Observe the hand
    HandState hand_state{};
    ActuatorHealth actuator_health{};
    (void)comms_.read(hand_state, actuator_health);
    fill_joint_state(hand_state);
    apply_tactile_bias(hand_state, cycle_count);
    hand_state.timestamp = system_nanoseconds();

    // 3) Watch the link and the drives
    const bool receiving = watch_link(loop, observed, cycle_count, communication_loss_cycles);
    watch_actuator_faults(loop, actuator_health, cycle_count);

    // 4) Control
    // The hold pose comes from this cycle's own observation, so it cannot be stale
    if (request_hold_.exchange(false, std::memory_order_relaxed)) {
      set_command_to_position_hold(hand_state);
    }
    canfd::CommandFrames frames{};
    bool write_ok = true;
    // Nothing is sent once the link has faulted: no node acknowledges a frame then, so it waits
    // in the adapter queue and reaches the hand only when the bus comes back
    const bool control_session = observed != HandLifecycle::Faulted &&
                                 (requested == HandLifecycle::Running ||
                                  requested == HandLifecycle::Stopped);
    if (control_session) {
      frames = command_to_frames(command, loop.active_controller_config, hand_state, actuator_health);

      // Re-read, because a stop() may have landed during this same cycle
      const HandLifecycle latest = requested_lifecycle_.load(std::memory_order_relaxed);
      const bool quick_stop = latest != HandLifecycle::Running;
      write_ok = comms_.write(frames, quick_stop);

      // A full TX queue is not a lost link, and treating it as one loops recovery against failure
      const int tx_errno = comms_.last_transmit_errno();
      const bool tx_transient = tx_errno == EAGAIN || tx_errno == EWOULDBLOCK || tx_errno == ENOBUFS;
      if (!write_ok && !tx_transient && latest == HandLifecycle::Running) {
        (void)event_ring_.push({RealtimeEventKind::TransmitFailure, cycle_count, -1, tx_errno});
        last_stop_cause_ = {RealtimeEventKind::TransmitFailure, cycle_count, tx_errno};
        lifecycle_.store(HandLifecycle::Faulted, std::memory_order_release);
        observed = HandLifecycle::Faulted;
      }
      if (tx_transient) write_ok = true;
    }

    // 5) Judge the stop and the homing
    // Link liveness, which unlike a latch returns to true on recovery
    check_communication_.store(control_session ? (receiving && write_ok) : receiving,
                               std::memory_order_relaxed);
    watch_drive_state(receiving);
    watch_homing(loop, cycle_count);

    // 6) Publish and wait
    fill_commanded(hand_state.commanded, loop.active_command, frames, control_session);
    const HandLifecycle echo_request = requested_lifecycle_.load(std::memory_order_relaxed);
    hand_state.commanded.selected_source =
        !control_session ? CommandSource::None
        : echo_request == HandLifecycle::Stopped ? CommandSource::QuickStop
        : comms_.is_homing() ? CommandSource::Homing
                             : CommandSource::Controller;
    state_buffer_.write(hand_state);

    // lifecycle is added by diagnostics() on the user thread
    const long cycle_end = now_nanoseconds();
    const double compute_ms = static_cast<double>(cycle_end - cycle_start) / 1000000.0;
    if ((cycle_end - cycle_start) > kPeriodNanoseconds) ++loop.deadline_misses;
    ++cycle_count;
    Diagnostics diagnostics{};
    diagnostics.control_cycles  = cycle_count;
    diagnostics.deadline_misses = loop.deadline_misses;
    diagnostics.last_period_ms  = last_period_ms;
    diagnostics.last_compute_ms = compute_ms;
    diagnostics.actuator_health = actuator_health;
    diagnostics_buffer_.write(diagnostics);

    // An overrun proceeds immediately instead of skipping the cycle
    next_wakeup += kPeriodNanoseconds;
    sleep_until_nanoseconds(next_wakeup);
  }
  } catch (const std::exception& loop_exception) {
    // The loop returns values instead of throwing, so reaching here is an SDK bug
    handle_control_loop_failure(loop_exception.what(), cycle_count);
  } catch (...) {
    handle_control_loop_failure("unknown exception", cycle_count);
  }

  // The SDK's code stays locked for the other hands
  (void)munlock(reinterpret_cast<const void*>(stack_floor), stack_top - stack_floor);
  (void)munlock(this, sizeof(*this));
}

// ------------------------------ Loop internals ------------------------------

// Silence makes the state stale, so it faults from any lifecycle and only reconnect() recovers
bool HandCore::watch_link(LoopState& loop, HandLifecycle& observed, std::uint64_t cycle,
                          long loss_cycles)
{
  // Mirror the first frame for the connect() wait
  if (!loop.first_frame_reported && comms_.any_frame_received()) {
    check_first_frame_.store(true, std::memory_order_relaxed);
    loop.first_frame_reported = true;
  }
  if (!comms_.any_frame_received()) {
    return false;
  }

  // The loop does not run in Faulted, so this edge is the only report of the silence
  const bool receiving = comms_.cycles_since_last_frame() <= loss_cycles;
  if (loop.previous_receiving && !receiving) {
    const auto anomaly = static_cast<std::int32_t>(comms_.last_receive_anomaly());
    last_stop_cause_ = {RealtimeEventKind::ReceiveSilence, cycle, anomaly};
    lifecycle_.store(HandLifecycle::Faulted, std::memory_order_release);
    observed = HandLifecycle::Faulted;
    (void)event_ring_.push({RealtimeEventKind::ReceiveSilence, cycle, -1, anomaly});
  }
  loop.previous_receiving = receiving;
  return receiving;
}

void HandCore::watch_actuator_faults(LoopState& loop, const ActuatorHealth& actuator_health,
                                     std::uint64_t cycle)
{
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    const ActuatorFault fault = actuator_health.fault[i];
    if (fault == loop.previous_fault[i]) continue;
    (void)event_ring_.push({fault == ActuatorFault::None ? RealtimeEventKind::ActuatorFaultCleared
                                                         : RealtimeEventKind::ActuatorFaultSet,
                            cycle, static_cast<std::int32_t>(i), static_cast<std::int32_t>(fault)});
    loop.previous_fault[i] = fault;
  }
}

// The evidence the drives left the last command behind, so a first frame is not enough
// receiving gates every verdict, because a stale status word proves nothing
void HandCore::watch_drive_state(bool receiving)
{
  const HandLifecycle requested = requested_lifecycle_.load(std::memory_order_relaxed);
  const HandLifecycle observed  = lifecycle_.load(std::memory_order_relaxed);
  if (observed == HandLifecycle::Faulted) return;

  // A decoded frame proves the link, whatever the drives are doing
  if (observed == HandLifecycle::Disconnected && comms_.any_frame_received()) {
    lifecycle_.store(HandLifecycle::Connected, std::memory_order_release);
  }
  if (!receiving) return;

  // Confirmed Running stays until a stop or a fault: one actuator dropping out is masked instead
  if (requested == HandLifecycle::Running && observed != HandLifecycle::Running) {
    if (comms_.drives_enabled()) lifecycle_.store(HandLifecycle::Running, std::memory_order_release);
  } else if (requested == HandLifecycle::Stopped && observed != HandLifecycle::Stopped) {
    if (comms_.quick_stop_attained()) {
      lifecycle_.store(HandLifecycle::Stopped, std::memory_order_release);
    }
  }
}

// The single completion point for every homing path, blocking home() included
void HandCore::watch_homing(LoopState& loop, std::uint64_t cycle)
{
  const bool homing_active = comms_.is_homing();
  if (homing_active) {
    const HomingPhase phase = comms_.homing_phase();
    if (!loop.homing_was_active || phase != loop.previous_homing_phase) {
      (void)event_ring_.push({RealtimeEventKind::HomingPhaseEntered, cycle, -1,
                              static_cast<std::int32_t>(phase)});
      loop.previous_homing_phase = phase;
    }
  }
  loop.homing_was_active = homing_active;

  if (!active_homing_.load(std::memory_order_relaxed) || comms_.is_homing()) {
    return;
  }
  const bool homing_ok = comms_.last_homing_outcome() == HomingOutcome::Succeeded;

  // On failure the Idle latched at the request stays, so the hand ends torque-free
  if (homing_ok) set_command_to_home_hold();

  result_homing_.store(homing_ok, std::memory_order_relaxed);
  homing_state_.store(homing_ok ? HomingState::Succeeded : HomingState::Failed,
                      std::memory_order_relaxed);
  // Released last, so once is_homing() reads false home() sees this result and state, and a
  // command sent after home() returns is not overwritten by the hold
  active_homing_.store(false, std::memory_order_release);

  std::int32_t failed_mask = 0;
  if (!homing_ok) {
    const auto& failed = comms_.last_homing_failed_actuators();
    for (std::size_t i = 0; i < kActuatorCount; ++i)
      if (failed[i]) failed_mask |= (1 << i);
  }
  (void)event_ring_.push({homing_ok ? RealtimeEventKind::HomingCompleted
                                    : RealtimeEventKind::HomingFailed,
                          cycle, -1, static_cast<std::int32_t>(comms_.last_homing_outcome()),
                          failed_mask});

  // Per actuator status words for the debug log
  if (!homing_ok) {
    const auto& failed = comms_.last_homing_failed_actuators();
    const auto& status = comms_.last_homing_status_snapshot();
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      if (!failed[i]) continue;
      (void)event_ring_.push({RealtimeEventKind::HomingFailed, cycle,
                              static_cast<std::int32_t>(i), static_cast<std::int32_t>(status[i])});
    }
  }
}

// Once, before the loop starts
void HandCore::apply_realtime_scheduling(std::uintptr_t stack_floor, std::uintptr_t stack_top)
{
  // mlock also faults the pages in, and the loop takes no first-touch fault on them
  int lock_error = 0;
  if (mlock(reinterpret_cast<const void*>(stack_floor), stack_top - stack_floor) != 0) lock_error = errno;
  if (mlock(this, sizeof(*this)) != 0 && lock_error == 0) lock_error = errno;
  const int code_error = lock_segments_holding(&kThreadPriority);
  if (lock_error == 0) lock_error = code_error;

  sched_param param{};
  param.sched_priority = kThreadPriority;
  const bool fifo_applied = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0;

  if (rt_cpu_affinity_ >= 0) {
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(rt_cpu_affinity_, &cpu_set);
    (void)pthread_setaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set);
  }

  // The scheduler and memory rows of Connection Info report these, since they describe the setup, not a failure
  realtime_report_.fifo_priority = fifo_applied ? kThreadPriority : 0;
  realtime_report_.running_cpu   = sched_getcpu();
  realtime_report_.memory_locked = lock_error == 0;
}

// Reopens comms only, without rebuilding the core, and returns to Running on success
// An open socket is not a recovery, so it waits for a decoded frame before believing the link
bool HandCore::try_reconnect_once()
{
  comms_.deactivate();
  if (!comms_.activate().ok()) {
    return false;
  }
  {
    // Same 300 ms bound as wait_first_frame
    constexpr int kFirstFrameCycles = 150;
    HandState scratch_state{};
    ActuatorHealth scratch_health{};
    bool observed = false;
    for (int cycle = 0; cycle < kFirstFrameCycles && !observed; ++cycle) {
      (void)comms_.read(scratch_state, scratch_health);
      observed = comms_.any_frame_received();
      timespec poll{0, 2000000L};
      nanosleep(&poll, nullptr);
    }
    if (!observed) {
      comms_.deactivate();
      return false;
    }
  }

  // "auto" may have resolved to a different channel
  interface_name_ = comms_.interface_name();

  // Clears active_homing_ too, so an interrupted homing is not read as complete
  reset_fault_state();

  check_communication_.store(false, std::memory_order_relaxed);

  // The drive may have rebooted
  homing_state_.store(HomingState::NotRun, std::memory_order_relaxed);

  // So the command from before the drop cannot move the hand on recovery
  reset_command_to_idle();

  if (auto_reconnect_home_) {
    request_homing_.store(true, std::memory_order_relaxed);
  }
  requested_lifecycle_.store(HandLifecycle::Running, std::memory_order_release);

  // The link is back, so the fault is over. The enable evidence is what makes it Running again
  lifecycle_.store(HandLifecycle::Connected, std::memory_order_release);
  log_info(hand_side_, std::string("auto-reconnect succeeded on ") + interface_name_ +
           " — control resumed" + (auto_reconnect_home_ ? " (homing)" : ""));
  return true;
}

// Before homing the encoders mean no pose yet, so FK returns NaN
// A joint that did not solve keeps its last valid angle, because a NaN would seed a controller
void HandCore::fill_joint_state(HandState& hand_state)
{
  std::array<int, kActuatorCount> encoder{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    encoder[i] = static_cast<int>(hand_state.actuators.position_count[i]);
  }
  std::array<double, kinematics::JOINT_NUM> joint_rad{};

  // Fingertip positions are not carried in HandState
  std::array<double, kinematics::TASK_NUM>  task{};
  solver_.fk_actuator_to_joint(encoder, joint_rad, task);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (std::isfinite(joint_rad[i])) {
      last_valid_joint_rad_[i] = joint_rad[i];
    }
    hand_state.joints.position_rad[i] = last_valid_joint_rad_[i];
  }

  // Velocity on every joint through the Jacobians, effort on the active ones, the passive j4 having
  // no actuator to take it from
  hand_state.joints.velocity_rad_s = kinematics::fk_actuator_to_joint_velocity(solver_, actuator_, hand_state);
  const std::array<double, kActiveJointCount> effort =
      kinematics::fk_actuator_to_joint_effort(solver_, actuator_, hand_state);
  hand_state.joints.effort_Nm.fill(std::numeric_limits<double>::quiet_NaN());
  for (std::size_t active = 0; active < kActiveJointCount; ++active) {
    hand_state.joints.effort_Nm[joint_of_active(active)] = effort[active];
  }
}

void HandCore::apply_tactile_bias(HandState& hand_state, std::uint64_t cycle)
{
  const TactileBiasRequest request = request_tactile_bias_.exchange(TactileBiasRequest::None);
  if (request == TactileBiasRequest::Set) {
    tactile_bias_ = hand_state.tactile;
  } else if (request == TactileBiasRequest::Reset) {
    tactile_bias_ = TactileState{};
  }
  if (request != TactileBiasRequest::None) {
    tactile_bias_buffer_.write(tactile_bias_);
    const RealtimeEventKind kind = request == TactileBiasRequest::Set ? RealtimeEventKind::TactileBiasSet
                                                                      : RealtimeEventKind::TactileBiasReset;
    (void)event_ring_.push({kind, cycle, -1, 0});
  }

  for (std::size_t finger = 0; finger < kFingerCount; ++finger) {
    for (std::size_t taxel = 0; taxel < kTactileTaxelsPerFinger; ++taxel) {
      hand_state.tactile.fingers[finger][taxel] -= tactile_bias_.fingers[finger][taxel];
    }
  }
  for (std::size_t taxel = 0; taxel < kPalmTactileCount; ++taxel) {
    hand_state.tactile.palm[taxel] -= tactile_bias_.palm[taxel];
  }
}

canfd::CommandFrames HandCore::command_to_frames(const HandCommand& command,
                                                 const ControllerConfig& controller_config,
                                                 const HandState& hand_state,
                                                 const ActuatorHealth& actuator_health)
{
  canfd::CommandFrames frames{};

  // The drive caps current at max_effort in CSP but not in CST, so CST clamps below
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    frames.max_effort[i] = static_cast<std::int16_t>(command.max_effort_pct[i]);
  }

  // Actuator and idle commands pass through, joint commands need IK and controller state
  if (const auto* c = std::get_if<ActuatorPositionCommand>(&command.controller)) {
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      frames.mode_of_operation[i] = canfd::mode_of_operation::kCSP;
      frames.target_position[i]   = static_cast<std::int32_t>(c->target[i]);
    }
  } else if (const auto* c = std::get_if<ActuatorEffortCommand>(&command.controller)) {
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      frames.mode_of_operation[i] = canfd::mode_of_operation::kCST;
      frames.target_effort[i]     = static_cast<std::int16_t>(
          std::clamp(c->target[i], -command.max_effort_pct[i], command.max_effort_pct[i]));
    }
  } else if (const auto* c = std::get_if<JointPositionCommand>(&command.controller)) {
    joint_position_controller_.compute(*c, controller_config.joint_position_controller, hand_state,
                                       frames);
  } else if (const auto* c = std::get_if<JointImpedanceCommand>(&command.controller)) {
    joint_impedance_controller_.compute(*c, controller_config.joint_impedance_controller, hand_state,
                                        command.max_effort_pct, frames);
  } else {
    // Idle, torque-free
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      frames.mode_of_operation[i] = canfd::mode_of_operation::kCST;
      frames.target_effort[i]     = 0;
    }
  }

  // An inactive controller must restart clean on re-entry
  if (!std::holds_alternative<JointPositionCommand>(command.controller))  joint_position_controller_.reset();
  if (!std::holds_alternative<JointImpedanceCommand>(command.controller)) joint_impedance_controller_.reset();

  // Masked actuators hold the measured position, because falling back to 0 would swing the hand
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    const bool masked = actuator_health.fault[i] != ActuatorFault::None || !actuator_health.enabled[i];
    if (!masked) continue;
    if (frames.mode_of_operation[i] == canfd::mode_of_operation::kCSP) {
      frames.target_position[i] = hand_state.actuators.position_count[i];
    } else {
      frames.target_effort[i] = 0;
    }
  }
  return frames;
}

// Keeps the thread from terminating the process, and leaves a trace
// Faulted is final for this core: only reconnect() recovers, while state() still reads
void HandCore::handle_control_loop_failure(const char* reason, std::uint64_t cycle)
{
  last_stop_cause_ = {RealtimeEventKind::ControlLoopFailure, cycle, 0};

  // Releases a blocking home() or stop() that is still waiting
  lifecycle_.store(HandLifecycle::Faulted, std::memory_order_release);

  // Says "without confirmation" because this path has no loop left to read the status words with
  log_critical(hand_side_, error_code::kControlLoopTerminated,
               std::string("control loop terminated, quick stop sent without confirmation (") + reason + ")");

  // Unlike a link fault the link is still up, and without the loop the drives hold the last
  // command, so quick stop is what leaves them torque-free
  try {
    canfd::CommandFrames frames{};
    for (int cycle = 0; cycle < 30; ++cycle) {
      (void)comms_.write(frames, true);
      timespec pause{0, 2000000L};
      nanosleep(&pause, nullptr);
    }
  } catch (...) {
    // Letting anything out here would terminate the process
  }
}

// --------------------------------- Helpers ----------------------------------

namespace
{

std::uintptr_t locked_stack_floor(std::uintptr_t top)
{
  void* stack = nullptr;
  std::size_t size = 0;
  pthread_attr_t attributes;
  if (pthread_getattr_np(pthread_self(), &attributes) == 0) {
    (void)pthread_attr_getstack(&attributes, &stack, &size);
    pthread_attr_destroy(&attributes);
  }
  const auto page = static_cast<std::uintptr_t>(sysconf(_SC_PAGESIZE));
  return std::max(reinterpret_cast<std::uintptr_t>(stack) + page, top - kLockedStackBytes);
}

int lock_segments_holding(const void* address)
{
  struct Search {
    std::uintptr_t address;
    int error;
  } search{reinterpret_cast<std::uintptr_t>(address), 0};
  dl_iterate_phdr(
      [](dl_phdr_info* info, std::size_t, void* data) -> int {
        auto& found = *static_cast<Search*>(data);
        bool holds = false;
        for (int i = 0; i < info->dlpi_phnum; ++i) {
          const ElfW(Phdr)& segment = info->dlpi_phdr[i];
          const std::uintptr_t start = info->dlpi_addr + segment.p_vaddr;
          if (segment.p_type == PT_LOAD && found.address >= start &&
              found.address < start + segment.p_memsz) {
            holds = true;
          }
        }
        if (!holds) return 0;
        for (int i = 0; i < info->dlpi_phnum; ++i) {
          const ElfW(Phdr)& segment = info->dlpi_phdr[i];
          if (segment.p_type != PT_LOAD) continue;
          const auto* start = reinterpret_cast<const void*>(info->dlpi_addr + segment.p_vaddr);
          if (mlock(start, segment.p_memsz) != 0 && found.error == 0) found.error = errno;
        }
        return 1;
      },
      &search);
  return search.error;
}

void fill_commanded(CommandedState& echo, const HandCommand& command,
                    const canfd::CommandFrames& frames, bool output_available)
{
  echo.controller_input = command.controller;
  echo.max_effort_pct = command.max_effort_pct;
  if (!output_available) {
    echo.controller_output = std::monostate{};
    return;
  }

  if (frames.mode_of_operation[0] == canfd::mode_of_operation::kCSP) {
    ActuatorPositionSetpoint output{};
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      output.target_position_cnt[i] = static_cast<double>(frames.target_position[i]);
    }
    echo.controller_output = output;
  } else {
    ActuatorEffortSetpoint output{};
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      output.target_effort_pct[i] = static_cast<double>(frames.target_effort[i]);
    }
    echo.controller_output = output;
  }
}

std::size_t joint_of_active(std::size_t active)
{
  return active < 4 ? active : 5 + 4 * ((active - 4) / 3) + (active - 4) % 3;
}

}  // namespace

}  // namespace aidin_hand2
