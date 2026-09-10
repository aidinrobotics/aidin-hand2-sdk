#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/diagnostics.hpp>
#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>

#include "hand_core/comms/hand_comms.hpp"
#include "hand_core/controllers/joint_impedance_controller.hpp"
#include "hand_core/controllers/joint_position_controller.hpp"
#include "hand_core/lifecycle/action_check.hpp"
#include "types/hand_command.hpp"
#include "types/realtime_buffer.hpp"
#include "types/stop_confirmation.hpp"
#include "types/realtime_event_ring.hpp"
#include "types/status.hpp"

namespace aidin_hand2
{

namespace test { struct HandCoreTestPeer; }

// Execution unit owned by HandManager: HandComms, the RT control thread and the state buffers
// Three threads that never call each other, they meet only on the channels drawn below
//
//   user thread                          RT control thread        event logging thread
//   -----------                          -----------------        --------------------
//   set_command ----------> command_buffer_ ----------->
//   set_controller_config -> controller_config_buffer_ ->
//   home ------------------> request_homing_ ----------->
//                         <- result_homing_ <------------
//   connect/run/stop ------> requested_lifecycle_ ------->
//                         <- lifecycle_ <----------------
//                         <- stop_confirmation_ <--------
//   state/diagnostics     <- state_buffer_ <-------------
//                                          event_ring_ -------------> log text
//
// The RT loop lives from connect until disconnect, destroy or Faulted, and comms_ belongs to the
// RT thread while it runs.
//
// requested_lifecycle_ is what a call asked for and the only TX input, lifecycle_ is what the hand
// was observed to reach. Requested Stopped with observed Running is an unconfirmed stop.
class HandCore {
  friend struct test::HandCoreTestPeer;

 public:
  // ------------------------------ Construction ------------------------------

  explicit HandCore(const HandConfig& config);
  ~HandCore();

  HandCore(const HandCore&) = delete;
  HandCore& operator=(const HandCore&) = delete;
  HandCore(HandCore&&) = delete;
  HandCore& operator=(HandCore&&) = delete;

  // ------------------------------- Connection -------------------------------

  // Open the CAN socket and start the read-only control loop (blocking)
  [[nodiscard]] Status connect();

  // Stop the control loop and close the CAN socket (blocking)
  [[nodiscard]] Status disconnect();

  // Reopen the CAN socket and restart the control loop, keeping runtime settings (blocking)
  [[nodiscard]] Status reconnect();

  // Disconnect and refuse every later call (blocking)
  void close();

  // ------------------------------- Operation --------------------------------

  // Enable the drives and wait for them to confirm it (blocking)
  [[nodiscard]] Status run();

  // Quick stop the drives (blocking)
  [[nodiscard]] Status stop();

  // Hard stop homing (blocking)
  [[nodiscard]] Status home();

  // Hard stop homing (non-blocking)
  [[nodiscard]] Status start_homing();

  // True while a homing sequence is running
  [[nodiscard]] bool is_homing() const;

  // -------------------------------- Command ---------------------------------

  [[nodiscard]] Status set_command(const Idle& command);
  [[nodiscard]] Status set_command(const JointPositionCommand& command);
  [[nodiscard]] Status set_command(const JointImpedanceCommand& command);
  [[nodiscard]] Status set_command(const ActuatorPositionCommand& command);
  [[nodiscard]] Status set_command(const ActuatorEffortCommand& command);

  // --------------------------------- Config ---------------------------------

  [[nodiscard]] Status set_max_effort(const std::array<double, kActuatorCount>& limit);
  [[nodiscard]] Status set_controller_config(const ControllerConfig& config);

  // ------------------------------ Observation -------------------------------

  [[nodiscard]] HandState state() const;
  [[nodiscard]] Diagnostics diagnostics() const;
  [[nodiscard]] CommandMode command_mode() const;
  [[nodiscard]] HandLifecycle lifecycle() const noexcept { return lifecycle_.load(); }
  [[nodiscard]] HandSide hand_side() const noexcept { return hand_side_; }

 private:
  // =============================== Functions ================================

  // ---------------------------- State transition ----------------------------

  // require_homed false skips the homing gate, which set_command(Idle) needs
  [[nodiscard]] Status check_allowed(HandAction action, bool require_homed = true) const;

  // Request control without waiting for the drives, which is what homing needs
  [[nodiscard]] Status request_control();

  // request_control() plus the wait for the drives to confirm the enable, which run() needs
  [[nodiscard]] Status enable_control();

  // The only path for an observed lifecycle change that logs an "<api>() succeeded"
  void transition_to(HandLifecycle next);

  // Waits for the observed lifecycle to reach one value, giving up on the deadline or on Faulted
  bool wait_observed(HandLifecycle target, long timeout_nanoseconds);

  [[nodiscard]] Status wait_first_frame(const char* operation);
  void stop_control_loop();

  // Clear the stop verdict, the cause and the homing channels
  void reset_fault_state();

  // ----------------------------- Command write ------------------------------

  void republish_staged();
  void reset_command_to_idle();

  // Hold actuator position 0 right after homing
  void set_command_to_home_hold();

  // Hold the measured pose after stop -> run
  void set_command_to_position_hold(const HandState& hand_state);

  // ------------------------ Control loop [RT thread] ------------------------

  // Everything run_control_loop carries from one cycle to the next
  struct LoopState {
    // Last value read, kept when no new one arrives
    HandCommand      active_command{};
    ControllerConfig active_controller_config{};

    // Compared against to find what changed since the previous cycle
    std::array<ActuatorFault, kActuatorCount> previous_fault{};
    bool          previous_receiving{true};
    bool          first_frame_reported{false};
    bool          previous_latched{false};
    bool          homing_was_active{false};
    HomingPhase   previous_homing_phase{};

    // One latch episode, reporting each outcome once
    long confirmation_cycles{0};
    bool confirmation_by_fault{false};
    bool confirmation_confirmed_reported{false};
    bool confirmation_not_confirmed_reported{false};

    std::uint64_t deadline_misses{0};
    long          previous_cycle_start{0};

    // 0 until the loop enters Faulted with auto_reconnect on
    long auto_reconnect_deadline{0};
    bool auto_reconnect_started{false};
  };

  void run_control_loop();

  // ----------------------- Loop internals [RT thread] -----------------------

  // Best effort, silent when the process lacks privileges
  void apply_realtime_scheduling() const;

  // One in-loop link retry while Faulted
  [[nodiscard]] bool try_reconnect_once();

  // FK, updates last_valid_joint_rad_
  void fill_joint_state(HandState& hand_state);

  [[nodiscard]] canfd::CommandFrames command_to_frames(
      const HandCommand& command, const ControllerConfig& controller_config,
      const HandState& hand_state, const ActuatorHealth& actuator_health);

  // Returns whether frames are still arriving, and moves observed to Faulted on a drop
  [[nodiscard]] bool watch_link(LoopState& loop, HandLifecycle& observed,
                                std::uint64_t cycle, long loss_cycles);

  void watch_actuator_faults(LoopState& loop, const ActuatorHealth& actuator_health,
                             HandLifecycle observed, std::uint64_t cycle);

  // The only place drive evidence moves the observed lifecycle
  void watch_drive_state(bool receiving);

  void watch_stop_verdict(LoopState& loop, bool receiving, std::uint64_t cycle, long window);

  void watch_homing(LoopState& loop, std::uint64_t cycle);

  void handle_control_loop_failure(const char* reason, std::uint64_t cycle);

  // ------------------- Event ring drain [logging thread] --------------------

  void run_event_logging_loop();
  void log_pending_events();

  // =============================== Variables ================================

  // -------------------- Config [all threads, read only] ---------------------

  // Fixed after construction, declaration order is constructor order

  HandComms comms_;
  long period_nanoseconds_;

  // -1 = unset
  int  rt_cpu_affinity_;

  bool auto_reconnect_;

  // 0 = no limit
  int  auto_reconnect_timeout_ms_;

  bool auto_reconnect_home_;
  bool auto_home_;

  // False only under AIDIN_HAND2_WORKSPACE_CLAMP=0
  bool workspace_clamp_;

  std::string interface_name_;
  HandSide    hand_side_;

  // ---------------- Single thread: plain [RT, user, logging] ----------------

  // [RT]
  controllers::JointPositionController  joint_position_controller_;
  controllers::JointImpedanceController joint_impedance_controller_;

  // [RT] holds the last solved angle for joints FK could not solve
  std::array<double, kJointCount> last_valid_joint_rad_{};

  // [user] staging_mutex_ is mutable because command_mode() is const
  HandCommand staging_command_{};

  // The RT loop keeps its own copy and every connect() starts a thread that begins from the
  // default, so the last config is kept here to be republished
  ControllerConfig staging_controller_config_{};
  mutable std::mutex  staging_mutex_;
  mutable HandState   last_state_{};
  mutable Diagnostics last_diagnostics_{};
  std::thread rt_thread_;
  std::thread event_logging_thread_;

  // [logging]
  std::uint64_t count_reported_dropped_{0};

  // ------------- Cross thread: buffer [user <-> RT -> logging] --------------

  // [user -> RT]
  RealtimeBuffer<HandCommand>      command_buffer_;
  RealtimeBuffer<ControllerConfig> controller_config_buffer_;

  // [RT -> user]
  mutable RealtimeBuffer<HandState>   state_buffer_;
  mutable RealtimeBuffer<Diagnostics> diagnostics_buffer_;

  // [RT -> logging]
  RealtimeEventRing event_ring_;

  // ------------- Cross thread: atomic [RT <-> user <-> logging] -------------

  // Prefixes
  //   request_   gate held until consumed
  //   requested_ standing request, held until the next public call changes it
  //   check_     observation
  //   active_    in progress
  //   result_    outcome
  //   count_     counter
  //   last_      previous value

  // Request, read by RT as the TX input, and Disconnected is the loop's exit signal
  std::atomic<HandLifecycle> requested_lifecycle_{HandLifecycle::Disconnected};

  // Fact, written by RT from drive evidence and by the user thread once the socket is closed
  std::atomic<HandLifecycle> lifecycle_{HandLifecycle::Disconnected};

  // Stop verdict for the event log, written by RT
  // last_stop_cause_ is written before the release store to lifecycle_, read after the acquire
  std::atomic<StopConfirmation> stop_confirmation_{StopConfirmation::Waiting};
  StopCause last_stop_cause_{};

  // Homing, home() requests, the RT loop runs it and mirrors the result back
  std::atomic<HomingState> homing_state_{HomingState::NotRun};
  std::atomic<bool> request_homing_{false};
  std::atomic<bool> active_homing_{false};
  std::atomic<bool> result_homing_{false};

  // Pose hold, enable_control() requests it on stop -> run and the RT loop fills the pose
  std::atomic<bool> request_hold_{false};

  // Link, check_first_frame_ is consumed by the connect() wait
  std::atomic<bool> check_communication_{false};
  std::atomic<bool> check_first_frame_{false};

  // Rejected commands, the flag keeps the log on the change instead of every cycle
  std::atomic<std::uint64_t> count_nan_command_{0};
  std::atomic<bool>          last_command_was_nan_{false};

  // Destroy gate, one-shot, every public call is refused once it is set
  std::atomic<bool> request_destroy_{false};

  // Event logging thread run flag
  std::atomic<bool> event_logging_running_{false};
};

}  // namespace aidin_hand2