// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <memory>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/diagnostics.hpp>
#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>

namespace aidin_hand2
{

class HandCore;

// Non-owning handle to one hand; HandManager owns the resources
class Hand {
 public:
  // ------------------------------- Connection -------------------------------

  // Open the CAN socket and start the read-only control loop (blocking)
  void connect();

  // Stop the control loop and close the CAN socket (blocking)
  void disconnect();

  // Reopen the CAN socket and restart the control loop, keeping runtime settings (blocking)
  void reconnect();

  // -------------------------------- Operation -------------------------------

  // Enable the drives (non-blocking)
  void run();

  // Quick stop the drives (blocking)
  void stop();

  // Hard stop homing (blocking)
  void home();

  // Hard stop homing (non-blocking)
  void start_homing();

  // True while a homing sequence is running
  [[nodiscard]] bool is_homing() const;

  // -------------------------------- Command ---------------------------------

  // Latch a new target and switch to that controller
  void set_command(const Idle& command);
  void set_command(const JointPositionCommand& command);
  void set_command(const JointImpedanceCommand& command);
  void set_command(const ActuatorPositionCommand& command);
  void set_command(const ActuatorEffortCommand& command);

  // --------------------------------- Config ---------------------------------

  // Cap the drive current, % of rated, 0-2000
  void set_max_effort(double limit);
  void set_max_effort(const std::array<double, kActuatorCount>& limit);

  // Set the position filter and the impedance gains
  void set_controller_config(const ControllerConfig& config);

  // ------------------------------ Observation -------------------------------

  // Last observed actuator, joint, tactile and commanded state
  [[nodiscard]] HandState get_state() const;

  // Lifecycle, homing, loop timing and per-actuator health
  [[nodiscard]] Diagnostics get_diagnostics() const;

  // Active controller mode
  [[nodiscard]] CommandMode get_command_mode() const;

 private:
  explicit Hand(std::weak_ptr<HandCore> core);

  std::weak_ptr<HandCore> core_;

  friend class HandManager;
};

}  // namespace aidin_hand2
