#include <aidin_hand2/hand/hand.hpp>

#include <memory>
#include <utility>

#include "hand_core/hand_core.hpp"
#include "hand_core/lifecycle/action_check.hpp"
#include "types/status.hpp"

namespace aidin_hand2
{

namespace
{

// Throws when the handle outlived its core
std::shared_ptr<HandCore> lock_core(const std::weak_ptr<HandCore>& core, HandSide side);

// Runs the action on the locked core and raises a failed Status as an exception
template <class Action>
void invoke(const std::weak_ptr<HandCore>& core, HandSide side, Action&& action);

}  // namespace

// ------------------------------- Construction -------------------------------

Hand::Hand(std::weak_ptr<HandCore> core, HandSide side)
: core_(std::move(core)), side_(side)
{
}

// -------------------------------- Connection --------------------------------

void Hand::connect()
{
  guard_boundary(side_, "connect hand", [&] { invoke(core_, side_, [](HandCore& c) { return c.connect(); }); });
}

void Hand::disconnect()
{
  guard_boundary(side_, "disconnect hand", [&] { invoke(core_, side_, [](HandCore& c) { return c.disconnect(); }); });
}

void Hand::reconnect()
{
  guard_boundary(side_, "reconnect hand", [&] { invoke(core_, side_, [](HandCore& c) { return c.reconnect(); }); });
}

// -------------------------------- Operation ---------------------------------

void Hand::run()
{
  guard_boundary(side_, "run hand", [&] { invoke(core_, side_, [](HandCore& c) { return c.run(); }); });
}

void Hand::stop()
{
  guard_boundary(side_, "stop hand", [&] { invoke(core_, side_, [](HandCore& c) { return c.stop(); }); });
}

void Hand::home()
{
  guard_boundary(side_, "home hand", [&] { invoke(core_, side_, [](HandCore& c) { return c.home(); }); });
}

void Hand::start_homing()
{
  guard_boundary(side_, "home hand", [&] { invoke(core_, side_, [](HandCore& c) { return c.start_homing(); }); });
}

bool Hand::is_homing() const
{
  return guard_boundary(side_, "read homing state", [&] { return lock_core(core_, side_)->is_homing(); });
}

// --------------------------------- Command ----------------------------------

void Hand::set_command(const Idle& command)
{
  guard_boundary(side_, "set command", [&] { invoke(core_, side_, [&](HandCore& c) { return c.set_command(command); }); });
}

void Hand::set_command(const JointPositionCommand& command)
{
  guard_boundary(side_, "set command", [&] { invoke(core_, side_, [&](HandCore& c) { return c.set_command(command); }); });
}

void Hand::set_command(const JointImpedanceCommand& command)
{
  guard_boundary(side_, "set command", [&] { invoke(core_, side_, [&](HandCore& c) { return c.set_command(command); }); });
}

void Hand::set_command(const ActuatorPositionCommand& command)
{
  guard_boundary(side_, "set command", [&] { invoke(core_, side_, [&](HandCore& c) { return c.set_command(command); }); });
}

void Hand::set_command(const ActuatorEffortCommand& command)
{
  guard_boundary(side_, "set command", [&] { invoke(core_, side_, [&](HandCore& c) { return c.set_command(command); }); });
}

// ---------------------------------- Config ----------------------------------

void Hand::set_max_effort(double limit)
{
  std::array<double, kActuatorCount> limits;
  limits.fill(limit);
  set_max_effort(limits);
}

void Hand::set_max_effort(const std::array<double, kActuatorCount>& limit)
{
  guard_boundary(side_, "set max effort", [&] { invoke(core_, side_, [&](HandCore& c) { return c.set_max_effort(limit); }); });
}

void Hand::set_controller_config(const ControllerConfig& config)
{
  guard_boundary(side_, "set controller config",
                 [&] { invoke(core_, side_, [&](HandCore& c) { return c.set_controller_config(config); }); });
}

// --------------------------------- Tactile ----------------------------------

void Hand::set_tactile_bias()
{
  guard_boundary(side_, "set tactile bias", [&] { invoke(core_, side_, [](HandCore& c) { return c.set_tactile_bias(); }); });
}

void Hand::reset_tactile_bias()
{
  guard_boundary(side_, "reset tactile bias", [&] { invoke(core_, side_, [](HandCore& c) { return c.reset_tactile_bias(); }); });
}

TactileState Hand::get_tactile_bias() const
{
  return guard_boundary(side_, "read tactile bias", [&] { return lock_core(core_, side_)->tactile_bias(); });
}

// ------------------------------- Observation --------------------------------

HandState Hand::get_state() const
{
  return guard_boundary(side_, "read state", [&] { return lock_core(core_, side_)->state(); });
}

Diagnostics Hand::get_diagnostics() const
{
  return guard_boundary(side_, "read diagnostics", [&] { return lock_core(core_, side_)->diagnostics(); });
}

CommandMode Hand::get_command_mode() const
{
  return guard_boundary(side_, "read command mode", [&] { return lock_core(core_, side_)->command_mode(); });
}

// --------------------------------- Helpers ----------------------------------

namespace
{

std::shared_ptr<HandCore> lock_core(const std::weak_ptr<HandCore>& core, HandSide side)
{
  auto locked = core.lock();
  if (!locked) {
    throw_if_error(side, hand_destroyed());
  }
  return locked;
}

template <class Action>
void invoke(const std::weak_ptr<HandCore>& core, HandSide side, Action&& action)
{
  throw_if_error(side, action(*lock_core(core, side)));
}

}  // namespace

}  // namespace aidin_hand2
