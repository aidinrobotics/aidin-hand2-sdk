#include <aidin_hand2/hand/hand.hpp>

#include <memory>
#include <utility>

#include "hand_core/hand_core.hpp"
#include "types/status.hpp"

namespace aidin_hand2
{

namespace
{

// Throws when the handle outlived its core
std::shared_ptr<HandCore> lock_core(const std::weak_ptr<HandCore>& core);

// Runs the action on the locked core and raises a failed Status as an exception
template <class Action>
void invoke(const std::weak_ptr<HandCore>& core, Action&& action);

}  // namespace

// ------------------------------- Construction -------------------------------

Hand::Hand(std::weak_ptr<HandCore> core)
: core_(std::move(core))
{
}

// -------------------------------- Connection --------------------------------

void Hand::connect()
{
  guard_boundary("connect()", [&] { invoke(core_, [](HandCore& c) { return c.connect(); }); });
}

void Hand::disconnect()
{
  guard_boundary("disconnect()", [&] { invoke(core_, [](HandCore& c) { return c.disconnect(); }); });
}

void Hand::reconnect()
{
  guard_boundary("reconnect()", [&] { invoke(core_, [](HandCore& c) { return c.reconnect(); }); });
}

// -------------------------------- Operation ---------------------------------

void Hand::run()
{
  guard_boundary("run()", [&] { invoke(core_, [](HandCore& c) { return c.run(); }); });
}

void Hand::stop()
{
  guard_boundary("stop()", [&] { invoke(core_, [](HandCore& c) { return c.stop(); }); });
}

void Hand::home()
{
  guard_boundary("home()", [&] { invoke(core_, [](HandCore& c) { return c.home(); }); });
}

void Hand::start_homing()
{
  guard_boundary("start_homing()", [&] { invoke(core_, [](HandCore& c) { return c.start_homing(); }); });
}

bool Hand::is_homing() const
{
  return guard_boundary("is_homing()", [&] { return lock_core(core_)->is_homing(); });
}

// --------------------------------- Command ----------------------------------

void Hand::set_command(const Idle& command)
{
  guard_boundary("set_command()", [&] { invoke(core_, [&](HandCore& c) { return c.set_command(command); }); });
}

void Hand::set_command(const JointPositionCommand& command)
{
  guard_boundary("set_command()", [&] { invoke(core_, [&](HandCore& c) { return c.set_command(command); }); });
}

void Hand::set_command(const JointImpedanceCommand& command)
{
  guard_boundary("set_command()", [&] { invoke(core_, [&](HandCore& c) { return c.set_command(command); }); });
}

void Hand::set_command(const ActuatorPositionCommand& command)
{
  guard_boundary("set_command()", [&] { invoke(core_, [&](HandCore& c) { return c.set_command(command); }); });
}

void Hand::set_command(const ActuatorEffortCommand& command)
{
  guard_boundary("set_command()", [&] { invoke(core_, [&](HandCore& c) { return c.set_command(command); }); });
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
  guard_boundary("set_max_effort()", [&] { invoke(core_, [&](HandCore& c) { return c.set_max_effort(limit); }); });
}

void Hand::set_controller_config(const ControllerConfig& config)
{
  guard_boundary("set_controller_config()",
                 [&] { invoke(core_, [&](HandCore& c) { return c.set_controller_config(config); }); });
}

// ------------------------------- Observation --------------------------------

HandState Hand::get_state() const
{
  return guard_boundary("get_state()", [&] { return lock_core(core_)->state(); });
}

Diagnostics Hand::get_diagnostics() const
{
  return guard_boundary("get_diagnostics()", [&] { return lock_core(core_)->diagnostics(); });
}

CommandMode Hand::get_command_mode() const
{
  return guard_boundary("get_command_mode()", [&] { return lock_core(core_)->command_mode(); });
}

// --------------------------------- Helpers ----------------------------------

namespace
{

std::shared_ptr<HandCore> lock_core(const std::weak_ptr<HandCore>& core)
{
  auto locked = core.lock();
  if (!locked) {
    throw_error(ErrorCode::WrongCallOrder,
                "Hand is no longer valid (destroyed or manager gone)");
  }
  return locked;
}

template <class Action>
void invoke(const std::weak_ptr<HandCore>& core, Action&& action)
{
  auto c = lock_core(core);
  throw_if_error(c->hand_side(), action(*c));
}

}  // namespace

}  // namespace aidin_hand2
