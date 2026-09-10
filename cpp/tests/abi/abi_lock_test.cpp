// Purpose: fail the build when the public ABI changes, so that the version bump is not forgotten.
//
// Why this exists: the SDK ships as libaidin_hand2.so.0.4, and the SOVERSION boundary is the
//   minor. A consumer that does not rebuild keeps loading whatever 0.4.x is installed. So if a
//   field is added to a public struct, or an enumerator is inserted mid-enum, and the minor is
//   not bumped, the old consumer loads the new library and there is no diagnostic:
//     - a grown struct returned by value overruns the caller's return slot (stack smashing)
//     - a shifted enumerator makes the caller's `== Faulted` comparison quietly false, so an
//       emergency stop branch never runs
//   Nothing else in the build catches that. find_package's SameMinorVersion and the SOVERSION
//   both only act once the minor has moved.
//
// What is checked: sizeof for the public types that cross the boundary, and the last enumerator
//   of every public enum. Inserting an enumerator anywhere but the end shifts the last one, so
//   locking the tail catches it. Appending at the end is ABI-safe and deliberately not caught.
//
// Coverage by containment: HandState holds ActuatorState, JointState, TactileState and
//   CommandedState, and CommandedState holds the command types, so a change to any of those
//   moves sizeof(HandState). Diagnostics holds ActuatorHealth. That is why the nested types are
//   not listed separately.
//
// When this fails: bump the minor in project() in cpp/CMakeLists.txt, then update the number
//   here. SOVERSION follows from project(), so the soname moves with it. The failing diagnostic
//   reads SizeLock<aidin_hand2::HandState, 2576, 2448>, so the new number comes from there.

#include <cstddef>
#include <cstdint>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{

// The type is a template parameter so that two types of equal size stay distinct instantiations,
// and so that the diagnostic names the offending type and prints its actual size.
template <class T, std::size_t Actual, std::size_t Expected>
struct SizeLock {
  static_assert(Actual == Expected, "public ABI size changed: bump the minor in project()");
};

#define AIDIN_ABI_SIZE_LOCK(T, expected) template struct SizeLock<T, sizeof(T), expected>

AIDIN_ABI_SIZE_LOCK(HandState,               2448);
AIDIN_ABI_SIZE_LOCK(Diagnostics,               96);
AIDIN_ABI_SIZE_LOCK(HandConfig,                88);
AIDIN_ABI_SIZE_LOCK(ControllerConfig,         280);
AIDIN_ABI_SIZE_LOCK(Idle,                       1);
AIDIN_ABI_SIZE_LOCK(JointPositionCommand,     128);
AIDIN_ABI_SIZE_LOCK(JointImpedanceCommand,    128);
AIDIN_ABI_SIZE_LOCK(ActuatorPositionCommand,  128);
AIDIN_ABI_SIZE_LOCK(ActuatorEffortCommand,    128);
AIDIN_ABI_SIZE_LOCK(Hand,                      16);
AIDIN_ABI_SIZE_LOCK(HandManager,               24);
AIDIN_ABI_SIZE_LOCK(Exception,                 24);

#undef AIDIN_ABI_SIZE_LOCK

// Enum tails. A value here moving means an enumerator was inserted or renumbered, which changes
// what an already-built consumer reads.
static_assert(static_cast<int>(HandLifecycle::Faulted) == 4, "HandLifecycle renumbered");
static_assert(static_cast<int>(HomingState::Failed) == 3, "HomingState renumbered");
static_assert(static_cast<int>(CommandMode::ActuatorEffort) == 4, "CommandMode renumbered");
static_assert(static_cast<int>(CommandSource::Homing) == 3, "CommandSource renumbered");
static_assert(static_cast<int>(ErrorCode::UnexpectedError) == 7, "ErrorCode renumbered");
static_assert(static_cast<int>(HandSide::Right) == 1, "HandSide renumbered");
static_assert(static_cast<int>(Finger::Baby) == 4, "Finger renumbered");
static_assert(static_cast<int>(LogLevel::Off) == 6, "LogLevel renumbered");

// ActuatorFault carries the CiA-402 codes, so every value is explicit and insertion is safe.
// The tail is locked anyway, to catch a code being changed by accident.
static_assert(static_cast<std::uint16_t>(ActuatorFault::EmergencySwitchError) == 0xFF05,
              "ActuatorFault codes changed");

}  // namespace

// Everything above is a compile-time check, so reaching main means the ABI is unchanged.
int main()
{
  return 0;
}
