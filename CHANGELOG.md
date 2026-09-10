# Changelog

All notable changes to the AIDIN Hand Gen2 SDK are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/), and this
project adheres to [Semantic Versioning](https://semver.org/). The interface covered by that
versioning is the public C++ API — the headers under `include/aidin_hand2/` and the CMake package.

## [0.5.2] - 2026-09-10

Both thumb hardware generations are supported from one release, and the prebuilt kinematics
library is named after the SDK release so a library from one release can no longer satisfy another.

### Added

- **Support for the 2 mm thumb ball screw lead, alongside the 1 mm one.** Configure with
  `-DAIDIN_HAND2_THUMB_LEAD=2mm` for the earlier hardware; 1 mm stays the default. Both libraries
  ship, and their sonames differ, so a build for one hardware generation cannot load the other's
  kinematics. 0.5.1 told owners of the 2 mm hardware to stay on 0.5.0, which meant giving up every
  later fix; this replaces that with a build option.

### Changed

- **Breaking: `libaidin_hand2_kinematics.so.<SDK version>` replaces
  `libaidin_hand2_kinematics.so.1`.** Relink against this release; a consumer built against 0.5.1
  looks for a file name this release does not ship.
- **The kinematics library no longer carries a version of its own.** Its two numbers are gone and
  the file name comes from the project version, so there is one version to reason about.

> [!IMPORTANT]
> The old name made a silent mismatch possible, and it happened. What the library exports is four
> signatures that have never changed, but what it means is a set of calibration constants, and
> 0.5.1 halved the lead of two thumb screws. Under the fixed name a 0.5.0 library satisfied a
> 0.5.1 consumer, so those two actuators converted at half the counts per millimetre with no
> diagnostic of any kind. Installing two releases into different prefixes — `/usr/local` and
> `~/.local`, say — was enough to trigger it. A mismatch now fails to load and names the file it
> wanted.

### Documentation

- **Installing into a second prefix now carries a warning.** Build & install offered
  `--prefix <prefix>` as the way to avoid `sudo` without saying that an SDK left behind in an
  earlier prefix stays reachable, which is what let the mismatch above happen. Remove the earlier
  one when you move.
- **Build & install separates the two install paths and documents how to uninstall.** Section 3
  interleaved the `/usr/local` steps with the user-prefix steps paragraph by paragraph, so a
  reader following it in order had to keep filtering out the half that did not apply. It is now
  3.1 and 3.2, one of which you follow, and the new section 5 removes what was installed.

## [0.5.1] - 2026-09-10

The thumb's d1 and d2 screws changed from a 2 mm lead to a 1 mm one, so the kinematics converts
those two actuators at twice the count per millimetre. arm64 becomes a shipped architecture in the
same release. The public API is unchanged.

### Added

- **arm64 is a shipped architecture.** `cpp/prebuilt/aarch64/libaidin_hand2_kinematics.so.1`
  travels with the tree, so an arm64 host configures and builds the same way an x86_64 one does.
  Until now `find_package` stopped on arm64 with a message naming the architectures on offer.
- **CI covers both architectures.** Build and unit tests run on Ubuntu 22.04 and 24.04 for x86_64
  and arm64, and the unit tests run again under ThreadSanitizer. The kernel-side behaviour —
  CAN-FD, PREEMPT_RT and the 500 Hz loop — still needs the device and is verified there.

### Changed

- **The thumb d1 and d2 actuators convert at 16384 counts per mm** instead of 8192, because their
  screws now have a 1 mm lead. Thumb d0 and d3 and all twelve long finger actuators keep the 2 mm
  lead and its 8192. A joint target therefore produces different encoder counts for those two
  actuators than 0.5.0 did, and the same counts read back as different joint angles.
- **The prebuilt kinematics library is version 1.1.0.** The soname stays 1, because the four
  declarations it exports did not change, so it drops in without relinking `libaidin_hand2`.

> [!IMPORTANT]
> The default build is for a hand whose two thumb ball screws have a 1 mm lead. For the 2 mm
> hardware, configure with `-DAIDIN_HAND2_THUMB_LEAD=2mm`; see
> [2. Build](docs/en/06_sdk_build_and_install.md#2-build). Building for the wrong one moves those
> two actuators twice or half as far as asked.

### Documentation

- The `CPR` constant is documented as counts per millimetre of screw travel rather than counts per
  motor revolution. One motor revolution is 1024 counts and the lead is what turns that into a
  distance, which is exactly what this release changes. An example comment stated the old reading.

## [0.5.0] - 2026-09-10

This release makes the lifecycle report what the hand actually did rather than what was asked of
it. Several calls that used to return early now block until the drives confirm, so a call that
succeeds means the hardware got there. Read the Changed entries marked **Breaking** before
upgrading.

### Added

- **Prebuilt kinematics.** The kinematics core now arrives as
  `cpp/prebuilt/<arch>/libaidin_hand2_kinematics.so.1` instead of being compiled from source, so
  the equations and the hand geometry constants stay out of the distributed tree.
  The library exports only the four functions that `hand_kinematics_core.hpp` declares, and
  `BUILD-INFO` records the commit each library was built from. The soname is fixed at 1, so a
  rebuilt library drops in without relinking `libaidin_hand2`.
- **Eigen is no longer a dependency.** It is compiled into the prebuilt kinematics library.
- **ABI lock test.** `cpp/tests/abi/abi_lock_test.cpp` fails when a public type changes size or
  layout — the change that would silently break a consumer linking a prebuilt library.
- **The examples are rewritten to follow the C++ guide chapter by chapter.** `cpp/examples/` holds
  21 numbered files in the guide's chapter order, and a chapter with several subjects gets several
  files — one per controller in chapter 4 and one per command type in chapter 5. Each file is
  self-contained with no shared helpers, prints what it demonstrates to the terminal, and handles
  `SIGINT`, so Ctrl-C leaves the loop and the destructor confirms the quick stop and disconnects.
  `03_lifecycle_walkthrough.cpp` offers every transition call from every state behind a number key
  and refreshes the state line while it waits, so the refusals and a fault arriving on its own are
  as visible as the transitions. The four recovery-pattern files — `18_self_managed.cpp`,
  `19_external_server.cpp` with `19_external_client.cpp`, and `20_exit_on_failure.cpp` — mirror the
  three structures in [9.3 Recovery patterns](docs/en/07_cpp_usage_guide.md#93-recovery-patterns).
  `basic_control.cpp` is unchanged and stays the check that
  [SDK build & install](docs/en/06_sdk_build_and_install.md) walks through.
- **Apache License 2.0.** `LICENSE` carries the text and `NOTICE` the attribution that downstream
  distributions must keep. `THIRD_PARTY_NOTICES.md` lists every third-party component with its
  license and source, and `licenses/` holds the full texts. Public headers and examples carry an
  SPDX identifier.

### Changed

- **Breaking: request the new minor version.** Use `find_package(aidin_hand2 0.5 REQUIRED)`. The
  compatibility policy is `SameMinorVersion`, so a 0.4 consumer no longer configures against 0.5.
- **Breaking: `Diagnostics::lifecycle` reports an observation, not a request.** `stop()` no longer
  becomes `Stopped` before the drives confirm the quick stop, and `run()` no longer becomes
  `Running` before they confirm the enable. A `stop()` that threw therefore leaves the lifecycle
  `Running`, which means the actuators may still hold the last command.
- **Breaking: `run()` blocks until the drives report Operation Enabled**, up to 4000 ms, and throws
  `HardwareFault` or `CommunicationLost` when they do not. It used to return as soon as the request
  was recorded.
- **Breaking: `reconnect()` is allowed only in `Faulted`.** It used to be allowed in `Connected`
  and `Stopped` as well, where it silently rebuilt a healthy link. Use `disconnect()` and
  `connect()` to rebuild a link that is not faulted.
- **Breaking: `set_command()` no longer throws on a non-finite or out-of-range value.** The command
  is dropped, the previous one keeps going out, `Diagnostics::nan_command_count` increments and a
  warning is logged. Detect it through that counter rather than through a `try`/`catch`. Validation
  still throws from the setup calls — the `HandConfig` constructor, `create()`,
  `set_controller_config()` and `set_max_effort()`.
- **`disconnect()` no longer closes the socket while a stop is unconfirmed.** The remedy for
  `drives did not reach quick stop within 500 ms` is now powering the robot hand off, or
  `destroy()`, which disconnects anyway and logs it.
- **`stop()` and `disconnect()` re-verify on every call.** Calling either again after an
  unconfirmed stop used to succeed without checking anything, because the lifecycle already said
  `Stopped`.
- **`disconnect()` clears the stored command.** `set_command()` is accepted only in `Running`, so a
  command left over from before a disconnect could never apply, yet it stayed visible in
  `Diagnostics` and was re-sent on the next `run()`. `reconnect()` and homing already cleared it.
- **`set_controller_config()` survives a reconnect.** A `RealtimeBuffer` hands a value over once
  and every `connect()` starts an RT thread from the defaults, so a config set before the link
  dropped was silently replaced once the hand came back. `connect()` now republishes the staged
  config and the staged command with it.
- **A transition call asked for the state it already holds no longer fails.** `connect()` in
  `Connected` and `disconnect()` in `Disconnected` join `run()` in `Running` and `stop()` in
  `Stopped` as no-ops that log what they skipped. `reconnect()` stays restricted to `Faulted`, so a
  fault cannot be cleared without its cause being seen.
- **Homing leaves the enable stage as soon as the actuators answer.** It used to wait out the full
  4000 ms time limit before deciding, because the decision was made only at the timeout. The time
  limit is now reached only when an actuator never answers and has to be left out, so a healthy
  hand homes faster. The failure path is unchanged: when nothing answers, every actuator is
  reported as the cause and `EnableTimeout` is raised.
- **The file sink keeps 300 MiB instead of 20 MiB** — 50 MiB per file and five rotated files, up
  from 5 MiB and three. A repeating communication or actuator fault writes about 200 KiB/s, which
  used to cycle through the whole budget in under two minutes and left too little to analyse an
  incident afterwards.
- **`JointPositionController` defaults changed:** `cutoff_freq` is `10.0` Hz instead of `50.0`, and
  `deadband` is `0.000873` rad instead of `0.0`. The old defaults let input noise cross the
  backlash band repeatedly.
- **The rejection message for `connect()` in `Running` or `Stopped` pointed at a dead end.** It
  read `already connected — use reconnect() to rebuild`, but `reconnect()` is allowed only in
  `Faulted`. It now reads `already connected — call disconnect() first to rebuild the link`.

### Documentation

- **The C++ guide is reorganized around what you configure and when.** Control settings and
  commands are separate chapters, `HandState` and `Diagnostics` each have their own, and error
  handling ends with three recovery patterns that the new examples mirror. The lifecycle diagram is
  replaced.
- **Every chapter of the C++ guide ends with the examples that run it.** The introduction says
  where the executables are built and what the two arguments are, and each chapter closes with the
  files for that chapter and what each one prints.
- **The API reference is split into one file per header** under `08_cpp_api_reference/`, with an
  index that lists every public function and type by the file that declares it. Both languages
  changed; the single-file version is gone.
- **The English documentation is synchronized with the Korean.** Chapter structure, section
  numbers, tables and symbol links now match across `07`, `08`, `09` and `15`. Eight API reference
  entries regained notes that were shorter than the Korean, and three error messages in `15` were
  stale — one wording differed from `hand_core.cpp` and two were missing.
- **Terminology is fixed in both languages.** The realtime loop is the *control and communication
  loop*, the hardware is the *robot hand*, and *communication* replaces *link* — which now appears
  only for the Linux `ip link` command and for static linking.
- **Actuator and joint indices are documented consistently:** the thumb runs 0–3 and each long
  finger 1–3. Six places numbered long fingers from 0.
- **Header comments that disagreed with the implementation are corrected**, and the documentation
  scope is settled: `start_homing()`, `is_homing()` and the three unused kinematics functions stay
  out of the reference on purpose.

## [0.4.0] - 2026-08-26

### Added

- `ControllerConfig` in `types/config.hpp`, with the nested
  `ControllerConfig::JointPositionController` (`filter_enabled`, `cutoff_freq`, `deadband`) and
  `ControllerConfig::JointImpedanceController` (`stiffness`, `damping`).
- `void Hand::set_controller_config(const ControllerConfig&)` — callable at any time, applied from
  the next cycle. There is no create-time field on `HandConfig` and no getter.

### Removed

- `JointPositionCommand::speed_rad_s`. Bandwidth now comes from
  `ControllerConfig::JointPositionController::cutoff_freq`.
- `JointImpedanceCommand::gains`, the `ImpedanceGains` type, and the `kDefaultStiffness` /
  `kDefaultDamping` constants. Gains now come from
  `ControllerConfig::JointImpedanceController`, whose member defaults carry the same values.

### Changed

- JointPosition shapes a target with a trailing deadband and a 3rd-order low-pass instead of a
  speed rate limit. Re-entry still starts from the measured pose.
- Command validation covers only the targets. The finite and non-negative checks for the filter
  values and the gains moved to `set_controller_config()`.

## [0.3.1] - 2026-08-26

### Changed

- `run()` after `stop()` commands the observed pose once instead of leaving the
  stored command at `Idle`, so the hand holds its place until the next command.

## [0.3.0] - 2026-08-24

### Changed

- Homing runs once instead of twice, about 5 s shorter.
- Homing clears the completion bit before the hard-stop push, so the finger stays
  on the stop until the zeroing.
- Homing origin is the hard stop; the offset that keeps target 0 off it moved to
  the CAN frame boundary.
- After a successful homing the hand holds actuator position 0 instead of going
  torque-free.
- Homing settle is 2 s, and the enable, clear and trigger timeouts are 4 s.
- The stored command resets to `Idle` on `run()`, `stop()`, `reconnect()` and the
  homing trigger.
- `Diagnostics.homed` replaced by `Diagnostics.homing_state`.
- `types/hand_description.hpp` renamed to `types/description.hpp`.
- `types/hand_lifecycle.hpp` merged into `types/state.hpp`.
- `HandSide` moved from `types/config.hpp` to `types/description.hpp`.
- `ErrorCode::ControlLoopFailure` renamed to `ControlLoopFault`.

### Fixed

- A failed homing no longer leaves the previous command driving the hand.
- The first command after an `Idle` interval no longer ignores `speed_rad_s`.
- `start_homing()` failures now log the step, the reason and the actuators involved.

### Removed

- FK LUT tables. `init_kinematics_lut()` remains as a no-op.

## [0.2.0]

### Changed

- `HandLifecycle` has five states. `Connecting` and `FaultStopping` were
  removed: `connect()` blocks until the first frame, so `Connecting` was never
  observable, and `FaultStopping` always converged to `Faulted`. **The
  underlying enum values shifted.**
- `set_command()` is rejected outside `Running`. In 0.1.0 it returned success in
  `Connected` and `Stopped`, which have no transmit path, so the hand did not
  move.
- Consumers must request a version: `find_package(aidin_hand2 0.2 REQUIRED)`.
  The compatibility policy is `SameMinorVersion`, so a 0.1 consumer no longer
  configures against 0.2.

### Fixed

- Thumb task-IK used a sign opposite to the forward transform, and the thumb q1
  home offset is now 14.0° so joint 0 matches encoder 0.
- `RealtimeBuffer` had a data race: the writer could reuse a slot while the
  reader was still in it.

### Removed

- `web_bridge` and the GUI moved to the `aidin-hand2-gui` repository. The
  `AIDIN_HAND2_BUILD_WEB_BRIDGE` CMake option is gone.

## [0.1.0]

- Initial release.
