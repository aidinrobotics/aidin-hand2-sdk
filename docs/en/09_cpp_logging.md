# Logging

Logging sends the log that the SDK writes to three places: the console, a file, and a callback.
This document explains how you configure the three sinks and how you read one log line. For how
you read state and diagnostics, see the [C++ guide](07_cpp_usage_guide.md#7-handstate).

## Contents

&nbsp;&nbsp;[**1. Sinks**](#1-sinks)<br>
&nbsp;&nbsp;[**2. File sink**](#2-file-sink)<br>
&nbsp;&nbsp;[**3. Callback sink**](#3-callback-sink)<br>
&nbsp;&nbsp;[**4. When to configure**](#4-when-to-configure)<br>
&nbsp;&nbsp;[**5. Log record**](#5-log-record)<br>
&nbsp;&nbsp;[**6. Shutdown**](#6-shutdown)<br>
&nbsp;&nbsp;[**7. Full example**](#7-full-example)

## 1. Sinks

A sink is a destination the log is delivered to. The SDK provides three: console, file and
callback. With no setup the log goes to the console only.

| Sink | Default | Level |
|---|---|---|
| console | on, writes to `stderr` | `Info` |
| file | off | `Debug`, fixed |
| callback | none registered | `Debug`, fixed |

The following example lowers the level of the console sink and then turns the output off.

```cpp
namespace ah2 = aidin_hand2;

ah2::set_log_level(ah2::LogLevel::Debug);   // applies to the console sink only
ah2::set_log_to_console(false);             // turns the console output off
```

The level of the file sink and of the callback sink is fixed at `Debug`, so
[`set_log_level()`](08_cpp_api_reference/logging.md#set_log_level) does not affect them. The `Off` value of
the [`LogLevel`](08_cpp_api_reference/logging.md#enum-loglevel) enum also stops the console output, but
`Off` is a level setting, so another `set_log_level()` call brings the output back. To keep it off,
pass `false` to [`set_log_to_console()`](08_cpp_api_reference/logging.md#set_log_to_console). You can select
`Trace` as well, but the SDK writes no log at the `Trace` level, so it matches `Debug`.

## 2. File sink

The file sink writes the log to a file as well, from the moment you give it a path.

```cpp
ah2::set_log_to_file("/var/log/my_robot/aidin-hand2-left.log");
```

Create the directory during deployment and grant write permission.

```bash
sudo install -d -o robot -g robot -m 0750 /var/log/my_robot
```

| Item | Value |
|---|---|
| Rotation | A new file every 50 MiB. The current file and 5 earlier files come to at most 300 MiB |
| Level | `Debug` and above |
| Flush | Immediately at `Warn` and above. Otherwise about every second |

When the current file reaches 50 MiB, it moves down to an earlier file and every number goes up by
one. If you give the path `hand.log`, the earlier files run from `hand.1.log` to `hand.5.log`, a
larger number holds an older record, and the SDK deletes whatever passes `hand.5.log`.

The total stops at 300 MiB to keep the disk from filling up during a long run. The SDK does not
write every cycle; it writes only on a state transition or an event. Because of that, the event
rate rather than the run time decides how fast the 300 MiB goes. The file does not grow at all
without an event, and in a stretch where a communication error or an actuator fault repeats, the
files cycle through in about 25 minutes. To keep records longer than that, add a policy to your
deployment that moves the earlier files elsewhere.

> [!WARNING]
> Failing to open the file ends in one warning line, not an exception. If you also turned the
> console off and registered no callback, no signal remains at all. After you start the process,
> check from a health check that the file appeared and that it is growing.

## 3. Callback sink

Use the callback sink to pass the SDK log to an application logger or to ROS 2 logging.

```cpp
ah2::set_log_callback(
    [](ah2::LogLevel level, const std::string& message) {
      app_logger.write(map_level(level), message);
    });
```

| Item | Value |
|---|---|
| Level | Everything from `Debug` up |
| Arguments | `(level, the message body)` |
| Count | One. Another call replaces it, `nullptr` clears it |
| Other sinks | Independent of the console and file sinks |

The body that the callback receives is the message part from [5. Log record](#5-log-record), so
`[left]` and the error code come with it.

The SDK calls the callback synchronously on the thread that produced the log, and that thread
includes the control and communication loop. The time that the callback takes is spent inside the
control period, so do not perform blocking I/O, do not call back into the SDK, and do not let an
exception escape. The following example pushes to a queue and returns right away.

```cpp
ah2::set_log_to_console(false);
ah2::set_log_callback(
    [&queue](ah2::LogLevel level, const std::string& message) noexcept {
      queue.try_push(level, message);
    });
```

A callback that returns late lets events of the control and communication loop pile up before they
reach the log. Past a limit, the SDK leaves some events out of the log and writes the warning
`<count> realtime events dropped (event ring full)`.

## 4. When to configure

Decide the sinks before [`create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate). Otherwise you miss
the warnings that `create()` writes, such as an index outside the range in the
`disabled_actuators` field.

The following order sets all three sinks and then creates the hand.

```cpp
ah2::set_log_level(ah2::LogLevel::Info);
ah2::set_log_to_console(true);
ah2::set_log_to_file("/var/log/my_robot/hand.log");
ah2::set_log_callback(callback);

ah2::Hand hand = manager.create(config);
hand.connect();
```

After [`connect()`](08_cpp_api_reference/hand.md#handconnect), several threads produce log lines at the
same time. The SDK does not line a sink change up against that writing, so in production decide
once at startup and do not change it afterwards.

## 5. Log record

One log line consists of the following pieces.

```text
[1756713600.123456789] [aidin_hand2] [info]     [left] homing complete — home position established
[1756713600.223456789] [aidin_hand2] [critical] [left] [E404] communication lost (no RX)
```

| Piece | Description |
|---|---|
| `[epoch.nanosecond]` | The wall clock |
| `[aidin_hand2]` | The logger name |
| `[info]` | The level. The console sink pads it to 10 columns and colors it |
| `[left]` | Which side of the robot hand. Only on lines that carry hand context |
| `[E404]` | The error code. Only on lines whose cause and remedy are in [Error messages](15_error_messages.md) |
| message | The body |

`[left]` and the error code go into the message body rather than into the formatter pattern, and some
lines carry neither. The wall clock in front tells when an event happened. The events of the control
and communication loop are written in batches every 50 ms, so this time can trail the event by up to
50 ms.

When [`connect()`](08_cpp_api_reference/hand.md#handconnect) succeeds, the SDK writes a multi-line
Connection Info record at the info level. It shows in one place the settings of this run and what
the control and communication loop got from the system. The example below shows the message part
only.

```text
[left] connect() succeeded
-------------------------- Connection Info -------------------------
side        left
model       gen2
interface   can0
rate        500 Hz
auto_home   on
scheduler   SCHED_FIFO 90
cpu         3
affinity    none
memory      locked
actuators   16
disabled    0
faulted     0
sdk         0.9.0
--------------------------------------------------------------------
```

| Row | Description |
|---|---|
| `side`, `model`, `interface` | The `hand_side`, `hand_model` and `interface_name` fields of `HandConfig` |
| `rate` | The rate of the control and communication loop, fixed at 500 Hz |
| `auto_home` | The `auto_home` field |
| `scheduler` | The `SCHED_FIFO` priority of the control loop, or `SCHED_OTHER` without the privilege |
| `cpu` | The CPU the control loop runs on |
| `affinity` | The `rt_cpu_affinity` field, `none` for `-1` |
| `memory` | `locked` if the memory of the control loop is locked, `not locked` otherwise |
| `actuators` | The number of actuators |
| `disabled` | How many actuators the `disabled_actuators` field turns off |
| `faulted` | How many actuators are in fault at connect |
| `sdk` | The SDK version |

The SDK log holds the events that happen apart from any call. An event in the control and
communication loop carries the error code. The example below carries the message part only; a real
line carries the leading pieces from the table above.

```text
[left] [E404] communication lost (no RX)
```

An exception that a function throws does not reach the SDK log, so write `what()` to the application
log one time. [`Exception::what()`](08_cpp_api_reference/types_error.md#exception--stdruntime_error)
carries an error code such as `[E601]` too, which lines it up with the SDK log by the same code.

```cpp
try {
  // an SDK call
} catch (const ah2::Exception& error) {
  app_logger.error("SDK {}", error.what());
}
```

The cause and remedy of each code are in [Error messages](15_error_messages.md). For what you must
record alongside to trace the cause of an incident, see the
[C++ guide](07_cpp_usage_guide.md#82-diagnostic-record).

> [!IMPORTANT]
> Do not rely on the log for a safety decision. Read the
> [`lifecycle`](07_cpp_usage_guide.md#8-diagnostics) field and the
> [actuator fault](15_error_messages.md#10-actuator-fault-values) directly. The same holds for a safety
> circuit that you placed outside the SDK.

## 6. Shutdown

The shutdown order stops the robot hand first and then flushes the log.

```cpp
hand.stop();
manager.destroy(hand);
ah2::flush_log();
```

[`flush_log()`](08_cpp_api_reference/logging.md#flush_log) is not a function that stops the robot hand. Do
not swap the order.

## 7. Full example

The following example sets all three sinks, writes an exception as well, and then shuts down in
order. It keeps the logging setup ahead of
[`create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate) and places
[`flush_log()`](08_cpp_api_reference/logging.md#flush_log) after the robot hand shuts down.

```cpp
#include <iostream>
#include <string>
#include <aidin_hand2/aidin_hand2.hpp>

namespace ah2 = aidin_hand2;

// ------------------------------ Application ---------------------------------
// this example prints straight to standard output. to pass the log to an application
// logger, convert the level to that logger's level and hand it over

void app_log(ah2::LogLevel level, const std::string& message)
{
  std::cout << "[app] [" << static_cast<int>(level) << "] " << message << '\n';
}

// -------------------------------- Logging -----------------------------------

int main()
{
  // decide the sinks before create()
  ah2::set_log_level(ah2::LogLevel::Info);                       // applies to the console sink only
  ah2::set_log_to_console(true);
  ah2::set_log_to_file("/var/log/my_robot/aidin-hand2.log");      // everything from Debug up
  ah2::set_log_callback(
      [](ah2::LogLevel level, const std::string& message) noexcept {
        app_log(level, message);                                 // returns right away
      });

  ah2::HandManager manager;
  int exit_code = 0;

  try {
    ah2::HandConfig config{"can0", ah2::HandSide::Right, ah2::HandModel::Gen2};
    ah2::Hand hand = manager.create(config);
    hand.connect();
    hand.run();

    // the control code

  } catch (const ah2::Exception& error) {
    // the SDK does not log a thrown exception, so what() goes here, code included
    app_log(ah2::LogLevel::Error, error.what());
    exit_code = 1;
  }

  manager.destroy_all();   // stops the robot hand and closes the connection
  ah2::flush_log();        // sends out the log that remains
  return exit_code;
}
```

The example declares `manager` outside the `try` so that the shutdown order holds even on a path
that an exception took. If you leave the work to the destructor of `manager`, `flush_log()` runs
before the robot hand shuts down.
