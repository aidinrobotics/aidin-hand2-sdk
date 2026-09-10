[← C++ API Reference](../08_cpp_api_reference.md)

# `logging/logging.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`LogLevel`](#enum-loglevel) | enum | Log level |
| [`set_log_level()`](#set_log_level) | function | Console sink level |
| [`set_log_to_console()`](#set_log_to_console) | function | Console sink on/off |
| [`set_log_to_file()`](#set_log_to_file) | function | Rotating file sink |
| [`set_log_callback()`](#set_log_callback) | function | Forward to an application logger |
| [`flush_log()`](#flush_log) | function | Push out what is buffered |
| [`LogCallback`](#type-aliases) | alias | The callback shape |

## enum `LogLevel`

```cpp
enum class LogLevel { Trace, Debug, Info, Warn, Error, Critical, Off };   // one to one with spdlog
```

## `set_log_level()`

```cpp
void set_log_level(LogLevel level);
```

**Parameters**     ｜ `level` — the minimum level of the console sink<br>
**Notes**          ｜ Callable at any time, but lowering it late does not bring back lines
already filtered
out, so decide it first along with the other sinks. The file sink and the callback are fixed at
`Debug`
and are
unaffected

## `set_log_to_console()`

```cpp
void set_log_to_console(bool enabled);
```

**Parameters**     ｜ `enabled` — whether to use the console sink<br>
**Notes**          ｜ Decide before [`create()`](hand_manager.md#handmanagercreate)

## `set_log_to_file()`

```cpp
void set_log_to_file(const std::string& path);
```

**Parameters**     ｜ `path` — the rotating file path (rotates every 50 MiB, five rotated files plus
the current one)<br>
**Notes**          ｜ Decide before [`create()`](hand_manager.md#handmanagercreate). Failing to open the file
leaves a warning log, not an exception

## `set_log_callback()`

```cpp
void set_log_callback(LogCallback callback);
```

**Parameters**     ｜ `callback` — takes `(level, the message body)`. `nullptr` clears it<br>
**Notes**          ｜ The SDK keeps one at a time and another call replaces it. Everything
from `Debug`
up is delivered, independently of the console and file sinks. Set it before [`create()`](hand_manager.md#handmanagercreate).
What the callback has to respect is in [Logging](../09_cpp_logging.md#3-callback-sink)

## `flush_log()`

```cpp
void flush_log();
```

Pushes out what is left in the buffer. Call it right before shutdown.

## Type aliases

```cpp
using LogCallback = std::function<void(LogLevel, const std::string&)>;
```

The contract is in [Logging](../09_cpp_logging.md#3-callback-sink).

---
