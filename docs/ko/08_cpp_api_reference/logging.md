[← C++ API Reference](../08_cpp_api_reference.md)

# `logging/logging.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`LogLevel`](#enum-loglevel) | enum | log level |
| [`set_log_level()`](#set_log_level) | function | console sink level |
| [`set_log_to_console()`](#set_log_to_console) | function | console sink on/off |
| [`set_log_to_file()`](#set_log_to_file) | function | rotating file sink |
| [`set_log_callback()`](#set_log_callback) | function | application logger로 중계 |
| [`flush_log()`](#flush_log) | function | 남은 log 내보내기 |
| [`LogCallback`](#type-aliases) | alias | callback 형태 |

---

## enum `LogLevel`

```cpp
enum class LogLevel { Trace, Debug, Info, Warn, Error, Critical, Off };   // spdlog와 1:1
```

---

## `set_log_level()`

```cpp
void set_log_level(LogLevel level);
```

**Parameters**     ｜ `level` — console sink의 최소 level<br>
**Notes**          ｜ 언제든 호출할 수 있지만, 늦게 낮추면 낮추기 전에 걸러진 log는 되살아나지 않으니
다른 sink 설정과 함께 먼저 정하십시오. file sink와 callback은 `Debug` 고정이라 영향을
받지 않습니다

---

## `set_log_to_console()`

```cpp
void set_log_to_console(bool enabled);
```

**Parameters**     ｜ `enabled` — console sink 사용 여부<br>
**Notes**          ｜ [`create()`](hand_manager.md#handmanagercreate)보다 먼저 정하십시오

---

## `set_log_to_file()`

```cpp
void set_log_to_file(const std::string& path);
```

**Parameters**     ｜ `path` — rotating file 경로 (50 MiB마다 회전, 현재 파일과 이전 파일 5개)<br>
**Notes**          ｜ [`create()`](hand_manager.md#handmanagercreate)보다 먼저 정하십시오. 파일을 열지 못해도 exception이 아니라 warning log입니다

---

## `set_log_callback()`

```cpp
void set_log_callback(LogCallback callback);
```

**Parameters**     ｜ `callback` — `(level, message 본문)`을 받는 함수. `nullptr`이면 해제<br>
**Notes**          ｜ 동시에 1개만 등록되고 다시 부르면 교체됩니다. `Debug` 이상이 전달되며 console·file
sink와 무관합니다. [`create()`](hand_manager.md#handmanagercreate)보다 먼저 정하십시오. callback 안에서 지켜야 할 것은
[Logging](../09_cpp_logging.md#3-callback-sink)에 있습니다

---

## `flush_log()`

```cpp
void flush_log();
```

버퍼에 남은 log를 내보냅니다. 종료 직전에 호출하십시오.

---

## Type aliases

```cpp
using LogCallback = std::function<void(LogLevel, const std::string&)>;
```

계약은 [Logging](../09_cpp_logging.md#3-callback-sink)에 있습니다.
