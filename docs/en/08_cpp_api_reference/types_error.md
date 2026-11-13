[← C++ API Reference](../08_cpp_api_reference.md)

# `types/error.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`ErrorKind`](#enum-errorkind) | enum | Failure classification |
| [`kind_of()`](#kind_of) | function | The classification an error code belongs to |
| [`Exception`](#exception--stdruntime_error) | class | What public calls throw |
| [`error_code`](#namespace-error_code) | namespace | Error code constants (`types/error_codes.hpp`) |

## enum `ErrorKind`

```cpp
enum class ErrorKind {
  None,                  //      success. Never carried by an SDK Exception
  InvalidArgument,       // E1xx invalid argument or config value
  WrongCallOrder,        // E2xx call not allowed in the current lifecycle
  InterfaceUnavailable,  // E3xx the CAN interface could not be opened
  CommunicationLost,     // E4xx CAN communication failure (no RX or TX failure)
  HardwareFault,         // E5xx a drive did not reach the requested state, or reported a fault
  HandFaulted,           // E6xx call rejected because the lifecycle is Faulted
  ControlLoopFault,      // E7xx an SDK thread failed to start, or exited on an exception
  UnexpectedError,       // E9xx unexpected exception or internal state error
};

constexpr const char* to_string(ErrorKind kind) noexcept;
```

The classification answers "what should I check". The hundreds digit of an error code gives its
classification, and the messages and remedies for each code are in
[Error messages](../15_error_messages.md#12-code-ranges).

## `kind_of()`

```cpp
constexpr ErrorKind kind_of(std::uint16_t code) noexcept;
```

**Parameters**     ｜ `code` — the error code (`401` for `E401`)<br>
**Returns**        ｜ The `ErrorKind` of the hundreds digit. `None` when the hundreds digit is `0`, `UnexpectedError` when it is `8` or more

## `Exception` : `std::runtime_error`

### `Exception::Exception()`

```cpp
Exception(HandSide side, std::uint16_t code, std::string message);
Exception(std::uint16_t code, std::string message);
Exception(ErrorKind kind, std::string message);
```

**Parameters**     ｜ `side` — the robot hand that failed · `code` — the error code · `kind` — the classification · `message` — the reason<br>
**Notes**          ｜ The SDK uses the first two constructors. A failure about one robot hand comes from the
first, and its `what()` is `[left] [E<code>] <message>`. A failure with no robot hand yet, or one that
spans both hands, comes from the second, and its `what()` is `[E<code>] <message>`. In both, `kind()` is
`kind_of(code)`. An application uses the third one to throw its own failure as the same type. Its
`code()` is `0` and its `what()` is `message` as is

### `Exception::kind()`

```cpp
[[nodiscard]] ErrorKind kind() const noexcept;
```

**Returns**        ｜ The classification

### `Exception::code()`

```cpp
[[nodiscard]] std::uint16_t code() const noexcept;
```

**Returns**        ｜ The error code (`401` for `E401`). `0` for an exception that an application made with the second constructor<br>
**Notes**          ｜ The inherited `what()` already carries the code as `[E401]`, so recording `what()` alone in
your logs is enough. To compare with a code, use a constant of [`error_code`](#namespace-error_code)

---

## namespace `error_code`

The error code constants in `types/error_codes.hpp`. Use them instead of a number when you compare with
[`code()`](#exceptioncode) or with the `fault_code` field of
[`Diagnostics`](types_diagnostics.md#diagnostics). The message and the remedy of each code are in
[Error messages](../15_error_messages.md).

```cpp
namespace aidin_hand2::error_code {
// E1xx InvalidArgument
constexpr std::uint16_t kInterfaceNameEmpty          = 101;
constexpr std::uint16_t kHandSideInvalid             = 102;
constexpr std::uint16_t kHandModelInvalid            = 103;
constexpr std::uint16_t kMaxEffortInvalid            = 104;
constexpr std::uint16_t kPositionControllerInvalid   = 105;
constexpr std::uint16_t kImpedanceStiffnessInvalid   = 106;
constexpr std::uint16_t kImpedanceCutoffInvalid      = 107;

// E2xx WrongCallOrder
constexpr std::uint16_t kAlreadyConnected            = 201;
constexpr std::uint16_t kNotConnected                = 202;
constexpr std::uint16_t kHomingInterrupted           = 203;
constexpr std::uint16_t kNotRunning                  = 204;
constexpr std::uint16_t kNotHomed                    = 205;
constexpr std::uint16_t kNotFaulted                  = 206;
constexpr std::uint16_t kHandDestroyed               = 207;

// E3xx InterfaceUnavailable
constexpr std::uint16_t kInterfaceNotFound           = 301;
constexpr std::uint16_t kSocketPermissionDenied      = 302;
constexpr std::uint16_t kCanNotSupported             = 303;
constexpr std::uint16_t kCanFdNotConfigured          = 304;
constexpr std::uint16_t kInterfaceDown               = 305;
constexpr std::uint16_t kInterfaceOpenFailed         = 306;
constexpr std::uint16_t kOtherMaster                 = 307;

// E4xx CommunicationLost
constexpr std::uint16_t kNoFirstFrame                = 401;
constexpr std::uint16_t kLinkFailed                  = 402;
constexpr std::uint16_t kQuickStopUnconfirmed        = 403;
constexpr std::uint16_t kReceiveLost                 = 404;
constexpr std::uint16_t kTransmitFailed              = 405;
constexpr std::uint16_t kAutoReconnectTimeout        = 406;

// E5xx HardwareFault
constexpr std::uint16_t kEnableTimeout               = 501;
constexpr std::uint16_t kHomingFailed                = 502;
constexpr std::uint16_t kQuickStopTimeout            = 503;
constexpr std::uint16_t kActuatorFault               = 504;

// E6xx HandFaulted
constexpr std::uint16_t kHandFaulted                 = 601;

// E7xx ControlLoopFault
constexpr std::uint16_t kThreadStartFailed           = 701;
constexpr std::uint16_t kControlLoopTerminated       = 702;
constexpr std::uint16_t kEventLoggingTerminated      = 703;

// E9xx UnexpectedError
constexpr std::uint16_t kUnexpectedException         = 901;
constexpr std::uint16_t kInternalState               = 902;
}
```

The following calls [`home()`](hand.md#handhome) only when a command was refused because the hand has
no origin yet.

```cpp
try {
  hand.set_command(command);
} catch (const ah2::Exception& error) {
  if (error.code() != ah2::error_code::kNotHomed) throw;
  hand.home();
}
```
