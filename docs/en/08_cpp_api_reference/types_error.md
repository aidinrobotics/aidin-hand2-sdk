[← C++ API Reference](../08_cpp_api_reference.md)

# `types/error.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`ErrorCode`](#enum-errorcode) | enum | Failure classification |
| [`Exception`](#exception--stdruntime_error) | class | What public calls throw |

## enum `ErrorCode`

```cpp
enum class ErrorCode {
  None,                  // success. Never carried by an Exception
  InvalidArgument,       // bad config, target, or range        (the calling code)
  WrongCallOrder,        // call does not match the state       (the calling code)
  InterfaceUnavailable,  // interface missing, down, busy       (device and environment)
  CommunicationLost,     // power, wiring, CAN, RX lost         (device and environment)
  HardwareFault,         // drive fault or load, link is fine   (device and environment)
  ControlLoopFault,      // the control and communication loop terminated abnormally  (report to the SDK)
  UnexpectedError,       // unexpected failure, wrapped at the boundary  (report to the SDK)
};

constexpr const char* to_string(ErrorCode code) noexcept;
```

The classification answers "what should I check". The messages and remedies for each code are in
[Error messages](../15_error_messages.md#2-errorcode-summary).

## `Exception` : `std::runtime_error`

### `Exception::Exception()`

```cpp
Exception(ErrorCode code, std::string message);
```

**Parameters**     ｜ `code` — the classification · `message` — a human-readable description

### `Exception::code()`

```cpp
[[nodiscard]] ErrorCode code() const noexcept;
```

**Returns**        ｜ The error code<br>
**Notes**          ｜ The inherited `what()` carries only the message, not the error code. Record
`to_string(code())` with it in
your logs

---
