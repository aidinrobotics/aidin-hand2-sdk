[← C++ API Reference](../08_cpp_api_reference.md)

# `types/error.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`ErrorKind`](#enum-errorkind) | enum | 실패 분류 |
| [`kind_of()`](#kind_of) | function | 오류 번호가 속한 분류 |
| [`Exception`](#exception--stdruntime_error) | class | public 호출이 던지는 예외 |
| [`error_code`](#namespace-error_code) | namespace | 오류 번호 상수 (`types/error_codes.hpp`) |

---

## enum `ErrorKind`

```cpp
enum class ErrorKind {
  None,                  //      성공. SDK의 Exception에는 실리지 않음
  InvalidArgument,       // E1xx 인자나 설정 값이 유효하지 않음
  WrongCallOrder,        // E2xx 현재 lifecycle에서 허용되지 않는 호출
  InterfaceUnavailable,  // E3xx CAN interface를 열지 못함
  CommunicationLost,     // E4xx CAN 통신 실패(수신 끊김·송신 실패)
  HardwareFault,         // E5xx drive가 요청한 상태에 도달하지 못하거나 fault를 보고함
  HandFaulted,           // E6xx lifecycle이 Faulted라 호출이 거부됨
  ControlLoopFault,      // E7xx SDK thread를 시작하지 못하거나 thread가 예외로 종료됨
  UnexpectedError,       // E9xx 예상하지 못한 예외나 내부 상태 오류
};

constexpr const char* to_string(ErrorKind kind) noexcept;
```

분류 기준은 "무엇을 점검해야 하는가"입니다. 오류 번호의 백의 자리가 분류를 나타내며, 번호별
message와 조치는 [Error messages](../15_error_messages.md#12-code-ranges)에 있습니다.

---

## `kind_of()`

```cpp
constexpr ErrorKind kind_of(std::uint16_t code) noexcept;
```

**Parameters**     ｜ `code` — 오류 번호 (`E401`이면 `401`)<br>
**Returns**        ｜ 백의 자리에 해당하는 `ErrorKind`. 백의 자리가 `0`이면 `None`, `8` 이상이면 `UnexpectedError`

---

## `Exception` : `std::runtime_error`

### `Exception::Exception()`

```cpp
Exception(HandSide side, std::uint16_t code, std::string message);
Exception(std::uint16_t code, std::string message);
Exception(ErrorKind kind, std::string message);
```

**Parameters**     ｜ `side` — 실패한 로봇 핸드 · `code` — 오류 번호 · `kind` — 실패 분류 · `message` — 실패 사유<br>
**Notes**          ｜ 앞의 두 생성자는 SDK가 씁니다. 한 로봇 핸드에 관한 실패는 첫 생성자로 만들고 `what()`이
`[left] [E<code>] <message>`입니다. 로봇 핸드가 없거나 두 손에 걸친 실패는 둘째 생성자로 만들고
`what()`이 `[E<code>] <message>`입니다. 두 경우 모두 `kind()`는 `kind_of(code)`입니다. 셋째 생성자는
application이 자기 실패를 같은 타입으로 던질 때 씁니다. `code()`는 `0`이고 `what()`은 `message`
그대로입니다

---

### `Exception::kind()`

```cpp
[[nodiscard]] ErrorKind kind() const noexcept;
```

**Returns**        ｜ 실패 분류

---

### `Exception::code()`

```cpp
[[nodiscard]] std::uint16_t code() const noexcept;
```

**Returns**        ｜ 오류 번호 (`E401`이면 `401`). application이 둘째 생성자로 만든 예외는 `0`<br>
**Notes**          ｜ 상속된 `what()`에도 `[E401]` 형태로 번호가 들어 있으므로, log에는 `what()`만 남겨도 됩니다.
번호와 비교할 때는 [`error_code`](#namespace-error_code)의 상수를 씁니다

---

## namespace `error_code`

`types/error_codes.hpp`에 있는 오류 번호 상수입니다. [`code()`](#exceptioncode)나
[`Diagnostics`](types_diagnostics.md#diagnostics)의 `fault_code` 필드와 비교할 때 숫자 대신 씁니다.
번호별 message와 조치는 [Error messages](../15_error_messages.md)에 있습니다.

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

다음은 원점을 잡기 전에 command를 보내 거부된 경우에만 [`home()`](hand.md#handhome)을 부르는 예입니다.

```cpp
try {
  hand.set_command(command);
} catch (const ah2::Exception& error) {
  if (error.code() != ah2::error_code::kNotHomed) throw;
  hand.home();
}
```
