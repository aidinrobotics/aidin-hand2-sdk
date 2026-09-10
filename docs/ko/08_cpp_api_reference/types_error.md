[← C++ API Reference](../08_cpp_api_reference.md)

# `types/error.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`ErrorCode`](#enum-errorcode) | enum | 실패 분류 |
| [`Exception`](#exception--stdruntime_error) | class | public 호출이 던지는 예외 |

---

## enum `ErrorCode`

```cpp
enum class ErrorCode {
  None,                  // 성공. Exception에는 실리지 않음
  InvalidArgument,       // 잘못된 config·target·범위        (호출한 코드)
  WrongCallOrder,        // 상태에 맞지 않는 호출             (호출한 코드)
  InterfaceUnavailable,  // interface 없음·down·권한·점유     (장비와 환경)
  CommunicationLost,     // 전원·배선·CAN·RX 끊김            (장비와 환경)
  HardwareFault,         // actuator fault·부하, 통신은 정상  (장비와 환경)
  ControlLoopFault,      // 제어·통신 루프 비정상 종료        (SDK에 보고)
  UnexpectedError,       // 예상 밖 실패, 경계에서 감쌈        (SDK에 보고)
};

constexpr const char* to_string(ErrorCode code) noexcept;
```

분류 기준은 "무엇을 점검해야 하는가"입니다. code별 메시지와 조치는
[Error messages](../15_error_messages.md#2-errorcode-요약)에 있습니다.

---

## `Exception` : `std::runtime_error`

### `Exception::Exception()`

```cpp
Exception(ErrorCode code, std::string message);
```

**Parameters**     ｜ `code` — 실패 분류 · `message` — 사람이 읽는 설명

---

### `Exception::code()`

```cpp
[[nodiscard]] ErrorCode code() const noexcept;
```

**Returns**        ｜ error code<br>
**Notes**          ｜ 상속된 `what()`에는 message만 들어 있고 error code는 붙지 않습니다. log에 `to_string(code())`를 함께
남기십시오
