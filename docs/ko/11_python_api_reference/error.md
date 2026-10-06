[← Python API Reference](../11_python_api_reference.md)

# Errors

| Symbol | Kind | Description |
|---|---|---|
| [`ErrorCode`](#enum-errorcode) | `enum.Enum` | 실패 분류 |
| [`Error`](#error) | exception | SDK 호출이 던지는 예외 |

---

## enum `ErrorCode`

```python
class ErrorCode(enum.Enum):
    NONE = 0                    # 성공. Error에는 실리지 않음
    INVALID_ARGUMENT = 1        # 잘못된 config·target·범위        (호출한 코드)
    WRONG_CALL_ORDER = 2        # 상태에 맞지 않는 호출             (호출한 코드)
    INTERFACE_UNAVAILABLE = 3   # interface 없음·down·권한·점유     (장비와 환경)
    COMMUNICATION_LOST = 4      # 전원·배선·CAN·RX 끊김            (장비와 환경)
    HARDWARE_FAULT = 5          # actuator fault·부하, 통신은 정상  (장비와 환경)
    CONTROL_LOOP_FAULT = 6      # 제어·통신 루프 비정상 종료        (SDK에 보고)
    UNEXPECTED_ERROR = 7        # 예상 밖 실패                      (SDK에 보고)
```

분류 기준은 "무엇을 점검해야 하는가"입니다. code별 메시지와 조치는
[Error messages](../15_error_messages.md#2-errorcode-요약)에 있습니다. 메시지 문서의 code 이름은 C++
표기(`InvalidArgument`)이며 Python의 `INVALID_ARGUMENT`와 같은 값입니다.

---

## `Error`

```python
class Error(RuntimeError):
    code: ErrorCode
```

SDK 호출이 실패하면 던지는 예외입니다. `str(error)`는 사람이 읽는 사유이며 code는 포함하지 않습니다.
log에 남길 때는 `error.code.name`을 함께 남기십시오.

```python
try:
    hand.run()
except Error as error:
    print(f"{error.code.name}: {error}")
```

SDK는 `Error`를 던지기 전에 `[exception] <ErrorCode>: <message>`를 error level로 log에 남깁니다.
log의 `<ErrorCode>`도 C++ 표기입니다.

인자의 형태가 맞지 않아 SDK에 전달되기 전에 실패한 호출은 `Error`가 아니라 Python의 `ValueError`
(배열 길이)나 `TypeError`(타입)를 던지며, SDK log에 남지 않습니다.
