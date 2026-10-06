[← Python API Reference](../11_python_api_reference.md)

# Logging

| Symbol | Kind | Description |
|---|---|---|
| [`LogLevel`](#enum-loglevel) | `enum.Enum` | log level |
| [`set_log_level()`](#set_log_level) | function | console sink level |
| [`set_log_to_console()`](#set_log_to_console) | function | console sink on/off |
| [`set_log_to_file()`](#set_log_to_file) | function | rotating file sink |
| [`set_log_callback()`](#set_log_callback) | function | Python 함수로 전달 |
| [`flush_log()`](#flush_log) | function | 남은 log 내보내기 |

설정은 process 전체에 하나이며, 모든 hand의 log에 적용됩니다.

---

## enum `LogLevel`

```python
class LogLevel(enum.Enum):
    TRACE = 0
    DEBUG = 1
    INFO = 2
    WARN = 3
    ERROR = 4
    CRITICAL = 5
    OFF = 6
```

---

## `set_log_level()`

```python
def set_log_level(level: LogLevel) -> None: ...
```

**Parameters**     ｜ `level` — console sink의 최소 level<br>
**Notes**          ｜ 언제든 호출할 수 있지만, 늦게 낮추면 낮추기 전에 걸러진 log는 되살아나지 않으니
다른 sink 설정과 함께 먼저 정하십시오. file sink와 callback은 `DEBUG` 고정이라 영향을 받지 않습니다

---

## `set_log_to_console()`

```python
def set_log_to_console(enabled: bool) -> None: ...
```

**Parameters**     ｜ `enabled` — console sink 사용 여부. console은 `stderr`로 출력합니다<br>
**Notes**          ｜ [`create()`](hand_manager.md#handmanagercreate)보다 먼저 정하십시오

---

## `set_log_to_file()`

```python
def set_log_to_file(path: str | os.PathLike) -> None: ...
```

**Parameters**     ｜ `path` — rotating file 경로 (50 MiB마다 회전, 현재 파일과 이전 파일 5개)<br>
**Notes**          ｜ [`create()`](hand_manager.md#handmanagercreate)보다 먼저 정하십시오. 파일을 열지
못해도 예외가 아니라 warning log입니다

---

## `set_log_callback()`

```python
def set_log_callback(callback: Callable[[LogLevel, str], None] | None) -> None: ...
```

**Parameters**     ｜ `callback` — `(level, message 본문)`을 받는 함수. `None`이면 해제<br>
**Raises**         ｜ `TypeError` — 호출할 수 없는 값<br>
**Notes**          ｜ 동시에 1개만 등록되고 다시 부르면 교체됩니다. `DEBUG` 이상이 전달되며 console·file
sink와 무관합니다. [`create()`](hand_manager.md#handmanagercreate)보다 먼저 정하십시오

callback은 패키지가 소유한 별도의 thread에서 log가 생긴 순서대로 호출됩니다. SDK의 제어·통신
루프에서 직접 호출하지 않으므로 callback에 시간이 걸려도 제어 주기에는 영향이 없지만, 다음 log는
callback이 반환할 때까지 기다립니다.

- callback이 계속 늦어 기다리는 log가 많아지면 일부가 버려질 수 있습니다. 오래 걸리는 처리는
  callback 밖으로 옮기십시오.
- callback에서 발생한 예외는 SDK로 전파되지 않고 표준 오류에 출력됩니다. callback 등록은 유지됩니다.
- callback 안에서 SDK 함수를 호출하지 마십시오. callback은 application의 thread와 동시에 실행됩니다.
- 프로그램이 끝날 때 callback 등록은 자동으로 해제됩니다.

사용 예는 [Logging](../12_python_logging.md#3-callback-sink)에 있습니다.

---

## `flush_log()`

```python
def flush_log() -> None: ...
```

버퍼에 남은 log를 file sink에 쓰고, callback에 아직 전달하지 않은 log도 전달한 뒤 반환합니다. 종료
직전에 호출하십시오.
