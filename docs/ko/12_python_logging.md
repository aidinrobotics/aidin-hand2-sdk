# Logging (Python)

logging은 SDK가 남기는 log를 console·file·callback 세 곳으로 보내는 기능입니다. 이 문서는 Python에서
세 sink를 설정하는 방법과 SDK log를 Python의 `logging` 모듈로 넘기는 방법을 설명합니다. log 한 줄의
구성과 파일 회전 같은 공통 동작은 C++ SDK와 같으며, 해당 절로 연결합니다.

## Contents

&nbsp;&nbsp;[**1. Sinks**](#1-sinks)<br>
&nbsp;&nbsp;[**2. File sink**](#2-file-sink)<br>
&nbsp;&nbsp;[**3. Callback sink**](#3-callback-sink)<br>
&nbsp;&nbsp;[**4. When to configure**](#4-when-to-configure)<br>
&nbsp;&nbsp;[**5. Log record**](#5-log-record)<br>
&nbsp;&nbsp;[**6. Shutdown**](#6-shutdown)<br>
&nbsp;&nbsp;[**7. Full example**](#7-full-example)

## 1. Sinks

sink는 log가 나가는 출구이며 세 종류입니다. 아무것도 설정하지 않으면 console로만 출력됩니다.

| Sink | Default | Level |
|---|---|---|
| console | 켜짐, `stderr`로 출력 | `INFO` |
| file | 꺼짐 | `DEBUG` 고정 |
| callback | 등록 없음 | `DEBUG` 고정 |

다음은 console sink의 level을 낮추고 출력을 끄는 예입니다.

```python
import aidin_hand2 as ah2

ah2.set_log_level(ah2.LogLevel.DEBUG)   # console sink에만 적용됩니다
ah2.set_log_to_console(False)           # console 출력을 끕니다
```

file sink와 callback sink의 level은 `DEBUG`로 고정이라
[`set_log_level()`](11_python_api_reference/logging.md#set_log_level)의 영향을 받지 않습니다.
console 출력을 계속 꺼 두려면 `LogLevel.OFF`가 아니라
[`set_log_to_console()`](11_python_api_reference/logging.md#set_log_to_console)에 `False`를
넘기십시오. `OFF`는 level 설정이라 `set_log_level()`을 다시 호출하면 출력이 다시 나옵니다.

SDK의 console 출력은 Python의 `sys.stderr`를 거치지 않고 process의 표준 오류로 바로 나갑니다.
`sys.stderr`를 바꿔도 SDK log는 따라가지 않으므로, Python 쪽에서 받으려면 callback sink를 쓰십시오.

## 2. File sink

file sink는 path를 지정한 시점부터 log를 파일에도 기록합니다.

```python
ah2.set_log_to_file("/var/log/my_robot/aidin-hand2-left.log")
```

디렉터리는 미리 만들고 쓰기 권한을 부여하십시오. 파일은 50 MiB마다 회전하고 현재 파일과 이전 파일
5개를 남깁니다. 회전 방식과 보관 기간은 [C++ logging](09_cpp_logging.md#2-file-sink)과 같습니다.

> [!WARNING]
> 파일을 열지 못해도 예외가 아니라 warning 한 줄로 끝납니다. console까지 꺼 두고 callback도
> 등록하지 않았다면 아무 신호도 남지 않습니다. 프로그램을 시작한 뒤 파일이 생겼는지 확인하십시오.

## 3. Callback sink

callback sink는 SDK log를 Python 함수로 넘깁니다. 다음은 Python의 `logging` 모듈로 넘기는 예입니다.

```python
import logging
import aidin_hand2 as ah2

logger = logging.getLogger("aidin_hand2")

LEVELS = {
    ah2.LogLevel.TRACE: logging.DEBUG,
    ah2.LogLevel.DEBUG: logging.DEBUG,
    ah2.LogLevel.INFO: logging.INFO,
    ah2.LogLevel.WARN: logging.WARNING,
    ah2.LogLevel.ERROR: logging.ERROR,
    ah2.LogLevel.CRITICAL: logging.CRITICAL,
}

def forward(level, message):
    logger.log(LEVELS[level], message)

ah2.set_log_to_console(False)           # 같은 log가 두 번 출력되지 않도록 console을 끕니다
ah2.set_log_callback(forward)
```

ROS 2 node의 logger로 넘길 때도 같은 방법으로 level을 대응시킵니다.

| Item | Value |
|---|---|
| Level | `DEBUG` 이상 전부 |
| Arguments | `(level, message 본문)` |
| Count | 1개. 다시 호출하면 교체, `None`이면 해제 |
| Other sinks | console·file sink와 무관 |

callback이 받는 본문은 [5. Log record](#5-log-record)의 message 부분이라 `[left]`와 `[cycle N]`도
함께 전달됩니다.

callback은 패키지가 소유한 별도의 thread에서 log가 생긴 순서대로 호출됩니다. SDK의 제어·통신
루프에서 직접 호출하지 않으므로 callback에 시간이 걸려도 제어 주기에는 영향이 없습니다. 다만 다음
log는 callback이 반환할 때까지 기다리고, 기다리는 log가 계속 늘면 일부가 버려질 수 있습니다.

- callback 안에서 SDK 함수를 호출하지 마십시오. callback은 application의 thread와 동시에 실행됩니다.
- callback에서 발생한 예외는 SDK로 전파되지 않고 표준 오류에 출력되며, 등록은 유지됩니다.
- `logging` 모듈은 thread-safe하므로 위 예처럼 바로 넘겨도 됩니다.

## 4. When to configure

sink는 [`create()`](11_python_api_reference/hand_manager.md#handmanagercreate)보다 먼저
결정하십시오. 그러지 않으면 `create()`가 남기는 경고(`disabled_actuators` 필드의 범위 밖 index)를
놓칩니다.

```python
ah2.set_log_level(ah2.LogLevel.INFO)
ah2.set_log_to_console(True)
ah2.set_log_to_file("/var/log/my_robot/hand.log")
ah2.set_log_callback(forward)

with ah2.HandManager() as manager:
    hand = manager.create(config)
    hand.connect()
```

[`connect()`](11_python_api_reference/hand.md#handconnect) 뒤에는 여러 thread가 동시에 log를
만듭니다. 시작할 때 한 번 결정하고 이후에는 바꾸지 마십시오.

## 5. Log record

log 한 줄의 구성은 C++ SDK와 같습니다. 각 조각의 뜻은
[C++ logging](09_cpp_logging.md#5-log-record)에 있습니다.

```text
[1756713600.123456789] [aidin_hand2] [info]     [left] [cycle 12345] homing complete — home position established
```

`Error`를 던지는 자리에는 error level로 `[exception] <ErrorCode>: <message>` 형태가 남습니다.
`<ErrorCode>`는 C++ 표기(`CommunicationLost`)이며 Python의 `ErrorCode.COMMUNICATION_LOST`와 같은
값입니다.

`str(error)`에는 code가 들어 있지 않습니다. application의 log에는 code를 함께 남기십시오.

```python
try:
    hand.run()
except ah2.Error as error:
    logger.error("SDK %s: %s", error.code.name, error)
```

남은 오류 문구로 원인을 좁히려면 [Error messages](15_error_messages.md)를 보십시오.

> [!IMPORTANT]
> 안전 판단은 log에 의존하지 말고 `lifecycle` 필드와 actuator fault를 직접 확인하십시오. 방법은
> [Python guide](10_python_usage_guide.md#8-diagnostics)에 있습니다.

## 6. Shutdown

종료는 로봇 핸드를 먼저 정지시키고 log를 flush하는 순서입니다. `with` 블록이 끝나면 로봇 핸드가
정지하므로, [`flush_log()`](11_python_api_reference/logging.md#flush_log)는 블록 밖에서
호출합니다.

```python
with ah2.HandManager() as manager:
    ...
ah2.flush_log()
```

`flush_log()`는 callback에 아직 전달하지 않은 log까지 전달한 뒤 반환합니다. 로봇 핸드를 정지시키는
함수가 아니므로 순서를 바꾸지 마십시오.

## 7. Full example

다음은 세 sink를 모두 설정하고 예외까지 남긴 뒤 순서대로 종료하는 예입니다.

```python
import logging
import sys
import aidin_hand2 as ah2

logging.basicConfig(level=logging.INFO, format="[%(name)s] %(levelname)s %(message)s")
logger = logging.getLogger("my_robot")
sdk_logger = logging.getLogger("aidin_hand2")

LEVELS = {
    ah2.LogLevel.TRACE: logging.DEBUG,
    ah2.LogLevel.DEBUG: logging.DEBUG,
    ah2.LogLevel.INFO: logging.INFO,
    ah2.LogLevel.WARN: logging.WARNING,
    ah2.LogLevel.ERROR: logging.ERROR,
    ah2.LogLevel.CRITICAL: logging.CRITICAL,
}

# sink는 create()보다 먼저 결정합니다
ah2.set_log_to_console(False)                                   # Python logging으로 출력합니다
ah2.set_log_to_file("/var/log/my_robot/aidin-hand2.log")        # DEBUG 이상 전부
ah2.set_log_callback(lambda level, message: sdk_logger.log(LEVELS[level], message))

exit_code = 0
try:
    with ah2.HandManager() as manager:
        hand = manager.create(ah2.HandConfig("can0", ah2.HandSide.RIGHT))
        hand.connect()
        hand.run()

        # 제어 코드

except ah2.Error as error:
    # SDK는 이미 error level로 남겼고, application 쪽에는 code를 함께 남깁니다
    logger.error("SDK %s: %s", error.code.name, error)
    exit_code = 1

ah2.flush_log()          # with 블록이 로봇 핸드를 정지시킨 뒤 남은 log를 내보냅니다
sys.exit(exit_code)
```
