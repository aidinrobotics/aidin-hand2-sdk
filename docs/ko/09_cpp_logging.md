# Logging

logging은 SDK가 남기는 log를 console·file·callback 세 곳으로 보내는 기능입니다. 이 문서는 세 sink를
설정하는 방법과 log 한 줄을 읽는 방법을 설명합니다. state와 diagnostics를 읽는 방법은
[C++ guide](07_cpp_usage_guide.md#7-handstate)에 있습니다.

## Contents

&nbsp;&nbsp;[**1. Sinks**](#1-sinks)<br>
&nbsp;&nbsp;[**2. File sink**](#2-file-sink)<br>
&nbsp;&nbsp;[**3. Callback sink**](#3-callback-sink)<br>
&nbsp;&nbsp;[**4. When to configure**](#4-when-to-configure)<br>
&nbsp;&nbsp;[**5. Log record**](#5-log-record)<br>
&nbsp;&nbsp;[**6. Shutdown**](#6-shutdown)<br>
&nbsp;&nbsp;[**7. Full example**](#7-full-example)

## 1. Sinks

sink는 log가 전달되는 대상입니다. SDK는 console, file, callback 세 가지를 제공하며, 아무것도 설정하지 않으면
log는 console로만 전달됩니다.

| Sink | Default | Level |
|---|---|---|
| console | 켜짐, `stderr`로 출력 | `Info` |
| file | 꺼짐 | `Debug` 고정 |
| callback | 등록 없음 | `Debug` 고정 |

다음은 console sink의 level을 낮추고 출력을 끄는 예입니다.

```cpp
namespace ah2 = aidin_hand2;

ah2::set_log_level(ah2::LogLevel::Debug);   // console sink에만 적용됩니다
ah2::set_log_to_console(false);             // console 출력을 끕니다
```

file sink와 callback sink의 level은 `Debug`로 고정이라
[`set_log_level()`](08_cpp_api_reference/logging.md#set_log_level)의 영향을 받지 않습니다.
[`LogLevel`](08_cpp_api_reference/logging.md#enum-loglevel) enum의 `Off`로도 console 출력이
멈추지만, `Off`는 level 설정이라 `set_log_level()`을 다시 호출하면 출력이 다시 나옵니다. 계속 꺼
두려면 [`set_log_to_console()`](08_cpp_api_reference/logging.md#set_log_to_console)에 `false`를
넘기십시오. `Trace`도 고를 수 있지만 SDK가 `Trace` level로 남기는 log가 없어 `Debug`와 같습니다.

## 2. File sink

file sink는 path를 지정한 시점부터 log를 파일에도 기록합니다.

```cpp
ah2::set_log_to_file("/var/log/my_robot/aidin-hand2-left.log");
```

디렉터리는 배포 과정에서 미리 만들고 쓰기 권한을 부여하십시오.

```bash
sudo install -d -o robot -g robot -m 0750 /var/log/my_robot
```

| Item | Value |
|---|---|
| Rotation | 50 MiB마다 새 파일. 현재 파일과 이전 파일 5개를 합쳐 최대 300 MiB |
| Level | `Debug` 이상 |
| Flush | `Warn` 이상은 즉시. 나머지는 약 1초 주기 |

현재 파일이 50 MiB에 도달하면 이전 파일로 밀려나고 번호가 하나씩 올라갑니다. `hand.log`를
지정하면 `hand.1.log`부터 `hand.5.log`까지가 이전 파일이고, 번호가 큰 쪽이 더 오래된 기록이며
`hand.5.log`를 넘어간 기록은 삭제됩니다.

총량을 300 MiB로 묶은 이유는 장기 운전에서 disk가 차는 것을 막기 위해서입니다. SDK는 매 cycle
기록하지 않고 상태 전이와 사건이 있을 때만 남기므로, 300 MiB를 소모하는 속도는 운전 시간이 아니라
사건 빈도가 결정합니다. 사건이 없으면 파일이 전혀 커지지 않고, 통신 오류나 actuator fault가
반복되는 구간에서는 약 25분 만에 가장 오래된 파일을 지우고 처음부터 다시 씁니다. 그보다 오래 보관해야 하면 이전 파일을 밖으로
옮기는 정책을 배포에 넣으십시오.

> [!WARNING]
> 파일을 열지 못해도 예외가 아니라 warning 한 줄로 종료됩니다. console까지 꺼 두고 callback도
> 등록하지 않았다면 아무 신호도 남지 않습니다. process를 띄운 뒤 파일이 생겼는지, 커지고 있는지
> health check로 확인하십시오.

## 3. Callback sink

callback sink는 SDK log를 application logger나 ROS 2 logging으로 넘길 때 사용합니다.

```cpp
ah2::set_log_callback(
    [](ah2::LogLevel level, const std::string& message) {
      app_logger.write(map_level(level), message);
    });
```

| Item | Value |
|---|---|
| Level | `Debug` 이상 전부 |
| Arguments | `(level, message 본문)` |
| Count | 1개. 다시 호출하면 교체, `nullptr`이면 해제 |
| Other sinks | console·file sink와 무관 |

callback이 받는 본문은 [5. Log record](#5-log-record)의 message 부분이라 `[left]`와 오류 번호도 함께
전달됩니다.

callback은 log를 만든 thread에서 동기적으로 호출되고, 그 thread에는 제어·통신 루프도 포함됩니다.
callback에 드는 시간은 제어 주기 안에서 소모되므로, block I/O를 하거나 SDK를 다시 호출하거나 예외를
밖으로 던지지 마십시오. 다음은 queue에 넣고 즉시 반환하는 예입니다.

```cpp
ah2::set_log_to_console(false);
ah2::set_log_callback(
    [&queue](ah2::LogLevel level, const std::string& message) noexcept {
      queue.try_push(level, message);
    });
```

callback이 늦게 반환하면 제어·통신 루프의 사건이 log로 옮겨지기 전에 쌓입니다. 쌓인 양이 한도를
넘으면 SDK는 일부 사건을 log에 남기지 못하고 `<count> realtime events dropped (event ring full)`
warning을 남깁니다.

## 4. When to configure

sink는 [`create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate)보다 먼저 결정하십시오.
그러지 않으면 `create()`가 남기는 경고(`disabled_actuators` 필드의 범위 밖 index)를 놓칩니다.

다음은 세 sink를 모두 정한 뒤 hand를 만드는 순서입니다.

```cpp
ah2::set_log_level(ah2::LogLevel::Info);
ah2::set_log_to_console(true);
ah2::set_log_to_file("/var/log/my_robot/hand.log");
ah2::set_log_callback(callback);

ah2::Hand hand = manager.create(config);
hand.connect();
```

[`connect()`](08_cpp_api_reference/hand.md#handconnect) 뒤에는 여러 thread가 동시에 log를
만듭니다. SDK는 sink 교체와 log 기록을 서로 맞춰 주지 않으므로, production에서는 시작할 때 한 번
결정하고 이후 변경하지 마십시오.

## 5. Log record

log 한 줄은 다음 조각으로 이루어집니다.

```text
[1756713600.123456789] [aidin_hand2] [info]     [left] homing complete — home position established
[1756713600.223456789] [aidin_hand2] [critical] [left] [E404] communication lost (no RX)
```

| Piece | Description |
|---|---|
| `[epoch.nanosecond]` | wall clock |
| `[aidin_hand2]` | logger 이름 |
| `[info]` | level. console sink는 10칸으로 맞추고 색을 입힘 |
| `[left]` | 로봇 핸드 좌우 구분. hand context가 있는 줄에만 붙음 |
| `[E404]` | 오류 번호. [Error messages](15_error_messages.md)에 원인과 조치가 있는 줄에만 붙음 |
| message | 본문 |

`[left]`와 오류 번호는 formatter pattern이 아니라 message 본문에 들어가고, 붙지 않는 줄도 있습니다.
사건이 생긴 시각은 앞의 wall clock으로 확인합니다. 제어·통신 루프의 사건은 50 ms마다 모아서 쓰므로,
이 시각은 사건보다 최대 50 ms 늦을 수 있습니다.

[`connect()`](08_cpp_api_reference/hand.md#handconnect)가 성공하면 info level로 여러 줄짜리 Connection
Info를 남깁니다. 이 실행에 쓴 설정과 제어·통신 루프가 받은 실행 환경을 한 곳에서 확인할 수 있습니다.
아래 예에는 message 부분만 실었습니다.

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
| `side`·`model`·`interface` | `HandConfig`의 `hand_side`·`hand_model`·`interface_name` 필드 |
| `rate` | 제어·통신 루프의 주기. 500 Hz로 고정 |
| `auto_home` | `auto_home` 필드 |
| `scheduler` | 제어·통신 루프의 `SCHED_FIFO` 우선순위. 권한이 없으면 `SCHED_OTHER` |
| `cpu` | 제어·통신 루프가 실행 중인 CPU |
| `affinity` | `rt_cpu_affinity` 필드. `-1`이면 `none` |
| `memory` | 제어·통신 루프의 메모리를 잠갔으면 `locked`, 못 했으면 `not locked` |
| `actuators` | 전체 actuator 수 |
| `disabled` | `disabled_actuators` 필드로 끈 actuator 수 |
| `faulted` | 연결 시점에 fault 상태인 actuator 수 |
| `sdk` | SDK 버전 |

SDK log에는 호출과 상관없이 생긴 사건이 남습니다. 제어·통신 루프에서 생긴 사건은 오류 번호를 붙여
남습니다. 아래 예에는 message 부분만 실었고, 실제 줄에는 위 표의 앞 조각이 그대로 붙습니다.

```text
[left] [E404] communication lost (no RX)
```

함수가 던진 예외는 SDK log에 남지 않으므로 application 쪽 log에 `what()`을 한 번 남기십시오.
[`Exception::what()`](08_cpp_api_reference/types_error.md#exception--stdruntime_error)에도 `[E601]` 같은
오류 번호가 들어 있어서, SDK log와 같은 번호로 맞춰 볼 수 있습니다.

```cpp
try {
  // SDK 호출
} catch (const ah2::Exception& error) {
  app_logger.error("SDK {}", error.what());
}
```

번호별 원인과 조치는 [Error messages](15_error_messages.md)에 있습니다. 사고 원인을 추적하기 위해
무엇을 함께 남겨야 하는지는 [C++ guide](07_cpp_usage_guide.md#82-diagnostic-record)에 정리돼 있습니다.

> [!IMPORTANT]
> 안전 판단은 log에 의존하지 말고 [`lifecycle`](07_cpp_usage_guide.md#8-diagnostics) 필드와
> [actuator fault](15_error_messages.md#10-actuator-fault-values)를 직접 확인하십시오. SDK 밖에 둔 안전
> 회로도 마찬가지입니다.

## 6. Shutdown

종료는 로봇 핸드를 먼저 정지시키고 log를 flush하는 순서입니다.

```cpp
hand.stop();
manager.destroy(hand);
ah2::flush_log();
```

[`flush_log()`](08_cpp_api_reference/logging.md#flush_log)는 로봇 핸드를 정지시키는 함수가
아닙니다. 순서를 바꾸지 마십시오.

## 7. Full example

다음은 세 sink를 모두 설정하고 예외까지 남긴 뒤 순서대로 종료하는 예입니다. logging 설정을
[`create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate)보다 앞에 두고,
[`flush_log()`](08_cpp_api_reference/logging.md#flush_log)를 로봇 핸드 종료 뒤에 두는 순서를
그대로 담았습니다.

```cpp
#include <iostream>
#include <string>
#include <aidin_hand2/aidin_hand2.hpp>

namespace ah2 = aidin_hand2;

// ------------------------------ Application ---------------------------------
// 여기서는 표준 출력에 그대로 찍습니다. application logger로 넘기려면
// level을 그 logger의 level로 변환해 전달하십시오

void app_log(ah2::LogLevel level, const std::string& message)
{
  std::cout << "[app] [" << static_cast<int>(level) << "] " << message << '\n';
}

// -------------------------------- Logging -----------------------------------

int main()
{
  // sink는 create()보다 먼저 결정합니다
  ah2::set_log_level(ah2::LogLevel::Info);                       // console sink에만 적용
  ah2::set_log_to_console(true);
  ah2::set_log_to_file("/var/log/my_robot/aidin-hand2.log");     // Debug 이상 전부
  ah2::set_log_callback(
      [](ah2::LogLevel level, const std::string& message) noexcept {
        app_log(level, message);                                 // 즉시 반환합니다
      });

  ah2::HandManager manager;
  int exit_code = 0;

  try {
    ah2::HandConfig config{"can0", ah2::HandSide::Right, ah2::HandModel::Gen2};
    ah2::Hand hand = manager.create(config);
    hand.connect();
    hand.run();

    // 제어 코드

  } catch (const ah2::Exception& error) {
    // SDK는 던진 예외를 log에 남기지 않으므로, 오류 번호가 붙은 what()을 여기서 남깁니다
    app_log(ah2::LogLevel::Error, error.what());
    exit_code = 1;
  }

  manager.destroy_all();   // 로봇 핸드를 정지시키고 연결을 끊습니다
  ah2::flush_log();        // 남은 log를 내보냅니다
  return exit_code;
}
```

`manager`를 `try` 밖에 선언한 이유는 예외로 빠져나온 경로에서도 종료 순서를 지키기 위해서입니다.
`manager`의 소멸자에 맡기면 `flush_log()`가 로봇 핸드 종료보다 먼저 실행됩니다.
