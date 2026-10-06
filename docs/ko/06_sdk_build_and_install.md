# SDK build & install

SDK는 C++ 라이브러리와 Python 패키지 두 가지로 제공합니다. 사용할 언어의 장만 따르십시오. 두 장은
서로 독립적이므로 다른 장을 거칠 필요가 없습니다.

| Language | Chapter | Result |
|---|---|---|
| C++ (C++ application, ROS 2 wrapper) | [1. C++](#1-c) | SDK를 빌드해 `/usr/local`에 설치 |
| Python | [2. Python](#2-python) | 미리 빌드된 패키지를 pip으로 설치 |

어느 쪽이든 로봇 핸드를 움직이려면 먼저 [Real-time kernel setup](04_real_time_kernel_setup.md)과
[CAN-FD setup](05_can_fd_setup.md)을 마치십시오.

## Contents

&nbsp;&nbsp;[**1. C++**](#1-c)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Install dependencies](#11-install-dependencies)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Build](#12-build)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 Install and link](#13-install-and-link)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.4 Test program](#14-test-program)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.5 Uninstall](#15-uninstall)<br>
&nbsp;&nbsp;[**2. Python**](#2-python)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 Install](#21-install)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 Verify](#22-verify)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 When pip finds no package](#23-when-pip-finds-no-package)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.4 Uninstall](#24-uninstall)

## 1. C++

SDK를 빌드·테스트·설치하고 예제로 동작을 확인합니다. 사용법은 [C++ guide](07_cpp_usage_guide.md)에
있습니다.

### 1.1 Install dependencies

Ubuntu 22.04 / 24.04를 지원합니다. 두 배포판의 apt 패키지가 요구 버전을 충족하므로
버전을 따로 지정할 필요는 없습니다.

```bash
sudo apt update
sudo apt install -y build-essential cmake libspdlog-dev can-utils
```

최소 요구는 spdlog 1.9와 C++17 컴파일러입니다. Eigen은 kinematics 구현에만 쓰이고 그 부분이
미리 빌드되어 배포되므로 설치하지 않아도 됩니다.

### 1.2 Build

이후 명령은 모두 SDK repo 루트에서 실행합니다.

```bash
cd aidin-hand2-sdk
```

> [!IMPORTANT]
> **AIDIN Hand Gen2에는 hand type A·B·C가 있고, 빌드할 때 type을 선택합니다.** type에 따라
> kinematics가 다르게 계산됩니다. type은 로봇 핸드 전달과 함께 알려드립니다.

아래 셋 중 해당하는 쪽만 실행하십시오.

**a) type A**

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
#   -- aidin_hand2: hand type = a
#   -- aidin_hand2: kinematics = .../libaidin_hand2_kinematics_type_a.a
```

**b) type B**

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DAIDIN_HAND2_HAND_TYPE=b
#   -- aidin_hand2: hand type = b
#   -- aidin_hand2: kinematics = .../libaidin_hand2_kinematics_type_b.a
```

**c) type C**

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DAIDIN_HAND2_HAND_TYPE=c
#   -- aidin_hand2: hand type = c
#   -- aidin_hand2: kinematics = .../libaidin_hand2_kinematics_type_c.a
```

출력 두 줄로 선택된 값을 확인한 뒤 빌드합니다.

```bash
cmake --build cpp/build -j"$(nproc)"
```

선택적으로 코드 로직 테스트를 실행할 수 있습니다.

```bash
ctest --test-dir cpp/build --output-on-failure
```

실패하면 install로 넘어가지 마십시오. 출력 로그와 아래 환경 정보를
[issue](https://github.com/aidinrobotics/aidin-hand2-sdk/issues)에 그대로 붙이면 됩니다. 별도
설명은 없어도 됩니다.

```bash
lsb_release -d; uname -srm; gcc --version | head -1; cmake --version | head -1
```

### 1.3 Install and link

설치되는 것은 공개 header, shared library 하나, CMake package config입니다.

두 방법 중 **하나만** 따르십시오. `/usr/local`이 기본이고, `sudo`를 쓸 수 없을 때만 b)로
가십시오.

**a) `/usr/local`에 설치 (권장)**

`/usr/local`은 CMake의 기본 탐색 경로이면서 loader가 캐시에 담는 경로이므로, application 쪽에
경로를 지정할 일이 없습니다.

```bash
sudo cmake --install cpp/build
sudo ldconfig
```

`ldconfig`는 shared library를 loader 캐시에 등록합니다. 빠뜨리면 실행 시점에 라이브러리를 찾지
못합니다.

application의 CMakeLists.txt는 두 줄입니다. 버전을 반드시 지정하십시오. SDK는 minor가 다르면
호환되지 않는데, 지정하지 않으면 CMake가 버전 검사를 건너뜁니다.

```cmake
find_package(aidin_hand2 0.7 REQUIRED)
target_link_libraries(your_app PRIVATE aidin_hand2::aidin_hand2)
```

**b) 사용자 prefix에 설치**

`sudo`를 쓸 수 없을 때만 씁니다. `ldconfig`는 필요하지 않지만, 대신 application 쪽에서 경로 두
가지를 챙겨야 합니다.

```bash
cmake --install cpp/build --prefix "$HOME/.local"
```

application의 CMakeLists.txt에 prefix를 알려 줍니다.

```cmake
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local")
find_package(aidin_hand2 0.7 REQUIRED)
target_link_libraries(your_app PRIVATE aidin_hand2::aidin_hand2)
```

application을 install한다면 그 시점에 CMake가 RUNPATH를 제거하므로 `INSTALL_RPATH`를 지정해야
합니다. 지정하지 않으면 설치된 application이 SDK를 찾지 못합니다.

```cmake
set_target_properties(your_app PROPERTIES INSTALL_RPATH "$ENV{HOME}/.local/lib")
```

> [!WARNING]
> a)와 b)를 **함께 쓰지 마십시오.** 나중에 `/usr/local`로 옮긴다면 이전 prefix의 SDK를 먼저
> 지우십시오. 두 곳에 남아 있으면 `find_package`가 어느 쪽을 찾을지 정해지지 않고, 서로 다른
> 릴리스의 라이브러리가 한 프로세스에 올라갈 수 있습니다.
>
> ```bash
> rm -rf "$HOME/.local/lib/libaidin_hand2"* "$HOME/.local/lib/cmake/aidin_hand2" \
>        "$HOME/.local/include/aidin_hand2"
> ```
>
> 어느 prefix에 무엇이 있는지는 이렇게 확인합니다.
>
> ```bash
> grep -m1 'set(PACKAGE_VERSION "' <prefix>/lib/cmake/aidin_hand2/aidin_hand2ConfigVersion.cmake
> ```

### 1.4 Test program

1.2절에서 빌드하면 예제 실행 파일 `cpp/build/basic_control`이 생깁니다. 연결 → homing →
joint를 0번부터 순서대로 하나씩 50° 움직였다 복귀(abduction 제외) → 정지 순으로 동작을
확인합니다.

먼저 어느 interface에 어느 손이 연결되어 있는지 `candump`로 확인합니다. 손 전원이 켜져 있으면
state frame이 올라오며, **왼손은 `0x2xx`(예: `0x221`), 오른손은 `0x1xx`(예: `0x121`)** ID를
씁니다.

```bash
candump -n 20 can0        # 확인할 interface. 올라오는 ID의 첫 자리로 손 판별
#   0x2xx 관측 → 왼손,  0x1xx 관측 → 오른손
```

확인한 손과 interface를 인자로 넘깁니다.

```bash
./cpp/build/basic_control left can0      # [right|left] [interface]. 위에서 확인한 값
```

정상 실행 시 출력은 다음과 같습니다.

```text
<!-- TODO: 실물 hand 로 basic_control 을 실행한 실제 정상 로그로 교체 -->
```

### 1.5 Uninstall

install한 파일 목록은 build tree의 `install_manifest.txt`에 있습니다. build tree가 남아 있다면 그
목록으로 지우는 것이 가장 정확합니다.

```bash
xargs -a cpp/build/install_manifest.txt sudo rm -f
sudo rm -rf /usr/local/include/aidin_hand2 /usr/local/lib/cmake/aidin_hand2
sudo ldconfig
```

manifest는 파일만 담고 빈 디렉터리는 남기므로 두 디렉터리를 따로 지웁니다. `ldconfig`로 loader
캐시에서 제거된 것을 반영합니다.

build tree가 없다면 경로로 지웁니다. `<prefix>`는 install할 때 쓴 값이고, 지정하지 않았다면
`/usr/local`입니다.

```bash
sudo rm -rf <prefix>/include/aidin_hand2 \
            <prefix>/lib/cmake/aidin_hand2 \
            <prefix>/lib/libaidin_hand2.so*
sudo ldconfig
```

0.5.2 이하를 쓰다가 올라온 경우에는 그때 설치된 kinematics 라이브러리가 남아 있습니다. 0.6.0
부터는 설치되지 않으므로 함께 지웁니다.

```bash
sudo rm -f <prefix>/lib/libaidin_hand2_kinematics.so*
sudo ldconfig
```

두 명령 모두 남은 것이 없는지 확인합니다.

```bash
ldconfig -p | grep aidin_hand2        # 아무것도 출력되지 않아야 합니다
ls <prefix>/lib/cmake/aidin_hand2     # No such file or directory 여야 합니다
```

prefix를 여러 곳에 쓴 적이 있다면 각각에 대해 반복하십시오. 한 곳만 지우면 남은 prefix의 SDK가
계속 탐색 대상이 되어, 서로 다른 릴리스의 라이브러리가 한 프로세스에 적재될 수 있습니다.

## 2. Python

Python 패키지 `aidin-hand2`를 설치하고 import와 로봇 핸드 연결을 확인합니다. 패키지에는 SDK가
미리 빌드되어 들어 있으므로 C++ SDK를 빌드하거나 설치하지 않아도 됩니다. 사용법은
[Python guide](10_python_usage_guide.md)에 있습니다.

### 2.1 Install

> [!IMPORTANT]
> **Python 패키지는 hand type A만 지원합니다.** type B·C 로봇 핸드에 사용하면 오류 없이
> kinematics가 다르게 계산됩니다. type B·C는 [1. C++](#1-c)를 따르십시오. type은 로봇 핸드
> 전달과 함께 알려드립니다.

설치할 수 있는 환경은 다음과 같습니다. Ubuntu 20.04에는 설치되지 않습니다.

| Item | Supported |
|---|---|
| OS | Ubuntu 22.04 · 24.04 (glibc 2.34 이상) |
| Architecture | x86_64 · aarch64 |
| Python | 3.10 · 3.11 · 3.12 · 3.13 |

venv를 만들어 그 안에 설치하십시오. Ubuntu 24.04는 시스템 Python에 pip으로 직접 설치하는 것을
막습니다. 경로 `~/venvs/aidin-hand2`는 예시입니다.

```bash
sudo apt install -y python3-venv
python3 -m venv ~/venvs/aidin-hand2
source ~/venvs/aidin-hand2/bin/activate
```

ROS 2의 `rclpy`를 같은 프로그램에서 쓰려면 venv를 만들 때 `--system-site-packages`를 붙이십시오.
붙이지 않으면 venv 안에서 시스템에 설치된 ROS 2 패키지가 보이지 않습니다.

패키지는 PyPI가 아니라 AIDIN Robotics의 package index에 있습니다. pip에 index 주소를 한 번
등록합니다.

```bash
pip config set --user global.extra-index-url https://aidinrobotics.github.io/aidin-hand2-sdk/simple/
```

설정은 `~/.config/pip/pip.conf`에 저장되고, 이 사용자 계정에서 실행하는 모든 pip에 적용됩니다.
venv를 새로 만들어도 다시 등록할 필요가 없습니다.

패키지를 설치합니다. pip이 Python 버전, architecture, glibc를 확인해 맞는 파일을 고릅니다.

```bash
pip install aidin-hand2
```

특정 버전이 필요하면 `pip install aidin-hand2==0.7.1`처럼 버전을 지정합니다.

> [!IMPORTANT]
> C++ application이나 ROS 2 wrapper를 같은 컴퓨터에서 함께 쓰면 **C++ SDK와 Python 패키지의
> 버전을 맞추십시오.** 두 SDK는 따로 설치되므로, 버전이 다르면 workspace 한계 같은 동작이 서로
> 달라집니다. C++ SDK의 버전은 다음 명령으로 확인합니다.
>
> ```bash
> grep -m1 'set(PACKAGE_VERSION "' /usr/local/lib/cmake/aidin_hand2/aidin_hand2ConfigVersion.cmake
> ```

**index를 등록하지 않고 설치하기**

설정을 남기지 않으려면 설치할 때마다 index 주소를 함께 넘깁니다.

```bash
pip install aidin-hand2 --extra-index-url https://aidinrobotics.github.io/aidin-hand2-sdk/simple/
```

`requirements.txt`로 설치한다면 파일 첫 줄에
`--extra-index-url https://aidinrobotics.github.io/aidin-hand2-sdk/simple/`을 적습니다.

**인터넷 없이 설치하기**

[Releases](https://github.com/aidinrobotics/aidin-hand2-sdk/releases)에서 버전별 파일을 받아
설치합니다. 파일 이름의 `cp310`은 Python 3.10, 끝의 `x86_64`·`aarch64`는 architecture이며,
`python3 --version`과 `uname -m`의 출력과 같은 파일을 고르십시오.

```bash
pip install ./aidin_hand2-0.7.1-cp310-cp310-manylinux_2_34_x86_64.whl
```

이 패키지는 numpy를 사용합니다. numpy가 설치되어 있지 않으면 numpy 파일도 함께 받아 두십시오.

### 2.2 Verify

먼저 패키지를 불러와 버전을 확인합니다.

```bash
python3 -c "import aidin_hand2 as ah2; print(ah2.__version__)"
```

설치한 버전이 출력되면 됩니다.

```text
0.7.1
```

다음으로 로봇 핸드에 연결해 state를 읽습니다. 연결만 하고 토크는 걸지 않으므로 로봇 핸드가
움직이지 않습니다. 로봇 핸드 전원을 켜고 [CAN-FD setup](05_can_fd_setup.md)으로 interface를 올린
상태여야 합니다.

어느 interface에 어느 손이 연결되어 있는지 `candump`로 확인합니다. state frame의 ID가 **왼손은
`0x2xx`(예: `0x221`), 오른손은 `0x1xx`(예: `0x121`)** 입니다.

```bash
sudo apt install -y can-utils
candump -n 20 can0        # 확인할 interface. 올라오는 ID의 첫 자리로 손 판별
#   0x2xx 관측 → 왼손,  0x1xx 관측 → 오른손
```

확인한 손과 interface를 넣어 실행합니다. 아래는 `can0`에 왼손이 연결된 경우입니다.

```bash
python3 - <<'EOF'
import aidin_hand2 as ah2

with ah2.HandManager() as manager:
    hand = manager.create(ah2.HandConfig("can0", ah2.HandSide.LEFT))
    hand.connect()
    print(hand.get_diagnostics().lifecycle)
    print(hand.get_state().actuators.position_count)
EOF
```

SDK log 사이에 다음과 같은 lifecycle과 숫자 배열이 출력되면 연결된 것입니다. 숫자는 actuator 16개의
encoder count이며 자세에 따라 다릅니다.

```text
HandLifecycle.CONNECTED
[ 26818  85284 104265  26142  53608  46004  17883  64654  60853  26413
  74720  78521  35984  87527  95122  46613]
```

`connect()`가 `Error`를 던지면 메시지에 원인과 조치가 들어 있습니다. 문구별 정리는
[Error messages](15_error_messages.md)에 있습니다.

### 2.3 When pip finds no package

`pip install aidin-hand2`가 다음 오류로 끝나면 pip이 이 컴퓨터에 맞는 파일을 찾지 못한 것입니다.
아무것도 설치되지 않은 상태입니다.

```text
ERROR: Could not find a version that satisfies the requirement aidin-hand2 (from versions: none)
ERROR: No matching distribution found for aidin-hand2
```

원인은 아래 넷 중 하나입니다. 위에서부터 확인하십시오.

| Cause | Check | Expected |
|---|---|---|
| index 주소를 등록하지 않음 | `pip config list` | `global.extra-index-url` 줄이 있음 |
| 지원하지 않는 Python | `python3 --version` | 3.10 · 3.11 · 3.12 · 3.13 |
| 지원하지 않는 architecture | `uname -m` | `x86_64` 또는 `aarch64` |
| glibc가 오래됨 | `ldd --version` 첫 줄 | 2.34 이상 (Ubuntu 22.04 이상) |

pip이 이 컴퓨터에서 받아들이는 파일 종류는 `pip debug --verbose`의 `Compatible tags` 목록에
나옵니다. 받으려는 파일 이름의 `cp310-cp310-manylinux_2_34_x86_64` 부분이 이 목록에 있어야
설치됩니다.

```bash
pip debug --verbose 2>/dev/null | grep -A5 "Compatible tags"
```

파일을 직접 지정해 설치할 때 맞지 않는 파일을 고르면 다음 오류로 거부됩니다. 2.1의 **인터넷 없이
설치하기**에 따라 파일을 다시 고르십시오.

```text
ERROR: aidin_hand2-0.7.1-cp310-cp310-manylinux_2_34_x86_64.whl is not a supported wheel on this platform.
```

**지원 목록에 없는 Python 버전에서 쓰기**

Ubuntu 22.04 이상이고 Python 버전만 목록에 없다면 소스에서 빌드해 설치할 수 있습니다. 컴파일러,
spdlog, git과 사용 중인 Python의 개발 header가 필요하고, 빌드에 몇 분이 걸립니다. 버전 `v0.7.1`은
예시입니다.

```bash
sudo apt install -y build-essential libspdlog-dev git python3-dev
pip install "aidin-hand2 @ git+https://github.com/aidinrobotics/aidin-hand2-sdk.git@v0.7.1#subdirectory=python"
```

이렇게 설치한 패키지는 시스템의 spdlog를 사용하므로, 설치한 뒤에 `libspdlog-dev`를 지우지
마십시오.

### 2.4 Uninstall

패키지를 지웁니다.

```bash
pip uninstall aidin-hand2
```

index를 더 쓰지 않는다면 등록한 주소도 지웁니다.

```bash
pip config unset --user global.extra-index-url
```

venv를 이 패키지 때문에 만들었다면 venv 디렉터리를 지워도 됩니다. 그 안에 설치한 다른 패키지도 함께
지워집니다.

```bash
rm -rf ~/venvs/aidin-hand2
```
