# SDK build & install

SDK를 빌드·테스트·설치하고 예제로 동작을 확인합니다. 먼저
[Real-time kernel setup](04_real_time_kernel_setup.md)과 [CAN-FD setup](05_can_fd_setup.md)을
마치십시오.

## Contents

&nbsp;&nbsp;[**1. Install dependencies**](#1-install-dependencies)<br>
&nbsp;&nbsp;[**2. Build**](#2-build)<br>
&nbsp;&nbsp;[**3. Install and link**](#3-install-and-link)<br>
&nbsp;&nbsp;[**4. Test program**](#4-test-program)

## 1. Install dependencies

Ubuntu 22.04 / 24.04를 지원합니다. 두 배포판의 apt 패키지가 요구 버전을 충족하므로
버전을 따로 지정할 필요는 없습니다.

```bash
sudo apt update
sudo apt install -y build-essential cmake libspdlog-dev can-utils
```

최소 요구는 spdlog 1.9와 C++17 컴파일러입니다. Eigen은 kinematics 구현에만 쓰이고 그 부분이
미리 빌드되어 배포되므로 설치하지 않아도 됩니다.

## 2. Build

이후 명령은 모두 SDK repo 루트에서 실행합니다.

```bash
cd aidin-hand2-sdk
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build cpp/build -j"$(nproc)"
```

선택적으로 코드 로직 테스트를 돌릴 수 있습니다.

```bash
ctest --test-dir cpp/build --output-on-failure
```

실패하면 install로 넘어가지 마십시오. 출력 로그와 아래 환경 정보를
[issue](https://github.com/aidinrobotics/aidin-hand2-sdk/issues)에 그대로 붙이면 됩니다 — 별도
설명은 없어도 됩니다.

```bash
lsb_release -d; uname -srm; gcc --version | head -1; cmake --version | head -1
```

## 3. Install and link

공개 header, shared library 두 개, CMake package config를 `/usr/local`에 설치합니다. shared
library는 loader가 찾아야 하므로 설치 후 `ldconfig`로 캐시를 갱신합니다.

```bash
sudo cmake --install cpp/build
sudo ldconfig
```

시스템 경로를 건드리지 않으려면 prefix를 지정합니다: `cmake --install cpp/build --prefix <prefix>`.
이때는 `ldconfig`가 필요하지 않습니다. CMake가 application에 RUNPATH를 넣어 주기 때문입니다.

application의 CMakeLists.txt에 다음을 추가하면 SDK를 link할 수 있습니다. 버전을 반드시 지정
하십시오. SDK는 minor가 다르면 호환되지 않는데, 지정하지 않으면 CMake가 버전 검사를 건너뜁니다.

```cmake
list(APPEND CMAKE_PREFIX_PATH "<prefix>")   # /usr/local이 아닌 prefix에 설치했을 때만
find_package(aidin_hand2 0.5 REQUIRED)
target_link_libraries(your_app PRIVATE aidin_hand2::aidin_hand2)
```

application을 직접 install한다면 CMake가 그 시점에 RUNPATH를 제거합니다. `/usr/local`이 아닌
prefix를 쓰셨다면 application 쪽에 `INSTALL_RPATH`를 지정하십시오.

## 4. Test program

2절에서 빌드하면 예제 실행 파일 `cpp/build/basic_control`이 생깁니다. 연결 → homing →
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
./cpp/build/basic_control left can0      # [right|left] [interface] — 위에서 확인한 값
```

정상 실행 시 출력은 다음과 같습니다.

```text
<!-- TODO: 실물 hand 로 basic_control 을 실행한 실제 정상 로그로 교체 -->
```
