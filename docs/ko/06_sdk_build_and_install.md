# SDK build & install

SDK를 빌드·테스트·설치하고 예제로 동작을 확인합니다. 먼저
[Real-time kernel setup](04_real_time_kernel_setup.md)과 [CAN-FD setup](05_can_fd_setup.md)을
마치십시오.

## Contents

&nbsp;&nbsp;[**1. Install dependencies**](#1-install-dependencies)<br>
&nbsp;&nbsp;[**2. Build**](#2-build)<br>
&nbsp;&nbsp;[**3. Install and link**](#3-install-and-link)<br>
&nbsp;&nbsp;[**4. Test program**](#4-test-program)<br>
&nbsp;&nbsp;[**5. Uninstall**](#5-uninstall)

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
```

> [!IMPORTANT]
> **thumb의 actuator 두 개는 하드웨어 세대에 따라 ball screw의 lead가 1 mm 또는 2 mm입니다.**
> lead가 actuator 1 mm 이동당 encoder count를 결정하므로, 하드웨어와 맞지 않는 쪽으로 빌드하면
> 그 두 actuator가 **두 배 또는 절반으로 움직입니다.** 빌드 전에 어느 하드웨어인지 확인하고,
> 확인이 어려우면 [issue](https://github.com/aidinrobotics/aidin-hand2-sdk/issues)로
> 문의하십시오.

아래 두 가지 중 해당하는 쪽만 실행하십시오.

**a) lead 1 mm**

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
#   -- aidin_hand2: thumb lead = 1mm
#   -- aidin_hand2: kinematics = .../libaidin_hand2_kinematics.so.0.5.2
```

**b) lead 2 mm**

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DAIDIN_HAND2_THUMB_LEAD=2mm
#   -- aidin_hand2: thumb lead = 2mm
#   -- aidin_hand2: kinematics = .../libaidin_hand2_kinematics.so.0.5.2-2mm
```

출력 두 줄로 선택된 값을 확인한 뒤 빌드합니다.

```bash
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

설치되는 것은 공개 header, shared library 두 개, CMake package config입니다.

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
find_package(aidin_hand2 0.5 REQUIRED)
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
find_package(aidin_hand2 0.5 REQUIRED)
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

## 5. Uninstall

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
            <prefix>/lib/libaidin_hand2.so* \
            <prefix>/lib/libaidin_hand2_kinematics.so*
sudo ldconfig
```

두 명령 모두 남은 것이 없는지 확인합니다.

```bash
ldconfig -p | grep aidin_hand2        # 아무것도 출력되지 않아야 합니다
ls <prefix>/lib/cmake/aidin_hand2     # No such file or directory 여야 합니다
```

prefix를 여러 곳에 쓴 적이 있다면 각각에 대해 반복하십시오. 한 곳만 지우면 남은 prefix의 SDK가
계속 탐색 대상이 되어, 서로 다른 릴리스의 라이브러리가 한 프로세스에 적재될 수 있습니다.
