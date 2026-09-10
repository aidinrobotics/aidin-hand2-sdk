[← C++ API Reference](../08_cpp_api_reference.md)

# `version.hpp`

```cpp
inline constexpr int kVersionMajor = 0;
inline constexpr int kVersionMinor = 5;
inline constexpr int kVersionPatch = 0;
inline constexpr const char* kVersionString = "0.5.0";   // 빌드한 SDK의 버전
```

이 header는 빌드할 때 `cmake/version.hpp.in`에서 생성되며, 값의 출처는 `cpp/CMakeLists.txt`의
`project(VERSION ...)` 한 곳입니다.

application에서는 버전을 명시해 찾으십시오. 호환 정책이 `SameMinorVersion`이라 minor가 다르면
configure 단계에서 걸립니다.

```cmake
find_package(aidin_hand2 0.5 REQUIRED)
target_link_libraries(your_app PRIVATE aidin_hand2::aidin_hand2)
```

연결 방법은 [SDK build & install](../06_sdk_build_and_install.md#3-install-and-link), 전체 흐름을
담은 예제는 [C++ guide](../07_cpp_usage_guide.md#933-exit-on-failure)에 있습니다.
