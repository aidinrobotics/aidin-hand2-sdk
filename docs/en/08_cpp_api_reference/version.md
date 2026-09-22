[← C++ API Reference](../08_cpp_api_reference.md)

# `version.hpp`

```cpp
inline constexpr int kVersionMajor = 0;
inline constexpr int kVersionMinor = 7;
inline constexpr int kVersionPatch = 0;
inline constexpr const char* kVersionString = "0.7.0";   // the version you built
```

CMake generates this header at build time from `cmake/version.hpp.in`, and the values come from
`project(VERSION ...)` in `cpp/CMakeLists.txt`, the single source.

Ask for the version explicitly in your application. The compatibility policy is
`SameMinorVersion`, so a different minor fails at configure time.

```cmake
find_package(aidin_hand2 0.7 REQUIRED)
target_link_libraries(your_app PRIVATE aidin_hand2::aidin_hand2)
```

How to link is in [SDK build & install](../06_sdk_build_and_install.md#3-install-and-link), and a
full worked example is in the [C++ guide](../07_cpp_usage_guide.md#933-exit-on-failure).
