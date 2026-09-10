cmake_minimum_required(VERSION 3.16)
project(aidin_hand2 VERSION 0.5.2 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# version.hpp 는 project(VERSION) 에서 생성된다 — 버전 기준은 위 project() 한 곳.
set(AIDIN_HAND2_GENERATED_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/include)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.hpp.in
  ${AIDIN_HAND2_GENERATED_INCLUDE_DIR}/aidin_hand2/version.hpp
  @ONLY
)

find_package(Threads REQUIRED)  # HandCore RT 스레드(std::thread)

# The version is required so that a distribution shipping an older spdlog fails here with a clear
# message instead of somewhere inside the build. Eigen is not looked for at all any more: the only
# file that included it is the kinematics core, and that now arrives prebuilt.
find_package(spdlog 1.9 REQUIRED)  # 로깅 backend — logging.cpp 전용, 공개 헤더엔 안 샘

# ── kinematics core ───────────────────────────────────────────────────────────
# The equations and the hand geometry constants live only in the internal repo. The public SDK is
# generated from it by dev/publish-public.sh, which drops hand_kinematics_core.cpp and
# hand_kinematics_constants.hpp and adds prebuilt/<arch>/. hand_kinematics_core.hpp stays in src/
# in both, because it is declarations only.
#
# There is only one configuration: the .so under prebuilt/<arch>/ is linked, in the internal repo
# and in the public one alike. So what CI verifies here is literally what a customer builds, and
# the TSan and arm64 runs cover the shipped artifact rather than a look-alike.
#
# That .so is committed in both repos, which is why a clone builds without a bootstrap step.
# dev/build-kinematics.sh regenerates it from the source and is run when the equations change.
#
# The soname carries the full SDK version, so every release has its own file name. What this
# library exports is four signatures that have never changed, but what it *means* is a set of
# equations and calibration constants, and those do change: 0.5.1 halved the lead of two thumb
# screws, which doubled their counts per millimetre. A soname that tracked only the signatures
# stayed at .so.1 across that change, so a 0.5.0 library satisfied a 0.5.1 consumer and the wrong
# thumb kinematics loaded in silence. Tying the name to the release removes the judgement call:
# nobody has to notice that a constant changed.
#
# The thumb comes in two hardware generations, differing in the lead of the d1 and d2 screws, and
# each has its own library. The 1 mm one is the default and carries no suffix, so only the
# exception is marked. Picking the wrong one is a loud failure rather than a quiet one: the
# sonames differ, so the library a consumer was built against is the only one that satisfies it.
set(AIDIN_HAND2_THUMB_LEAD "1mm" CACHE STRING "Thumb d1/d2 screw lead: 1mm or 2mm")
set_property(CACHE AIDIN_HAND2_THUMB_LEAD PROPERTY STRINGS "1mm" "2mm")
if(AIDIN_HAND2_THUMB_LEAD STREQUAL "1mm")
  set(_kin_suffix "")
  set(_kin_variant_src hand_kinematics_core.cpp)
elseif(AIDIN_HAND2_THUMB_LEAD STREQUAL "2mm")
  set(_kin_suffix "-2mm")
  set(_kin_variant_src hand_kinematics_core_2mm.cpp)
else()
  message(FATAL_ERROR
    "AIDIN_HAND2_THUMB_LEAD is '${AIDIN_HAND2_THUMB_LEAD}'. Use 1mm or 2mm.")
endif()

set(_kin_soname libaidin_hand2_kinematics.so.${PROJECT_VERSION}${_kin_suffix})
set(_kin_dir ${CMAKE_CURRENT_SOURCE_DIR}/prebuilt/${CMAKE_SYSTEM_PROCESSOR})
set(_kin_src ${CMAKE_CURRENT_SOURCE_DIR}/src/hand_core/kinematics/${_kin_variant_src})

if(NOT EXISTS ${_kin_dir}/${_kin_soname})
  # In the internal repo this is a missing rebuild, and the fix is one command.
  if(EXISTS ${_kin_src})
    message(FATAL_ERROR
      "no kinematics library yet for ${CMAKE_SYSTEM_PROCESSOR}. Run dev/build-kinematics.sh")
  endif()
  # In the public repo it means this architecture is not shipped, and nothing the reader can run
  # will fix it, so the message says which ones are and where to ask for another.
  file(GLOB _kin_shipped RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/prebuilt
       ${CMAKE_CURRENT_SOURCE_DIR}/prebuilt/*)
  message(FATAL_ERROR
    "this SDK release does not ship a kinematics library for ${CMAKE_SYSTEM_PROCESSOR}.\n"
    "  Shipped architectures: ${_kin_shipped}\n"
    "  To request another, open an issue at\n"
    "  https://github.com/aidinrobotics/aidin-hand2-sdk/issues")
endif()

# Only the internal repo has the source, and there a stale library is the one failure mode worth
# guarding: edit the equations, skip the rebuild, and the SDK quietly links the old ones. Nothing
# downstream would notice, so it is refused here.
if(EXISTS ${_kin_src} AND ${_kin_src} IS_NEWER_THAN ${_kin_dir}/${_kin_soname})
  message(FATAL_ERROR
    "kinematics source is newer than ${_kin_soname}. Run dev/build-kinematics.sh")
endif()

# The file is named after its SONAME and there is no version symlink chain. This library is an
# implementation detail of the SDK rather than something consumers link by name, so one file per
# release is enough and there is no second place for the version to drift.
add_library(aidin_hand2_kinematics SHARED IMPORTED)
set_target_properties(aidin_hand2_kinematics PROPERTIES
  IMPORTED_LOCATION ${_kin_dir}/${_kin_soname}
  IMPORTED_SONAME ${_kin_soname})
message(STATUS "aidin_hand2: thumb lead = ${AIDIN_HAND2_THUMB_LEAD}")
message(STATUS "aidin_hand2: kinematics = ${_kin_dir}/${_kin_soname}")

add_library(aidin_hand2 SHARED
  src/hand/hand.cpp
  src/hand/hand_manager.cpp
  src/hand_core/hand_core.cpp
  src/hand_core/threads/hand_core_control_thread.cpp
  src/hand_core/threads/hand_core_event_logging_thread.cpp
  src/hand_core/lifecycle/action_check.cpp
  src/hand_core/controllers/joint_position_controller.cpp
  src/hand_core/controllers/joint_impedance_controller.cpp
  src/hand_core/comms/hand_comms.cpp
  src/hand_core/comms/canfd/interface_setup.cpp
  src/hand_core/comms/canfd/protocol.cpp
  src/hand_core/comms/canfd/transport.cpp
  src/hand_core/comms/state_machine/homing_sequence.cpp
  src/hand_core/kinematics/hand_kinematics_public.cpp
  src/hand_core/kinematics/joint_clamp.cpp
  src/logging/logging.cpp
)

target_compile_features(aidin_hand2 PUBLIC cxx_std_17)

target_include_directories(aidin_hand2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${AIDIN_HAND2_GENERATED_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(aidin_hand2
  PRIVATE aidin_hand2_kinematics spdlog::spdlog  # not propagated, absent from the public headers
  PUBLIC Threads::Threads)                       # propagated — HandCore RT threads

# A .so links even with unresolved symbols. Catch them here or the consumer's link stage does.
target_link_options(aidin_hand2 PRIVATE "-Wl,--no-undefined")

# The SOVERSION boundary is minor, matching COMPATIBILITY SameMinorVersion. On a breaking change
# the soname moves and the loader refuses to load, so a consumer that did not rebuild fails
# loudly instead of misbehaving quietly.
set_target_properties(aidin_hand2 PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
  POSITION_INDEPENDENT_CODE ON  # implied by SHARED, kept in case this goes back to STATIC
  # Find the kinematics .so next to this one. DT_RUNPATH is not inherited by dependencies, so
  # without this a non-default install prefix makes consumers die with
  # "libaidin_hand2_kinematics.so.<version> not found".
  INSTALL_RPATH "$ORIGIN")

add_library(aidin_hand2::aidin_hand2 ALIAS aidin_hand2)

option(AIDIN_HAND2_BUILD_TESTS "Build aidin_hand2 unit tests" ON)
if(AIDIN_HAND2_BUILD_TESTS)
  enable_testing()
  # canfd: 프레임 byte order
  add_executable(byte_order_test tests/canfd/byte_order_test.cpp)
  target_link_libraries(byte_order_test PRIVATE aidin_hand2)
  target_include_directories(byte_order_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME byte_order COMMAND byte_order_test)

  # kinematics: workspace clamp 경계의 이음새·안쪽 보장·최단 투영 (공개 API 만 사용).
  add_executable(joint_clamp_test tests/kinematics/joint_clamp_test.cpp)
  target_link_libraries(joint_clamp_test PRIVATE aidin_hand2)
  target_include_directories(joint_clamp_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME joint_clamp COMMAND joint_clamp_test)

  # types: RealtimeBuffer SPSC 의미론 + tearing 부재(2-스레드).
  add_executable(realtime_buffer_test tests/types/realtime_buffer_test.cpp)
  target_link_libraries(realtime_buffer_test PRIVATE aidin_hand2 Threads::Threads)
  target_include_directories(realtime_buffer_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME realtime_buffer COMMAND realtime_buffer_test)

  # types: RealtimeEventRing FIFO·overflow drop·wraparound + 2-스레드 무손실 순서보존.
  add_executable(realtime_event_ring_test tests/types/realtime_event_ring_test.cpp)
  target_link_libraries(realtime_event_ring_test PRIVATE aidin_hand2 Threads::Threads)
  target_include_directories(realtime_event_ring_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME realtime_event_ring COMMAND realtime_event_ring_test)

  # logging: public 4함수 + 파일 sink + throw 흔적(debug) + to_string(ErrorCode) smoke.
  add_executable(logging_test tests/logging/logging_test.cpp)
  target_link_libraries(logging_test PRIVATE aidin_hand2)
  target_include_directories(logging_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME logging COMMAND logging_test)

  # hand_core: command_to_frames(IK/impedance/clamp/fault-mask) + FK joint 채움 — HW 무관.
  add_executable(command_to_frames_test tests/hand_core/command_to_frames_test.cpp)
  # Calls the kinematics internals, so it links the .so directly — aidin_hand2 holds it PRIVATE
  # and does not propagate it. Linking it also brings in prebuilt/include.
  target_link_libraries(command_to_frames_test PRIVATE aidin_hand2 aidin_hand2_kinematics)
  target_include_directories(command_to_frames_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME command_to_frames COMMAND command_to_frames_test)

  # state_machine: control_word 사다리 + homing 단계 전이 — Transport/HW 무관 순수 로직.
  foreach(sm_test control_word homing_sequence homing_sequence_extra stop_confirmation)
    add_executable(${sm_test}_test tests/state_machine/${sm_test}_test.cpp)
    target_link_libraries(${sm_test}_test PRIVATE aidin_hand2)
    target_include_directories(${sm_test}_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_test(NAME ${sm_test} COMMAND ${sm_test}_test)
  endforeach()

  # lifecycle: check_action_allowed 전이 표(전 셀) + Faulted 파생 + destroyed/homed gate + 원인 문구.
  add_executable(action_check_test tests/lifecycle/action_check_test.cpp)
  target_link_libraries(action_check_test PRIVATE aidin_hand2)
  target_include_directories(action_check_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME action_check COMMAND action_check_test)

  # protocol: decode 결과 분류 + velocity/current/tactile/palm decode + palm flatten + health 마스크 + Right-hand.
  add_executable(protocol_test tests/canfd/protocol_test.cpp)
  target_link_libraries(protocol_test PRIVATE aidin_hand2)
  target_include_directories(protocol_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME protocol COMMAND protocol_test)

  # controllers: JointPositionController 속도 슬루 + reset NaN-seed.
  add_executable(joint_position_controller_test tests/controllers/joint_position_controller_test.cpp)
  target_link_libraries(joint_position_controller_test PRIVATE aidin_hand2 aidin_hand2_kinematics)
  target_include_directories(joint_position_controller_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME joint_position_controller COMMAND joint_position_controller_test)

  # hand_core: command 검증기(NaN/Inf/범위) + set_max_effort 클램프 (HandCoreTestPeer 시임).
  add_executable(command_validators_test tests/hand_core/command_validators_test.cpp)
  target_link_libraries(command_validators_test PRIVATE aidin_hand2 aidin_hand2_kinematics)
  target_include_directories(command_validators_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME command_validators COMMAND command_validators_test)

  # kinematics: 조립 FK(passive/thumb q4) + 공개 task IK(thumb orient) 라운드트립.
  add_executable(kinematics_public_test tests/kinematics/kinematics_public_test.cpp)
  target_link_libraries(kinematics_public_test PRIVATE aidin_hand2)
  target_include_directories(kinematics_public_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME kinematics_public COMMAND kinematics_public_test)

  # kinematics: FK/IK round-trip regression. It uses only the four exported functions, so it sees
  # the .so's public face the way a consumer does, and it runs against the prebuilt configuration
  # unchanged, which makes it a check on the shipped artifact itself.
  add_executable(kinematics_core_test tests/kinematics/kinematics_core_test.cpp)
  target_link_libraries(kinematics_core_test PRIVATE aidin_hand2_kinematics)
  target_include_directories(kinematics_core_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME kinematics_core COMMAND kinematics_core_test)

  # abi: public struct sizes and enum tails. All static_assert, so it fails during the build, not
  # at ctest time. See the file header for what to do when it fires.
  add_executable(abi_lock_test tests/abi/abi_lock_test.cpp)
  target_link_libraries(abi_lock_test PRIVATE aidin_hand2)
  add_test(NAME abi_lock COMMAND abi_lock_test)

  # types: enum→string / to_command_mode 전수 커버리지.
  add_executable(to_string_test tests/types/to_string_test.cpp)
  target_link_libraries(to_string_test PRIVATE aidin_hand2)
  target_include_directories(to_string_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME to_string COMMAND to_string_test)

  # hand_core: fault-mask(faulted/not-enabled active 마스킹) + FK NaN 전파 차단 + homing present-fault.
  add_executable(fault_robustness_test tests/hand_core/fault_robustness_test.cpp)
  target_link_libraries(fault_robustness_test PRIVATE aidin_hand2 aidin_hand2_kinematics)
  target_include_directories(fault_robustness_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME fault_robustness COMMAND fault_robustness_test)

  # hand_core: 수동 reconnect() 가 저장된 command 를 비우고, 실패해도 멈춘 원인을 보존하는지.
  add_executable(reconnect_reset_test tests/hand_core/reconnect_reset_test.cpp)
  target_link_libraries(reconnect_reset_test PRIVATE aidin_hand2)
  target_include_directories(reconnect_reset_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME reconnect_reset COMMAND reconnect_reset_test)

  # liveness: stop()/destroy() 및 전체 lifecycle 이 정지확인 미수신에도 유한 반환(watchdog).
  # link-time seam: mock_transport.cpp 가 실제 transport.cpp 자리에 링크된다 — 따라서 이 타깃은
  # aidin_hand2 라이브러리를 링크하지 않고 소스를 직접 컴파일한다(transport.cpp 제외 + 중복 심볼 회피 + Transport 치환).
  add_executable(liveness_test
    tests/liveness/liveness_test.cpp
    tests/liveness/mock_transport.cpp
    src/hand/hand.cpp
    src/hand/hand_manager.cpp
    src/hand_core/hand_core.cpp
    src/hand_core/threads/hand_core_control_thread.cpp
    src/hand_core/threads/hand_core_event_logging_thread.cpp
    src/hand_core/lifecycle/action_check.cpp
    src/hand_core/controllers/joint_position_controller.cpp
    src/hand_core/controllers/joint_impedance_controller.cpp
    src/hand_core/comms/hand_comms.cpp
    src/hand_core/comms/canfd/interface_setup.cpp
    src/hand_core/comms/canfd/protocol.cpp
    src/hand_core/comms/state_machine/homing_sequence.cpp
    src/hand_core/kinematics/hand_kinematics_public.cpp
    src/hand_core/kinematics/joint_clamp.cpp
    src/logging/logging.cpp
  )
  # The kinematics source is not here, so aidin_hand2_kinematics is linked in below.
  target_compile_features(liveness_test PRIVATE cxx_std_17)
  target_include_directories(liveness_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}          # "tests/liveness/mock_transport.hpp" 해석용
    ${AIDIN_HAND2_GENERATED_INCLUDE_DIR}
  )
  target_link_libraries(liveness_test PRIVATE
    aidin_hand2_kinematics spdlog::spdlog Threads::Threads)
  add_test(NAME liveness COMMAND liveness_test)

  # hardware: 실기 통합 스모크(listen-only + watchdog). label "hardware" 로 기본 제외(ctest -LE hardware);
  # 실기에서만 HAND_TEST_CAN=can0 설정 후 ctest -L hardware. HAND_TEST_CAN 없으면 런타임 SKIP(통과).
  add_executable(hand_smoke_test tests/hardware/hand_smoke_test.cpp)
  target_link_libraries(hand_smoke_test PRIVATE aidin_hand2)
  add_test(NAME hand_smoke COMMAND hand_smoke_test)
  set_tests_properties(hand_smoke PROPERTIES LABELS hardware)
endif()

# 예제 — 공개 API(include/) 만 사용. src include 안 함(소비자 관점 dogfooding).
option(AIDIN_HAND2_BUILD_EXAMPLES "Build aidin_hand2 examples" ON)
if(AIDIN_HAND2_BUILD_EXAMPLES)
  # C++ guide의 장 순서를 따른다. 한 장에 여러 파일이 있을 수 있다.
  set(AIDIN_HAND2_EXAMPLES
    01_create_and_destroy       # 1. HandManager and Hand
    02_two_hands
    03_lifecycle_walkthrough    # 2. Lifecycle
    04_homing                   # 3. Homing
    05_max_effort               # 4. Control settings
    06_position_controller
    07_impedance_controller
    08_idle                     # 5. Command
    09_joint_position
    10_joint_impedance
    11_actuator_position
    12_actuator_effort
    13_kinematics               # 6. Kinematics
    14_read_state               # 7. HandState
    15_tactile
    16_diagnostics              # 8. Diagnostics
    17_fault_recovery           # 9. Error handling
    18_self_managed
    19_external_server
    19_external_client
    20_exit_on_failure
    basic_control
  )
  foreach(example IN LISTS AIDIN_HAND2_EXAMPLES)
    add_executable(${example} examples/${example}.cpp)
    target_link_libraries(${example} PRIVATE aidin_hand2)
  endforeach()
endif()

install(
  TARGETS aidin_hand2
  EXPORT aidin_hand2Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# The kinematics library is IMPORTED, which install(TARGETS) does not handle, so the file goes in
# by hand. It is a single file named after its SONAME, so nothing else has to be created.
install(FILES ${_kin_dir}/${_kin_soname} DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# 생성된 version.hpp (source include/ 에는 없음)
install(
  FILES ${AIDIN_HAND2_GENERATED_INCLUDE_DIR}/aidin_hand2/version.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/aidin_hand2
)

install(
  EXPORT aidin_hand2Targets
  NAMESPACE aidin_hand2::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/aidin_hand2
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/aidin_hand2Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/aidin_hand2Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/aidin_hand2
)

# 0.x 는 minor 가 breaking 경계 — 공개 헤더가 바뀌면 minor 를 올린다.
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/aidin_hand2ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMinorVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/aidin_hand2Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/aidin_hand2ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/aidin_hand2
)
