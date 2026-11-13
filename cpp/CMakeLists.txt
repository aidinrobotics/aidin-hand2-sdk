cmake_minimum_required(VERSION 3.16)
project(aidin_hand2 VERSION 0.9.0 LANGUAGES CXX)

# A printf format that no longer matches its argument is otherwise silent.
add_compile_options(-Wall -Wextra)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# version.hpp comes from project(VERSION)
set(AIDIN_HAND2_GENERATED_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/include)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.hpp.in
  ${AIDIN_HAND2_GENERATED_INCLUDE_DIR}/aidin_hand2/version.hpp
  @ONLY
)

find_package(Threads REQUIRED)

# 1.9 so that an older spdlog fails here rather than inside the build
find_package(spdlog 1.9 REQUIRED)

# ── Kinematics ────────────────────────────────────────────────────────────────
# Prebuilt archives from the private aidin-hand2-kinematics repository, one per hand model, with a
# C boundary. Both go into libaidin_hand2.so, and HandConfig::hand_model picks one at run time.

# Build options of older releases, dropped with a warning
foreach(_old_option AIDIN_HAND2_HAND_TYPE AIDIN_HAND2_THUMB_LEAD)
  if(DEFINED ${_old_option})
    message(WARNING
      "${_old_option} is gone and ignored. Every build carries the hand models Gen2 and Gen2B; "
      "pick one at run time with HandConfig::hand_model.")
    unset(${_old_option} CACHE)
  endif()
endforeach()

set(_kin_dir ${CMAKE_CURRENT_SOURCE_DIR}/prebuilt/${CMAKE_SYSTEM_PROCESSOR})
set(_kin_archives libaidin_hand2_kinematics_type_a.a libaidin_hand2_kinematics_type_b.a)

foreach(_kin_archive IN LISTS _kin_archives)
  if(NOT EXISTS ${_kin_dir}/${_kin_archive})
    # Nothing local fixes this, so name what is shipped
    file(GLOB _kin_shipped RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/prebuilt
         ${CMAKE_CURRENT_SOURCE_DIR}/prebuilt/*)
    message(FATAL_ERROR
      "this SDK release does not ship kinematics for ${CMAKE_SYSTEM_PROCESSOR}.\n"
      "  Shipped architectures: ${_kin_shipped}\n"
      "  To request another, open an issue at\n"
      "  https://github.com/aidinrobotics/aidin-hand2-sdk/issues")
  endif()
endforeach()

add_library(aidin_hand2_kinematics_type_a STATIC IMPORTED)
set_target_properties(aidin_hand2_kinematics_type_a PROPERTIES
  IMPORTED_LOCATION ${_kin_dir}/libaidin_hand2_kinematics_type_a.a)
add_library(aidin_hand2_kinematics_type_b STATIC IMPORTED)
set_target_properties(aidin_hand2_kinematics_type_b PROPERTIES
  IMPORTED_LOCATION ${_kin_dir}/libaidin_hand2_kinematics_type_b.a)

# Both archives under one name
add_library(aidin_hand2_kinematics INTERFACE)
target_link_libraries(aidin_hand2_kinematics INTERFACE
  aidin_hand2_kinematics_type_a aidin_hand2_kinematics_type_b)
message(STATUS "aidin_hand2: kinematics = ${_kin_dir} (hand models Gen2 and Gen2B)")

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
  src/hand_core/comms/canfd/protocol.cpp
  src/hand_core/comms/canfd/transport.cpp
  src/hand_core/comms/state_machine/homing_sequence.cpp
  src/hand_core/kinematics/kinematics.cpp
  src/hand_core/kinematics/gen2/kinematics.cpp
  src/hand_core/kinematics/gen2b/kinematics.cpp
  src/hand_core/kinematics/hand_kinematics_public.cpp
  src/hand_core/kinematics/joint_clamp.cpp
  src/hand_core/model/model_composition.cpp
  src/hand_core/model/gen2.cpp
  src/hand_core/model/gen2b.cpp
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
  PRIVATE aidin_hand2_kinematics spdlog::spdlog  # absent from the public headers
  PUBLIC Threads::Threads)

# Unresolved symbols fail here, and the archives' symbols stay local to this library
string(REPLACE ";" ":" _kin_exclude_libs "${_kin_archives}")
target_link_options(aidin_hand2 PRIVATE
  "-Wl,--no-undefined"
  "-Wl,--exclude-libs,${_kin_exclude_libs}")

# The soname moves with the minor, as SameMinorVersion below
set_target_properties(aidin_hand2 PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
  POSITION_INDEPENDENT_CODE ON)

add_library(aidin_hand2::aidin_hand2 ALIAS aidin_hand2)

option(AIDIN_HAND2_BUILD_TESTS "Build aidin_hand2 unit tests" ON)
if(AIDIN_HAND2_BUILD_TESTS)
  enable_testing()

  # Tests that link the library and include src/
  set(AIDIN_HAND2_TESTS
    canfd/byte_order
    canfd/protocol
    canfd/transport_open
    controllers/joint_impedance_controller
    controllers/joint_position_controller
    hand_core/command_to_frames
    hand_core/command_validators
    hand_core/connection_info
    hand_core/failure_messages
    hand_core/fault_robustness
    hand_core/homing_request
    hand_core/reconnect_reset
    hand_core/tactile_bias
    kinematics/joint_clamp
    kinematics/joint_vel_eff
    kinematics/kinematics_public
    lifecycle/action_check
    logging/logging
    state_machine/control_word
    state_machine/homing_sequence
    state_machine/homing_sequence_extra
    types/error
    types/realtime_buffer
    types/realtime_event_ring
    types/to_string
  )
  foreach(test_path IN LISTS AIDIN_HAND2_TESTS)
    get_filename_component(test_name ${test_path} NAME)
    add_executable(${test_name}_test tests/${test_path}_test.cpp)
    target_link_libraries(${test_name}_test PRIVATE aidin_hand2)
    target_include_directories(${test_name}_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_test(NAME ${test_name} COMMAND ${test_name}_test)
  endforeach()

  # Skipped where unprivileged user namespaces are restricted
  set_tests_properties(transport_open PROPERTIES SKIP_RETURN_CODE 77)

  # Fails at build time: every check is a static_assert
  add_executable(abi_lock_test tests/abi/abi_lock_test.cpp)
  target_link_libraries(abi_lock_test PRIVATE aidin_hand2)
  add_test(NAME abi_lock COMMAND abi_lock_test)

  # The archives through each model's kinematics.cpp only, without the rest of the SDK
  add_executable(kinematics_core_test tests/kinematics/kinematics_core_test.cpp
    src/hand_core/kinematics/gen2/kinematics.cpp
    src/hand_core/kinematics/gen2b/kinematics.cpp)
  target_link_libraries(kinematics_core_test PRIVATE aidin_hand2_kinematics)
  target_include_directories(kinematics_core_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME kinematics_core COMMAND kinematics_core_test)

  # Built from source so that mock_transport.cpp takes the place of transport.cpp
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
    src/hand_core/comms/canfd/protocol.cpp
    src/hand_core/comms/state_machine/homing_sequence.cpp
    src/hand_core/kinematics/kinematics.cpp
    src/hand_core/kinematics/gen2/kinematics.cpp
    src/hand_core/kinematics/gen2b/kinematics.cpp
    src/hand_core/kinematics/hand_kinematics_public.cpp
    src/hand_core/kinematics/joint_clamp.cpp
    src/hand_core/model/model_composition.cpp
    src/hand_core/model/gen2.cpp
    src/hand_core/model/gen2b.cpp
    src/logging/logging.cpp
  )
  target_compile_features(liveness_test PRIVATE cxx_std_17)
  target_include_directories(liveness_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}          # for tests/liveness/mock_transport.hpp
    ${AIDIN_HAND2_GENERATED_INCLUDE_DIR}
  )
  target_link_libraries(liveness_test PRIVATE
    aidin_hand2_kinematics spdlog::spdlog Threads::Threads)
  add_test(NAME liveness COMMAND liveness_test)

  # On a real hand only: HAND_TEST_CAN=can0 ctest -L hardware, a pass without HAND_TEST_CAN
  add_executable(hand_smoke_test tests/hardware/hand_smoke_test.cpp)
  target_link_libraries(hand_smoke_test PRIVATE aidin_hand2)
  add_test(NAME hand_smoke COMMAND hand_smoke_test)
  set_tests_properties(hand_smoke PROPERTIES LABELS hardware)
endif()

# Examples use the public headers only
option(AIDIN_HAND2_BUILD_EXAMPLES "Build aidin_hand2 examples" ON)
if(AIDIN_HAND2_BUILD_EXAMPLES)
  # In the order of the C++ guide chapters
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
    21_motion_sequence
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

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Generated, so not in include/
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

# 0.x treats a minor bump as breaking
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
