# A program built against an installed or extracted package alone:
#   cmake -S cpp/tests/package -B <dir> -DCMAKE_PREFIX_PATH=<package>
cmake_minimum_required(VERSION 3.16)
project(package_consumer LANGUAGES CXX)

find_package(aidin_hand2 REQUIRED)

add_executable(package_consumer main.cpp)
target_link_libraries(package_consumer PRIVATE aidin_hand2::aidin_hand2)
