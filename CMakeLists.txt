cmake_minimum_required(VERSION 3.15)
project(cut_boundary_safety_supervisor CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# This project targets WSL2/Linux only: the three executables use POSIX
# sockets and pthreads directly (see include/protocol.hpp), and the timing
# claims are measured on a native Linux filesystem (see README). No Windows
# build is provided, unlike some sibling repos in this portfolio.
option(ENABLE_ASAN "Build with AddressSanitizer + UBSan" OFF)
option(ENABLE_TSAN "Build with ThreadSanitizer" OFF)

add_compile_options(-O2 -Wall -Wextra)
if(ENABLE_ASAN)
  add_compile_options(-fsanitize=address,undefined -g -O1 -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()
if(ENABLE_TSAN)
  add_compile_options(-fsanitize=thread -g -O1 -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
endif()

find_package(Threads REQUIRED)

include_directories(include)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.17.0
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(commander src/commander_main.cpp)
target_link_libraries(commander PRIVATE Threads::Threads)

add_executable(supervisor src/supervisor_main.cpp)
target_link_libraries(supervisor PRIVATE Threads::Threads)

add_executable(actuator src/actuator_main.cpp)
target_link_libraries(actuator PRIVATE Threads::Threads)

add_executable(cbs_tests
  tests/test_main.cpp
  tests/boundary_test.cpp
  tests/supervisor_core_test.cpp
  tests/protocol_test.cpp
  tests/obstacle_test.cpp
)
target_link_libraries(cbs_tests PRIVATE gtest Threads::Threads)

# Obstacle-protection profile: single-process bench (see README's
# "Obstacle-protection profile" section for why this profile does not reuse
# the three-process IPC architecture above).
add_executable(obstacle_bench src/obstacle_bench_main.cpp)

include(GoogleTest)
if(NOT ENABLE_TSAN)
  # Under ThreadSanitizer + WSL2, gtest_discover_tests would run the
  # freshly-built binary directly (no setarch -R) as a post-build step and
  # fail immediately with "ThreadSanitizer: unexpected memory mapping"
  # (ASLR-related, see README "Building and running"). The TSan binary is
  # run manually with setarch -R instead; discovery is skipped for that
  # configuration only.
  gtest_discover_tests(cbs_tests)
endif()
