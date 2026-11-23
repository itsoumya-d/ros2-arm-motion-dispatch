cmake_minimum_required(VERSION 3.16)
project(ros2_arm_motion_dispatch LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(src)

add_library(arm_trajectory_core STATIC
    src/trajectory_validator.cpp
)

enable_testing()
add_executable(unit_tests tests/test_validator.cpp)
target_link_libraries(unit_tests arm_trajectory_core)
add_test(NAME TrajectoryValidatorTests COMMAND unit_tests)
