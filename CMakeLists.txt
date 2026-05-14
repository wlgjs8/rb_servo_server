cmake_minimum_required(VERSION 3.16)
project(rb_servo_server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(RB_SERVO_ENABLE_RBPODO "Enable rbpodo backend" OFF)

add_library(rb_servo_core
    src/core/types.cpp
    src/core/clock.cpp
    src/core/realtime.cpp
    src/config/config.cpp
    src/robot/mock_backend.cpp
    src/robot/rbpodo_backend.cpp
    src/robot/backend_factory.cpp
    src/control/command_buffer.cpp
    src/control/trajectory_filter.cpp
    src/control/safety_filter.cpp
    src/control/cartesian_controller.cpp
    src/control/force_controller.cpp
    src/control/dual_arm_servo_loop.cpp
    src/network/command_server.cpp
    src/network/state_publisher.cpp
    src/sensor/mock_force_torque_sensor.cpp
    src/logging/servo_logger.cpp
)

target_include_directories(rb_servo_core PUBLIC include)

target_compile_definitions(rb_servo_core PRIVATE
    $<$<BOOL:${RB_SERVO_ENABLE_RBPODO}>:RB_SERVO_ENABLE_RBPODO=1>
)

find_package(Threads REQUIRED)
target_link_libraries(rb_servo_core PUBLIC Threads::Threads)

# Config and command parsing intentionally use small built-in parsers for this scaffold.
# Replace with yaml-cpp + nlohmann/json if the project grows.

add_executable(rb_servo_server src/main.cpp)
target_link_libraries(rb_servo_server PRIVATE rb_servo_core)
