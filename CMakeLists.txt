
cmake_minimum_required(VERSION 3.16)
project(robot_foundations LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robot_core STATIC
    src/validator.cpp
    src/MockRobot.cpp
    src/robot_factory.cpp
)
target_include_directories(robot_core PUBLIC include)

add_executable(robot_cli src/main.cpp)
target_link_libraries(robot_cli PRIVATE robot_core)

enable_testing()
add_executable(mock_robot_math test/mock_robot_math.cpp)
target_link_libraries(mock_robot_math PRIVATE robot_core)
add_test(NAME mock_robot_math COMMAND mock_robot_math)