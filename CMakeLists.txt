cmake_minimum_required(VERSION 3.16)
project(robot_foundations LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(robot_core STATIC
    src/validator.cpp
    src/MockRobot.cpp
    src/robot_factory.cpp
    src/boundedqueue.cpp
    src/executor.cpp
)
target_include_directories(robot_core PUBLIC include)

add_executable(mock_arm_node src/mock_arm_node.cpp)
target_link_libraries(mock_arm_node robot_core)
ament_target_dependencies(mock_arm_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

add_executable(robot_cli src/main.cpp)
target_link_libraries(robot_cli PRIVATE robot_core)

install(TARGETS
  mock_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(mock_robot_math test/mock_robot_math.cpp)
  target_link_libraries(mock_robot_math PRIVATE robot_core)
  add_test(NAME mock_robot_math COMMAND mock_robot_math)

  add_executable(boundedqueue_test test/boundedqueue_test.cpp)
  target_link_libraries(boundedqueue_test PRIVATE robot_core)
  add_test(NAME boundedqueue_test COMMAND boundedqueue_test)
endif()

ament_package()
