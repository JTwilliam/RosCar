cmake_minimum_required(VERSION 3.16)
project(learm_bridge_cpp VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "Linux termios adapter requires Linux; use WSL or Raspberry Pi.")
endif()
find_package(ament_cmake REQUIRED)

add_library(learm_bridge_core STATIC src/protocol.cpp src/odometry.cpp src/serial_port.cpp)
target_compile_features(learm_bridge_core PUBLIC cxx_std_17)
target_include_directories(learm_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(learm_bridge_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(learm_replay src/replay.cpp)
target_link_libraries(learm_replay PRIVATE learm_bridge_core)
install(TARGETS learm_replay DESTINATION lib/${PROJECT_NAME})
install(TARGETS learm_bridge_core EXPORT export_learm_bridge_core ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Retain the original learning switch for existing build directories.
option(LEARM_BUILD_STUDENT_NODE "Build the ROS 2 serial bridge node" ON)
if(LEARM_BUILD_STUDENT_NODE)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_bridge_node.cpp")
    message(FATAL_ERROR "Create src/serial_bridge_node.cpp using NODE_EXERCISE.md first.")
  endif()
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  add_executable(serial_bridge_node src/serial_bridge_node.cpp)
  # Humble's ament_target_dependencies below uses the plain link signature.
  target_link_libraries(serial_bridge_node learm_bridge_core)
  ament_target_dependencies(serial_bridge_node rclcpp std_msgs nav_msgs diagnostic_msgs
                            geometry_msgs tf2_ros)
  install(TARGETS serial_bridge_node DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core PRIVATE learm_bridge_core util)
  add_test(NAME learm_core COMMAND test_core)
  add_test(NAME learm_python_parity
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/check_python_parity.py
            $<TARGET_FILE:learm_replay>)
  set_tests_properties(learm_python_parity PROPERTIES ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  if(LEARM_BUILD_STUDENT_NODE)
    add_test(NAME learm_ros_bridge
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros_bridge.py
              $<TARGET_FILE:serial_bridge_node>)
    set_tests_properties(learm_ros_bridge PROPERTIES TIMEOUT 60
      RUN_SERIAL TRUE ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1;ROS_DOMAIN_ID=173;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  endif()
endif()

ament_export_targets(export_learm_bridge_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
