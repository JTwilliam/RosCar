cmake_minimum_required(VERSION 3.16)
project(roscar_visualization)

find_package(ament_cmake REQUIRED)

install(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/imu_viewer.py"
  DESTINATION lib/${PROJECT_NAME} RENAME imu_viewer)
install(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/health_monitor.py"
  DESTINATION lib/${PROJECT_NAME} RENAME health_monitor)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/launch" "${CMAKE_CURRENT_SOURCE_DIR}/rviz"
  DESTINATION share/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE" DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME host_viewer_logic COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test_viewer.py")
  set_tests_properties(host_viewer_logic PROPERTIES ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  add_test(NAME host_health_monitor COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test_health_monitor.py")
  set_tests_properties(host_health_monitor PROPERTIES ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
endif()

ament_package()
