cmake_minimum_required(VERSION 3.8)
project(mowgli_unicore_gnss)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(compass_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/serial_port.cpp
  src/um982_parser.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(um982_node
  src/um982_node.cpp
)

target_link_libraries(um982_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(um982_node
  rclcpp
  sensor_msgs
  compass_msgs
  diagnostic_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  TARGETS ${PROJECT_NAME}_core um982_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_um982_parser
    test/test_um982_parser.cpp
  )
  target_link_libraries(test_um982_parser
    ${PROJECT_NAME}_core
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
