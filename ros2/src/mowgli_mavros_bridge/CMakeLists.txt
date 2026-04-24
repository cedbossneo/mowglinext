cmake_minimum_required(VERSION 3.08)
project(mowgli_mavros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavros REQUIRED)

add_executable(mavros_hardware_bridge_node
  src/mavros_hardware_bridge_node.cpp
)

target_include_directories(mavros_hardware_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mavros_hardware_bridge_node
  ${rclcpp_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${mowgli_interfaces_TARGETS}
  ${mavros_TARGETS}
  ${mavros_msgs_TARGETS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mavros_hardware_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()