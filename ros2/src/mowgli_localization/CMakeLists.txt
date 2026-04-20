cmake_minimum_required(VERSION 3.8)
project(mowgli_localization)

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

find_package(ament_cmake        REQUIRED)
find_package(diagnostic_msgs    REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(nav_msgs           REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(std_srvs           REQUIRED)
find_package(tf2                REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(mowgli_interfaces  REQUIRED)
find_package(ublox_ubx_msgs     REQUIRED)

# ---------------------------------------------------------------------------
# Common include directory macro
# ---------------------------------------------------------------------------

set(INCLUDE_DIR
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Executable: wheel_odometry_node
# ---------------------------------------------------------------------------

add_executable(wheel_odometry_node
  src/wheel_odometry_node.cpp
)

target_include_directories(wheel_odometry_node PUBLIC ${INCLUDE_DIR})

ament_target_dependencies(wheel_odometry_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Executable: navsat_to_absolute_pose_node
# ---------------------------------------------------------------------------

add_executable(navsat_to_absolute_pose_node
  src/navsat_to_absolute_pose_node.cpp
)

target_include_directories(navsat_to_absolute_pose_node PUBLIC ${INCLUDE_DIR})

ament_target_dependencies(navsat_to_absolute_pose_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Executable: localization_monitor_node
# ---------------------------------------------------------------------------

add_executable(localization_monitor_node
  src/localization_monitor_node.cpp
)

target_include_directories(localization_monitor_node PUBLIC ${INCLUDE_DIR})

ament_target_dependencies(localization_monitor_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Executable: gps_slam_corrector_node
# ---------------------------------------------------------------------------

add_executable(gps_slam_corrector_node
  src/gps_slam_corrector_node.cpp
)

target_include_directories(gps_slam_corrector_node PUBLIC ${INCLUDE_DIR})

ament_target_dependencies(gps_slam_corrector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---------------------------------------------------------------------------
# Executable: ublox_gnss_adapter_node
# ---------------------------------------------------------------------------

add_executable(ublox_gnss_adapter_node
  src/ublox_gnss_adapter_node.cpp
)

target_include_directories(ublox_gnss_adapter_node PUBLIC ${INCLUDE_DIR})

ament_target_dependencies(ublox_gnss_adapter_node
  diagnostic_msgs
  rclcpp
  sensor_msgs
  ublox_ubx_msgs
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    wheel_odometry_node
    navsat_to_absolute_pose_node
    localization_monitor_node
    gps_slam_corrector_node
    ublox_gnss_adapter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto      REQUIRED)
  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # ---- test_wheel_odometry ----
  # The kinematics test is pure C++ with no ROS2 dependencies; link only gtest.
  ament_add_gtest(test_wheel_odometry
    test/test_wheel_odometry.cpp
  )

  target_include_directories(test_wheel_odometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  diagnostic_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mowgli_interfaces
  ublox_ubx_msgs
)

ament_package()
