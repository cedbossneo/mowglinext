cmake_minimum_required(VERSION 3.8)
project(mowgli_coverage_planner)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Fields2Cover installs its cmake config to <prefix>/cmake/fields2cover/ rather
# than the standard <prefix>/lib/cmake/ path, so CMake config-mode search on
# Linux does not find it automatically.  Look for the config in common prefixes
# and set Fields2Cover_DIR explicitly before calling find_package.
if(NOT Fields2Cover_DIR)
  foreach(_f2c_prefix "/usr/local" "/usr")
    if(EXISTS "${_f2c_prefix}/cmake/fields2cover/Fields2CoverConfig.cmake")
      set(Fields2Cover_DIR "${_f2c_prefix}/cmake/fields2cover" CACHE PATH
          "Path to Fields2CoverConfig.cmake")
      break()
    endif()
  endforeach()
endif()
find_package(Fields2Cover REQUIRED)
find_package(GDAL REQUIRED)

# ---------------------------------------------------------------------------
# Library: mowgli_coverage_planner_lib
#
# Separating the implementation into a library lets the GTest executables
# link against it without re-compiling all sources.
# ---------------------------------------------------------------------------

add_library(mowgli_coverage_planner_lib STATIC
  src/polygon_utils.cpp
  src/coverage_planner_node.cpp
)

target_include_directories(mowgli_coverage_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Fields2Cover_INCLUDE_DIR}
)

ament_target_dependencies(mowgli_coverage_planner_lib
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  mowgli_interfaces
  tf2
  tf2_geometry_msgs
)

target_link_libraries(mowgli_coverage_planner_lib
  Fields2Cover
  ${GDAL_LIBRARIES}
)

target_include_directories(mowgli_coverage_planner_lib PUBLIC
  ${GDAL_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# Executable: coverage_planner_node
# ---------------------------------------------------------------------------

add_executable(coverage_planner_node
  src/main.cpp
)

target_include_directories(coverage_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(coverage_planner_node
  mowgli_coverage_planner_lib
)

ament_target_dependencies(coverage_planner_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  mowgli_interfaces
  tf2
  tf2_geometry_msgs
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS coverage_planner_node mowgli_coverage_planner_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  # --- test_polygon_utils ---------------------------------------------------
  ament_add_gtest(test_polygon_utils
    test/test_polygon_utils.cpp
  )

  target_include_directories(test_polygon_utils PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_polygon_utils
    mowgli_coverage_planner_lib
  )

  ament_target_dependencies(test_polygon_utils
    rclcpp
    rclcpp_action
    nav_msgs
    geometry_msgs
    mowgli_interfaces
    tf2
    tf2_geometry_msgs
  )

  # --- test_coverage_planner ------------------------------------------------
  ament_add_gtest(test_coverage_planner
    test/test_coverage_planner.cpp
  )

  target_include_directories(test_coverage_planner PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_coverage_planner
    mowgli_coverage_planner_lib
  )

  ament_target_dependencies(test_coverage_planner
    rclcpp
    rclcpp_action
    nav_msgs
    geometry_msgs
    mowgli_interfaces
    tf2
    tf2_geometry_msgs
  )
endif()

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(mowgli_coverage_planner_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
