cmake_minimum_required(VERSION 3.8)
project(mowgli_brv_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)

add_executable(coverage_planner_node
  src/brv_planner_node.cpp
  src/brv_algorithm.cpp
  src/mbb.cpp
  src/grid_coverage.cpp
  src/voronoi_roadmap.cpp
  src/geometry_utils.cpp
)

target_include_directories(coverage_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
)

ament_target_dependencies(coverage_planner_node
  rclcpp
  rclcpp_action
  mowgli_interfaces
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
)

install(TARGETS coverage_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mbb test/test_mbb.cpp src/mbb.cpp src/geometry_utils.cpp)
  target_include_directories(test_mbb PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

  ament_add_gtest(test_grid test/test_grid.cpp src/grid_coverage.cpp src/geometry_utils.cpp)
  target_include_directories(test_grid PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

  ament_add_gtest(test_geometry test/test_geometry.cpp src/geometry_utils.cpp)
  target_include_directories(test_geometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_voronoi test/test_voronoi.cpp
    src/voronoi_roadmap.cpp src/geometry_utils.cpp)
  target_include_directories(test_voronoi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

  ament_add_gtest(test_brv_algorithm test/test_brv_algorithm.cpp
    src/brv_algorithm.cpp src/mbb.cpp src/grid_coverage.cpp
    src/voronoi_roadmap.cpp src/geometry_utils.cpp)
  target_include_directories(test_brv_algorithm PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

  ament_add_gtest(test_coverage_scenarios test/test_coverage_scenarios.cpp
    src/brv_algorithm.cpp src/mbb.cpp src/grid_coverage.cpp
    src/voronoi_roadmap.cpp src/geometry_utils.cpp)
  target_include_directories(test_coverage_scenarios PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})
endif()

ament_package()
