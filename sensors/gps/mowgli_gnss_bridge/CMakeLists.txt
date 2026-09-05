cmake_minimum_required(VERSION 3.8)
project(mowgli_gnss_bridge)

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
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(universal_gnss_ros2 REQUIRED)

# ---------------------------------------------------------------------------
# Library: mowgli_gnss_bridge_lib
# Split from main.cpp so GTest can link the node without an executable entry.
# ---------------------------------------------------------------------------

add_library(mowgli_gnss_bridge_lib STATIC
  src/universal_gnss_topic_bridge.cpp
)

target_include_directories(mowgli_gnss_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mowgli_gnss_bridge_lib
  rclcpp
  std_msgs
  builtin_interfaces
  diagnostic_msgs
  rtcm_msgs
  mowgli_interfaces
  universal_gnss_ros2
)

# ---------------------------------------------------------------------------
# Executable: universal_gnss_topic_bridge
# ---------------------------------------------------------------------------

add_executable(universal_gnss_topic_bridge
  src/main.cpp
)

target_link_libraries(universal_gnss_topic_bridge
  mowgli_gnss_bridge_lib
)

ament_target_dependencies(universal_gnss_topic_bridge
  rclcpp
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(TARGETS
    universal_gnss_topic_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Match every package under ros2/src: this project formats with clang-format
  # (see .clang-format) and declares licensing with an SPDX identifier, neither
  # of which ament_uncrustify / ament_copyright recognise. Without these this
  # package is the only one in the repo that fails those two linters — which is
  # what the new "Test mowgli_gnss_bridge" CI job surfaced the moment anything
  # started running them.
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplint is deliberately left ENABLED here: it currently passes, and this
  # package is the GNSS trust boundary, so the extra checking is worth keeping.
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_universal_gnss_topic_bridge
    test/test_universal_gnss_topic_bridge.cpp
  )
  target_link_libraries(test_universal_gnss_topic_bridge
    mowgli_gnss_bridge_lib
  )
  ament_target_dependencies(test_universal_gnss_topic_bridge
    rclcpp
    mowgli_interfaces
    universal_gnss_ros2
    diagnostic_msgs
    rtcm_msgs
  )
endif()

ament_package()
