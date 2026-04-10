cmake_minimum_required(VERSION 3.8)
project(mowgli_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Install resource directories
# ---------------------------------------------------------------------------
install(
  DIRECTORY
    launch
    config
    urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/wait_for_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  # Integration test: verify all nodes start and advertise their public API
  add_launch_test(test/test_nodes_startup.launch.py
    TIMEOUT 60
  )

endif()

ament_package()
