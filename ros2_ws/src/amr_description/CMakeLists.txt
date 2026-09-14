cmake_minimum_required(VERSION 3.8)
project(amr_description)

# =============================================================================
# This file EXTENDS the original `ros2 pkg create --build-type ament_cmake`
# skeleton that was in the repo. Everything the skeleton had is preserved:
#   - cmake_minimum_required 3.8          (unchanged)
#   - project(amr_description)            (unchanged)
#   - the GNUCXX/Clang warning block      (unchanged)
#   - find_package(ament_cmake REQUIRED)  (unchanged)
#   - the whole BUILD_TESTING block       (unchanged)
#   - ament_package()                     (unchanged, still last)
#
# Added below: interface generation, the amr_fleet Python package, the
# executable entry points, and the launch/config installs. The Python package
# is installed with plain CMake because current ament_cmake_python documentation
# warns against combining ament_python_install_package() with rosidl generation
# in the same project.
#
# The commented-out urdf/meshes install block near the bottom is ready for the
# robot-model work; uncomment it when those directories exist.
# =============================================================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------------------- deps -----
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)     # installs the amr_fleet package
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)

# ----------------------------------------------------- custom interfaces ----
# NOTE ON NAMING: this generates a Python module literally called
# `amr_description`, so nodes write `from amr_description.msg import
# RobotState`. That is exactly WHY the hand-written Python package is named
# `amr_fleet` and not `amr_description` - two modules of the same name would
# install to the same site-packages path and one would silently shadow the
# other. See docs/PACKAGE_LAYOUT.md.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/Intent.msg"
  "msg/ZoneRequest.msg"
  "msg/ZoneGrant.msg"
  "msg/MapUpdate.msg"
  "msg/Task.msg"
  "msg/TaskBid.msg"
  "msg/TaskAward.msg"
  "msg/TaskComplete.msg"
  "msg/Conflict.msg"
  "msg/MotionPermit.msg"
  "srv/Replan.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# ------------------------------------------------------ python package ------
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY amr_fleet DESTINATION ${PYTHON_INSTALL_DIR})

# ------------------------------------------------------ executables ---------
# MUST install to lib/${PROJECT_NAME} - that is where `ros2 run` looks.
install(PROGRAMS
  scripts/fleet_agent
  scripts/task_generator
  scripts/fleet_monitor
  scripts/mock_robot
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------ data files ----------
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# --- ROBOT MODEL (not yet present in the repo) -------------------------------
# amr_description currently contains no urdf/ or meshes/ directory. When the
# robot-model work lands, uncomment this. Leaving it commented avoids a
# configure-time error on a directory that does not exist.
#
# install(DIRECTORY
#   urdf
#   meshes
#   rviz
#   DESTINATION share/${PROJECT_NAME}
# )

# ------------------------------------------------------ testing -------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Our pure-Python unit tests. They need no ROS and no simulator, so they run
  # in a fraction of a second as part of `colcon test`.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(amr_fleet_tests test
    TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

# Lets OTHER packages (amr_navigation, a dashboard) import our messages.
ament_export_dependencies(rosidl_default_runtime)

ament_package()
