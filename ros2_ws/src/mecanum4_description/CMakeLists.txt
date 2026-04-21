cmake_minimum_required(VERSION 3.8)
project(mecanum4_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/cmd_vel_to_wheels.py
    scripts/teleop_gui.py
    scripts/amcl_kidnap_spin.py
    scripts/docking_node.py
    scripts/camera_docking_node.py
    scripts/fused_docking_node.py
    scripts/cam_lidar_docking_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
