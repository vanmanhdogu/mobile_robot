cmake_minimum_required(VERSION 3.5)
project(semi_humanoid_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS
    scripts/scan_fix_relay.py
    scripts/odom_slam_test.py
    scripts/odom_slam_plot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
