cmake_minimum_required(VERSION 3.5)
project(semi_humanoid_v2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
