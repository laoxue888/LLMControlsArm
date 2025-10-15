cmake_minimum_required(VERSION 3.10.2)
project(robot_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(urdf REQUIRED)

install(
  DIRECTORY
    meshes
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()