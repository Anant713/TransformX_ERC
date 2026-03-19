cmake_minimum_required(VERSION 3.8)
project(my_bot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()