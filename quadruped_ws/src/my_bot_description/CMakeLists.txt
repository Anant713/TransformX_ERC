cmake_minimum_required(VERSION 3.8)
project(my_bot_description)

find_package(ament_cmake REQUIRED)

# Install all resource directories so ROS can find them with package://
install(
  DIRECTORY urdf meshes config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
