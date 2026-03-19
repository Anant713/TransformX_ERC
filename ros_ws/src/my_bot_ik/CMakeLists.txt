cmake_minimum_required(VERSION 3.8)
project(my_bot_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(ik_node src/ik_node.cpp)
ament_target_dependencies(ik_node
  rclcpp
  geometry_msgs
  trajectory_msgs
  sensor_msgs
)

add_executable(foot_trajectory_node src/foot_trajectory_node.cpp)
ament_target_dependencies(foot_trajectory_node
  rclcpp
  geometry_msgs
)

install(TARGETS
  ik_node
  foot_trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()