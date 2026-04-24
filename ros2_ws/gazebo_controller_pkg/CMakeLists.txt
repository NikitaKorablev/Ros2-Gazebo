cmake_minimum_required(VERSION 3.8)
project(gazebo_controller_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  gazebo_controller_pkg/wall_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()