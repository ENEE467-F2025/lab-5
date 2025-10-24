cmake_minimum_required(VERSION 3.8)
project(robot_3r_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
    scripts/test_docker.py
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()