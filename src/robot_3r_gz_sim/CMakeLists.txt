cmake_minimum_required(VERSION 3.8)
project(robot_3r_gz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_3r_description REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(ros_gz_sim REQUIRED)

install(DIRECTORY
  launch models scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/spawn_obstacles.py
  DESTINATION lib/${PROJECT_NAME}
)

# Ensure scripts in the source tree are executable during development/configure
# execute_process(COMMAND ${CMAKE_COMMAND} -E chmod +x ${CMAKE_SOURCE_DIR}/src/robot_3r_gz_sim/scripts/spawn_obstacles.py
#                 RESULT_VARIABLE _chmod_result)
# if(NOT _chmod_result EQUAL 0)
#   message(WARNING "Failed to chmod spawn_obstacles.py during configure (result=${_chmod_result})")
# endif()


ament_package()