cmake_minimum_required(VERSION 3.8)
project(robot_3r_moveit_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pymoveit2 REQUIRED)
find_package(robot_3r_description REQUIRED)
find_package(robot_3r_moveit_config REQUIRED)

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
    scripts/pose_goal.py
    scripts/cartesian_path.py
    scripts/joint_goal.py
    scripts/jsp_path.py
    scripts/moveit_fk.py
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()