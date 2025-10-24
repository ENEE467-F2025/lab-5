cmake_minimum_required(VERSION 3.8)
project(lab_moveit_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pymoveit2 REQUIRED)

install(DIRECTORY
  scripts launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
    scripts/ur3_pose_goal.py
    scripts/ur3_joint_goal.py
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()