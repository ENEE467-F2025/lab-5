cmake_minimum_required(VERSION 3.22)
project(ur3e_hande_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch scripts srdf
  DESTINATION share/${PROJECT_NAME}
)

# placeholder to install scripts
# install(PROGRAMS 
#           scripts/*.py
#    DESTINATION lib/${PROJECT_NAME}
#  )

ament_package()