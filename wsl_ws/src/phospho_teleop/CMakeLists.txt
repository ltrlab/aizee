cmake_minimum_required(VERSION 3.5)
project(phospho_teleop)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  teleop_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
