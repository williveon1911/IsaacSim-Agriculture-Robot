cmake_minimum_required(VERSION 3.8)
project(agri_robot_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME} OPTIONAL)


ament_package()
