cmake_minimum_required(VERSION 3.8)
project(agri_robot_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install Python tracker nodes as ROS2 executables.
install(PROGRAMS agri_tracker/tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pure_pursuit_tracker
)
install(PROGRAMS agri_tracker/fake_odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_odom_publisher
)
install(PROGRAMS agri_tracker/data_collector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME data_collector
)
install(PROGRAMS agri_tracker/evaluator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tracker_evaluator
)
install(PROGRAMS agri_tracker/tuner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tracker_tuner
)
install(PROGRAMS agri_tracker/visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tracker_visualizer
)

# Install directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_tracker_nodes test/test_tracker_nodes.py TIMEOUT 60)
endif()

ament_package()
