cmake_minimum_required(VERSION 3.0.2)
project(safe_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_srvs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Status.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/safe_robot_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
