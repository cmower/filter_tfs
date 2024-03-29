cmake_minimum_required(VERSION 3.0.2)
project(filter_tfs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  tf2_ros
)

catkin_package()

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/single
  scripts/apriltags
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
