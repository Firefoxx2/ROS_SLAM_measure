cmake_minimum_required(VERSION 3.0.2)
project(r1_slam_measure)

find_package(catkin 
REQUIRED COMPONENTS
  rospy
  std_msgs
  tf
)

catkin_package(
CATKIN_DEPENDS 
 rospy 
 std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/measure.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
