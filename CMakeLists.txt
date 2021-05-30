cmake_minimum_required(VERSION 3.7.2)
project(serving_fluids)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${catkin_INCLUE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    rospy
    gazebo_ros
)

install(
  PROGRAMS
    scripts/milestone_1.py
    scripts/pick.py
    scripts/place.py
    scripts/stow.py
    scripts/tf_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch maps config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
