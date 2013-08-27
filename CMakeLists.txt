cmake_minimum_required(VERSION 2.8.3)
project(kinect_pendulum_demo)

find_package(catkin
  REQUIRED
  std_msgs
  skeletonmsgs_nu
  tf
)

generate_messages(DEPENDENCIES std_msgs skeletonmsgs_nu)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  skeletonmsgs_nu
  tf
)
