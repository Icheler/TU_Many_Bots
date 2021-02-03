cmake_minimum_required(VERSION 3.0.2)
project(tmb_messages)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 rospy
 geometry_msgs
 message_generation
)

add_message_files(
  FILES
  Object_Sighted.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
