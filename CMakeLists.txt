cmake_minimum_required(VERSION 3.0.2)
project(height_estimator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  rp.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 LIBRARIES height_estimator
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  INCLUDE_DIRS
  include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_executable(h_est_node 
include/height_estimator/converter.cpp
include/height_estimator/coordinate_transform.cpp
include/height_estimator/height_estimator.cpp
include/height_estimator/roll_pitch_extractor.cpp
src/height_est_node.cpp)

target_link_libraries(h_est_node ${catkin_LIBRARIES})

add_executable(attitude_control_node 
include/height_estimator/converter.cpp
include/height_estimator/roll_pitch_extractor.cpp
include/height_estimator/attitude_control.cpp
src/attitude_control_node.cpp)

target_link_libraries(attitude_control_node ${catkin_LIBRARIES})