CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(whole_body_mpc_msgs)

# Set up project properties
SET(PROJECT_NAME whole_body_mpc_msgs)
SET(PROJECT_DESCRIPTION "Whole-body MPC messages")

# Print initial message
MESSAGE(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
MESSAGE(STATUS "Copyright (C) 2020 University of Edinburgh")
MESSAGE(STATUS "All rights reserved.")
MESSAGE(STATUS "Released under the BSD 3-Clause License.")

# Find required packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  whole_body_state_msgs
  message_generation
)

# Define messages to be generated
add_message_files(
  DIRECTORY msg
  FILES
   StateFeedbackGain.msg
   WholeBodyMpcReference.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  whole_body_state_msgs
)

# Define catkin dependencies
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs whole_body_state_msgs
)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
