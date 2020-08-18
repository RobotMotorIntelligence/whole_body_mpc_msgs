CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(riccati_controller)

# Set up project properties
SET(PROJECT_NAME riccati_controller)
SET(PROJECT_DESCRIPTION "Riccati controller and its messages")

# Print initial message
MESSAGE(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
MESSAGE("Copyright (C) 2020 University of Edinburgh")
MESSAGE("All rights reserved.")
MESSAGE("Released under the BSD 3-Clause License.")

# Set a default build type to 'Release' if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# Find required packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  state_msgs
  message_generation)

catkin_python_setup()

# Define messages to be generated
add_message_files(
   DIRECTORY msg
   FILES
    RiccatiGain.msg
    RiccatiControllerReference.msg)

generate_messages(
   DEPENDENCIES
   std_msgs
   state_msgs)


# Define catkin dependencies
catkin_package(CATKIN_DEPENDS  roscpp message_runtime std_msgs state_msgs)


# Installation src
INSTALL(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
