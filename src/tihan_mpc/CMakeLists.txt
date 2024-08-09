cmake_minimum_required(VERSION 3.0.2)
project(tihan_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

###########
## MSG ##
###########
add_message_files(
	FILES
	state_est.msg
	mpc_path.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

###########
## Build ##
###########

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)