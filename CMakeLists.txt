cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  tf
  vikit_ros
)

## For eigen
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

## messages
############################################
find_package(message_generation)

add_message_files(
   FILES
   adafruit_msg.msg
)
 
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kalman_filter
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

###################
# create kf library
###################
# set source files
LIST(APPEND SOURCEFILES
	src/HelperFunc.cpp
	src/KalmanFilter.cpp
	src/KalmanOrientation.cpp
	src/MyMatMath.cpp
	src/ValidationGuard.cpp)
ADD_LIBRARY(kf ${SOURCEFILES})
TARGET_LINK_LIBRARIES(kf ${catkin_LIBRARIES})

###############
# kf executable
###############
add_executable(kalman_filter
  src/main.cpp
)

target_link_libraries(kalman_filter
   ${catkin_LIBRARIES}
   kf
)
