# cmake_minimum_required(VERSION 2.8.3)
# project(imu_to_odom)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   # message_generation
#   nav_msgs
#   geometry_msgs
#   roslib
#   tf
#   minkindr
#   minkindr_conversions
# )

# add_definitions(-std=gnu++11 -o3)

# catkin_package(
#  INCLUDE_DIRS 
#  include
# #  LIBRARIES rosbag
# #  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  CATKIN_DEPENDS 
#  roscpp   
#  message_runtime
#  geometry_msgs
#  nav_msgs
#  roslib
#  minkindr
#  minkindr_conversions
# #  DEPENDS system_lib
# )

# include_directories(
# include
#   ${catkin_INCLUDE_DIRS}
# )

# #############
# # LIBRARIES #
# #############
# add_library(${PROJECT_NAME}
#   src/imu_to_odom.cpp
# )

# ############
# # BINARIES #
# ############
# add_executable(imu_to_odom_node
#   src/imu_to_odom_node.cpp
# )
# target_link_libraries(imu_to_odom_node ${PROJECT_NAME})

