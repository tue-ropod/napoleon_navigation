cmake_minimum_required(VERSION 2.8.3)
project(napoleon_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
  roscpp
  tf
  costmap_2d
  maneuver_planner
  pluginlib
  nav_core
  ed  
  ed_gui_server
  geolib2
  actionlib_msgs
  ropod_ros_msgs
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
  ropod_ros_msgs
)


catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES napoleon_navigation
  CATKIN_DEPENDS  message_runtime std_msgs geometry_msgs nav_msgs roscpp  tf costmap_2d maneuver_planner actionlib_msgs ropod_ros_msgs
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/napoleon_navigation.cpp
# )

add_executable(napoleon_navigation src/napoleon_navigation_rosnode.cpp src/napoleon_geometry.cpp src/napoleon_functions.cpp )
target_link_libraries(napoleon_navigation ${catkin_LIBRARIES})
add_dependencies(napoleon_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


