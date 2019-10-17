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
        actionlib_msgs
        ropod_ros_msgs
        OpenCV
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
        include/Definitions
        include/LoopRate
        include/Model
        include/Obstacles
        include/Tube
        include/Visualization
        # LIBRARIES napoleon_navigation
        CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs roscpp tf costmap_2d maneuver_planner actionlib_msgs ropod_ros_msgs
        # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        include/Definitions
        include/LoopRate
        include/Model
        include/Obstacles
        include/Tube
        include/Visualization
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src-old/${PROJECT_NAME}/napoleon_navigation.cpp
# )

SET(napoleon_includes
        include/Model/Model.cpp
        include/Model/Model.h
        include/Tube/Tube.cpp
        include/Tube/Tube.h
        include/Tube/Tubes.cpp
        include/Tube/Tubes.h
        include/Model/HolonomicModel.cpp
        include/Model/HolonomicModel.h
        include/Model/BicycleModel.cpp
        include/Model/BicycleModel.h
        include/Obstacles/Obstacle.cpp
        include/Obstacles/Obstacle.h
        include/Visualization/Visualization.cpp
        include/Visualization/Visualization.h
        include/Definitions/Vector2D.h
        include/Definitions/Polygon.h
        include/Definitions/Pose2D.h
        include/Definitions/Line.h
        include/Definitions/Circle.h
        include/LoopRate/LoopRate.h
        include/Visualization/VisualizationRviz.cpp
        include/Visualization/VisualizationRviz.h
        include/Visualization/VisualizationOpenCV.cpp
        include/Visualization/VisualizationOpenCV.h)

add_executable(napoleon_navigation NapoleonNavigation.cpp ${napoleon_includes} )
add_executable(napoleon_navigation_standalone NapoleonNavigationStandalone.cpp ${napoleon_includes} )
target_link_libraries(napoleon_navigation ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(napoleon_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})