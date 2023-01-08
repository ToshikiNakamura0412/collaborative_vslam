cmake_minimum_required(VERSION 3.0.2)
project(collaborative_vslam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -O2 -g -Wall)
add_compile_options(-std=c++17 -O2 -g -Wall)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf
    cv_bridge
    pcl_ros
    darknet_ros_msgs
    color_detector_msgs
    object_detector_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES collaborative_vslam
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(roomba_mask_creator     src/roomba_mask_creator.cpp)
add_library(scale_ratio_initializer src/scale_ratio_initializer.cpp)
add_library(collaborative_vslam     src/collaborative_vslam.cpp)
add_library(point                   src/point.cpp)

## Declare a C++ executable
add_executable(roomba_mask_creator_node     src/roomba_mask_creator_node.cpp)
add_executable(scale_ratio_initializer_node src/scale_ratio_initializer_node.cpp)
add_executable(collaborative_vslam_node     src/collaborative_vslam_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(roomba_mask_creator_node     ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(scale_ratio_initializer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(collaborative_vslam_node     ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(roomba_mask_creator_node       roomba_mask_creator ${catkin_LIBRARIES})
target_link_libraries(scale_ratio_initializer_node   scale_ratio_initializer ${catkin_LIBRARIES})
target_link_libraries(collaborative_vslam_node collaborative_vslam point ${catkin_LIBRARIES})
