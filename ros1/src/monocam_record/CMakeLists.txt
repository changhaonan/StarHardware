cmake_minimum_required(VERSION 2.8.3)
project(monocam_record)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_filters rosbag cv_bridge)
find_package(Boost 1.50)
find_package(OpenCV 4.2 REQUIRED)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs message_filters rosbag cv_bridge
    DEPENDS Boost
)

## Yaml set
set(YAML_CPP_LIBRARIES "/usr/local/lib/libyaml-cpp.so")
set(YAML_CPP_INCLUDE_DIRS "/usr/local/include/")

## Eigen set
set(Eigen_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../external/eigen")

## Context set
set(context_dir "${CMAKE_CURRENT_LIST_DIR}/../../../external/Easy3DViewer/utils/cpp/")

## Set include path
include_directories(include 
    ${Boost_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS} 
    ${YAML_CPP_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS} 
    ${Eigen_INCLUDE_DIRS}
    ${context_dir})

## Add library
add_library(ros_transfer_utils src/ros_transfer_utils.cpp)
target_link_libraries(ros_transfer_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Add executable
add_executable(ros2img src/ros2img.cpp)
target_link_libraries(ros2img 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ros_transfer_utils)

## Install
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)