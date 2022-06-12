cmake_minimum_required(VERSION 2.8.3)
project(multicam_calibration)

# CMAKE PrefixList
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake")
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu")
list(APPEND CMAKE_PREFIX_PATH "/usr/local")
list(APPEND CMAKE_PREFIX_PATH "/usr/share")
list(APPEND CMAKE_PREFIX_PATH "/usr/include")
list(APPEND CMAKE_PREFIX_PATH "/usr")

## Find catkin and any catkin packages
# find_package(Eigen 3.3.7 EXACT REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_filters rosbag cv_bridge apriltag_ros apriltag_msgs)
find_package(Boost 1.50)
find_package(PCL REQUIRED COMPONENTS common io registration search kdtree)
# find_package(PCL REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(Ceres REQUIRED)

## Yaml set
set(YAML_CPP_LIBRARIES "/usr/local/lib/libyaml-cpp.so")
set(YAML_CPP_INCLUDE_DIRS "/usr/local/include/")

## Context set
set(context_dir "${CMAKE_CURRENT_LIST_DIR}/../../../../external/Easy3DViewer/utils/cpp/")

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs message_filters rosbag cv_bridge apriltag_ros apriltag_msgs
    DEPENDS Boost
)

## Set include path
include_directories(include ${Boost_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS} 
    ${YAML_CPP_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS} 
    ${context_dir}
    ${CERES_INCLUDE_DIRS})

## Add Library
add_library(calibration_dectector multicam_apriltag_detector.cpp)
target_link_libraries(calibration_dectector ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(calibrator calibrator.cpp)
target_link_libraries(calibrator 
    ${catkin_LIBRARIES} 
    ${YAML_CPP_LIBRARIES} 
    ${Boost_LIBRARIES}
    ${CERES_LIBRARIES})

## Add Executable
add_executable(load_calibration_data load_calibration_data.cpp)
target_link_libraries(load_calibration_data 
    calibration_dectector
    calibrator
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${YAML_CPP_LIBRARIES}
    ${OpenCV_LIBRARIES})

add_executable(ros2web ros2web.cpp)
target_link_libraries(ros2web
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES} 
    ${YAML_CPP_LIBRARIES}
    ${OpenCV_LIBRARIES})

add_executable(calibration_from_file calibration_from_file.cpp)
target_link_libraries(calibration_from_file
    calibrator
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${OpenCV_LIBRARIES})

add_executable(calibration_icp calibration_icp.cpp)
target_link_libraries(calibration_icp
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES} 
    ${YAML_CPP_LIBRARIES}
    ${OpenCV_LIBRARIES})