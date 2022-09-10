#pragma once
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <context_utils.hpp>

namespace star {

    // Save image from ROS bag
    void SaveTopicImageFromROSBag(
        rosbag::Bag& bag, 
        const std::string& topic_name, 
        const std::string& save_dir,
        const std::string& encoding = "bgr8");

    // Save color & depth image from ROS bag
    void SaveRGBDImageFromROSBag(
        const std::string& bag_file, 
        const std::string& color_topic_name,
        const std::string& depth_topic_name,  
        const std::string& save_dir);

}