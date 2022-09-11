#pragma once
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "ros_sync_utils.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <context_utils.hpp>
#include <context.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

namespace star { namespace star_ros {

    class ROSRGBDSynchronizer {
    public:
        ROSRGBDSynchronizer(
            const std::string& bag_name, 
            const std::string& rgb_topic_name, 
            const std::string& depth_topic_name,
            const std::string& camera_info_topic_name, 
            const int queue_size=1000);
        ~ROSRGBDSynchronizer() = default;

        // Public API
        void SyncFromROSBag();
        void SaveToImage(
            const std::string &save_dir, 
            const unsigned start_index, 
            const unsigned end_index, 
            const unsigned step=1);
        struct RgbDepthPair {
            sensor_msgs::Image::ConstPtr rgb_msg;
            sensor_msgs::Image::ConstPtr depth_msg;
        };

    private:
        void callback(const ImageMsg::ConstPtr& rgb_msg, const ImageMsg::ConstPtr& depth_msg);
        BagSubscriber<ImageMsg> m_rgb_sub;
        BagSubscriber<ImageMsg> m_depth_sub;
        ApproxTimeSynchronizer_2 m_sync;
        ros::NodeHandle m_nh;

        // Camera
        Easy3DViewer::RGBDCamera m_rgbd_camera;
        bool m_camera_initialized = false;
        
        // Config member
        std::string m_bag_name;
        std::string m_rgb_topic_name;
        std::string m_depth_topic_name;
        std::string m_camera_info_topic_name;

        std::vector<RgbDepthPair> m_rgb_depth_pair_list;
    };
}
}