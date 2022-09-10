#include "ros_rgbd_synchronizer.h"

star::star_ros::ROSRGBDSynchronizer::ROSRGBDSynchronizer(
    const std::string& bag_name, 
    const std::string& rgb_topic_name, 
    const std::string& depth_topic_name, 
    const int queue_size)
    :
    m_bag_name(bag_name),
    m_rgb_topic_name(rgb_topic_name),
    m_depth_topic_name(depth_topic_name),
    m_sync(star::star_ros::ApproxSyncPolicy_2(queue_size), m_rgb_sub, m_depth_sub)
{
    m_sync.registerCallback(boost::bind(&ROSRGBDSynchronizer::callback, this, _1, _2));
}

void star::star_ros::ROSRGBDSynchronizer::SyncFromROSBag() {
    // 1. Open bag & create view
    rosbag::Bag bag;
    bag.open(m_bag_name, rosbag::bagmode::Read);
    std::vector<std::string> topics = {m_rgb_topic_name, m_depth_topic_name};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance& m : view) {
        if (m.getTopic() == m_rgb_topic_name || ("/" + m.getTopic() == m_rgb_topic_name)) {
            sensor_msgs::Image::ConstPtr rgb_image = m.instantiate<sensor_msgs::Image>();
            if (rgb_image != nullptr) {
                m_rgb_sub.newMessage(rgb_image);
            }
        }
        if (m.getTopic() == m_depth_topic_name || ("/" + m.getTopic() == m_depth_topic_name)) {
            sensor_msgs::Image::ConstPtr depth_image = m.instantiate<sensor_msgs::Image>();
            if (depth_image != nullptr) {
                m_depth_sub.newMessage(depth_image);
            }
        }
    }
    bag.close();
}

void star::star_ros::ROSRGBDSynchronizer::callback(const ImageMsg::ConstPtr& rgb_msg, const ImageMsg::ConstPtr& depth_msg) {
    RgbDepthPair pair;
    pair.rgb_msg = rgb_msg;
    pair.depth_msg = depth_msg;
    m_rgb_depth_pair_list.push_back(pair);
}

void star::star_ros::ROSRGBDSynchronizer::SaveToImage(const std::string &save_dir, const unsigned start_index, const unsigned end_index, const unsigned step) {
    if (m_rgb_depth_pair_list.empty()) {
        ROS_ERROR("No data to save!");
        return;
    }
    if (end_index > m_rgb_depth_pair_list.size()) {
        ROS_ERROR("End index is out of range!");
        return;
    }
    if (start_index > end_index) {
        ROS_ERROR("Start index is out of range!");
        return;
    }
    if (step == 0) {
        ROS_ERROR("Step is 0!");
        return;
    }
    
    if (end_index == 0) {
        ROS_INFO("End index is 0, save all data!");
    }
    unsigned true_end_index = std::max(end_index, (unsigned)m_rgb_depth_pair_list.size());

    unsigned save_index = 0;
    for (unsigned i = start_index; i < true_end_index; i += step) {
        const RgbDepthPair& pair = m_rgb_depth_pair_list[i];
        cv_bridge::CvImageConstPtr rgb_cv_ptr = cv_bridge::toCvShare(pair.rgb_msg);
        cv_bridge::CvImageConstPtr depth_cv_ptr = cv_bridge::toCvShare(pair.depth_msg);

        std::string color_img_name = Easy3DViewer::FileNameVolumeDeform(
            boost::filesystem::path(save_dir), 0, save_index, Easy3DViewer::color_img_file).string();
        std::string depth_img_name = Easy3DViewer::FileNameVolumeDeform(
            boost::filesystem::path(save_dir), 0, save_index, Easy3DViewer::depth_img_file).string();

        cv::imwrite(color_img_name, rgb_cv_ptr->image);
        cv::imwrite(depth_img_name, depth_cv_ptr->image);
        save_index++;
    }
}