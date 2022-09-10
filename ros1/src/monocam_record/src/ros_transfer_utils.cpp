#include "ros_transfer_utils.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/image_encodings.h>

namespace encodings = sensor_msgs::image_encodings;

void star::SaveTopicImageFromROSBag(
    rosbag::Bag& bag, 
    const std::string& topic_name, 
    const std::string& save_dir,
    const std::string& encoding) {

    // 1. Create view
    rosbag::View view(bag, rosbag::TopicQuery(topic_name));

    // 2. Prepare
    Easy3DViewer::FileType file_type;
    if (encoding == encodings::RGB8 || encoding == encodings::BGR8) {
        file_type = Easy3DViewer::color_img_file;
    } else if (encoding == encodings::TYPE_16UC1) {
        file_type = Easy3DViewer::depth_img_file;
    } else {
        printf("Encoding is not supported.\n");
        assert(false);
    }

    // 3. Go through the bag
    unsigned frame_idx = 0;
    for (rosbag::MessageInstance const m : view) {
        sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
        if (img != nullptr) {
            // 3.1. Generate file name
            std::string img_name = Easy3DViewer::FileNameVolumeDeform(
                boost::filesystem::path(save_dir), 0, frame_idx, file_type).string();

            // 3.2. Generate image as mat
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(img, encoding);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            cv::Mat image = cv_ptr->image;

            // 3.3. Save image
            cv::imwrite(img_name, image);
            frame_idx++;
        }
    }

    bag.close();
}

void star::SaveRGBDImageFromROSBag(
    const std::string& bag_file, 
    const std::string& color_topic_name,
    const std::string& depth_topic_name,  
    const std::string& save_dir) {
    
    // 1. Open bag file
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);

    // 2. Save color image
    SaveTopicImageFromROSBag(bag, color_topic_name, save_dir, encodings::RGB8); 

    // 3. Save depth image
    SaveTopicImageFromROSBag(bag, depth_topic_name, save_dir, encodings::TYPE_16UC1);
}