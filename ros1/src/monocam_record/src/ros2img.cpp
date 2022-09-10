#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <context_utils.hpp>

namespace encodings = sensor_msgs::image_encodings;
namespace po = boost::program_options;

void SaveTopicImageFromROSBag(
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
}

void SaveRGBDImageFromROSBag(
    rosbag::Bag& bag,  
    const std::string& color_topic_name,
    const std::string& depth_topic_name,  
    const std::string& save_dir) {
    // 1. Save color image
    SaveTopicImageFromROSBag(bag, color_topic_name, save_dir, encodings::RGB8); 

    // 2. Save depth image
    SaveTopicImageFromROSBag(bag, depth_topic_name, save_dir, encodings::TYPE_16UC1);
}

int main(int argc, char** argv) {
    // 1. Initialize ROS
    ros::init(argc, argv, "ros2img");
    ros::NodeHandle nh;

    std::string bag_file;
    std::string color_topic_name;
    std::string depth_topic_name;
    std::string save_dir;

    // 2. Parse arguments
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("bag_file", po::value<std::string>(&bag_file), "bag file")
        ("color_topic_name", po::value<std::string>(&color_topic_name), "color topic name")
        ("depth_topic_name", po::value<std::string>(&depth_topic_name), "depth topic name")
        ("save_dir", po::value<std::string>(&save_dir), "save dir")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    if (vm.count("bag_file")) {
        std::cout << "bag_file: " << vm["bag_file"].as<std::string>() << std::endl;
    } else {
        std::cout << "bag_file is not set." << std::endl;
        return 1;
    }

    if (vm.count("color_topic_name")) {
        std::cout << "color_topic_name: " << vm["color_topic_name"].as<std::string>() << std::endl;
    } else {
        std::cout << "color_topic_name is not set." << std::endl;
        return 1;
    }

    if (vm.count("depth_topic_name")) {
        std::cout << "depth_topic_name: " << vm["depth_topic_name"].as<std::string>() << std::endl;
    } else {
        std::cout << "depth_topic_name is not set." << std::endl;
        return 1;
    }

    if (vm.count("save_dir")) {
        std::cout << "save_dir: " << vm["save_dir"].as<std::string>() << std::endl;
    } else {
        std::cout << "save_dir is not set." << std::endl;
        return 1;
    }

    // 3. Open bag file
    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
        ROS_INFO("Bag file opened.");
    } catch (rosbag::BagException& e) {
        ROS_ERROR("Failed to open bag file: %s", e.what());
        return -1;
    }
    
    // 4. Save to image
    SaveRGBDImageFromROSBag(bag, color_topic_name, depth_topic_name, save_dir);

    bag.close();
}