#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <boost/filesystem.hpp>
#include "ros_rgbd_synchronizer.h"

namespace encodings = sensor_msgs::image_encodings;
namespace po = boost::program_options;

int main(int argc, char** argv) {
    // 1. Initialize ROS
    ros::init(argc, argv, "ros2img_sync");

    std::string bag_file;
    std::string color_topic_name;
    std::string depth_topic_name;
    std::string camera_info_topic_name;
    std::string save_dir;
    unsigned start_index;
    unsigned end_index;
    unsigned step;

    // 2. Parse arguments
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("bag_file", po::value<std::string>(&bag_file), "bag file")
        ("color_topic_name", po::value<std::string>(&color_topic_name), "color topic name")
        ("depth_topic_name", po::value<std::string>(&depth_topic_name), "depth topic name")
        ("camera_info_topic_name", po::value<std::string>(&camera_info_topic_name), "camera info topic name")
        ("save_dir", po::value<std::string>(&save_dir), "save dir")
        ("start_index", po::value<unsigned>(&start_index), "start index")
        ("end_index", po::value<unsigned>(&end_index), "end index")
        ("step", po::value<unsigned>(&step), "step")
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

    if (vm.count("camera_info_topic_name")) {
        std::cout << "camera_info_topic_name: " << vm["camera_info_topic_name"].as<std::string>() << std::endl;
    } else {
        std::cout << "camera_info_topic_name is not set, using default: /camera_info" << std::endl;
        camera_info_topic_name = "/camera_info";
    }

    if (vm.count("save_dir")) {
        std::cout << "save_dir: " << vm["save_dir"].as<std::string>() << std::endl;
    } else {
        std::cout << "save_dir is not set." << std::endl;
        return 1;
    }

    if (vm.count("start_index")) {
        std::cout << "start_index: " << vm["start_index"].as<unsigned>() << std::endl;
    } else {
        std::cout << "start_index is not set, using default 0" << std::endl;
        start_index = 0;
    }

    if (vm.count("end_index")) {
        std::cout << "end_index: " << vm["end_index"].as<unsigned>() << std::endl;
    } else {
        std::cout << "end_index is not set, using default 0" << std::endl;
        end_index = 0;
    }

    if (vm.count("step")) {
        std::cout << "step: " << vm["step"].as<unsigned>() << std::endl;
    } else {
        std::cout << "step is not set, using default 1" << std::endl;
        step = 1;
    }

    // 3. Create a ROSRGBDSynchronizer
    star::star_ros::ROSRGBDSynchronizer synchronizer(bag_file, color_topic_name, depth_topic_name, camera_info_topic_name);
    synchronizer.SyncFromROSBag();
    synchronizer.SaveToImage(save_dir, start_index, end_index, step);
}