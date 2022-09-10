#include <boost/program_options.hpp>
#include <ros/ros.h>
#include "ros_transfer_utils.h"

namespace po = boost::program_options;

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros2img");
    ros::NodeHandle nh;

    std::string bag_file;
    std::string color_topic_name;
    std::string depth_topic_name;
    std::string save_dir;

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

    star::SaveRGBDImageFromROSBag(bag_file, color_topic_name, depth_topic_name, save_dir);
}