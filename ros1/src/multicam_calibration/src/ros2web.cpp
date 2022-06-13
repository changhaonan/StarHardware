#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include "types.h"
#include "context.hpp"
#include "context_utils.hpp"


namespace multicam_calibration {
    // Global setting
    constexpr unsigned num_camera = 3;

    // Type define
    using PointCloudMsg = sensor_msgs::PointCloud2;
    using ImageMsg = sensor_msgs::Image;
    typedef message_filters::sync_policies::ApproximateTime<
        PointCloudMsg, PointCloudMsg, PointCloudMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg> ApproxSyncPolicy9;
    typedef message_filters::Synchronizer<ApproxSyncPolicy9> ApproxTimeSynchronizer9;

    // Sync for at most 3 cameras
    std::unique_ptr<ApproxTimeSynchronizer9> approx_sync_;

    // Global data ptr buffer
    std::vector<MultiCalibrationData<num_camera>> multi_calibration_dataset_;
    std::vector<MultiPointCloudData<num_camera>> multi_point_cloud_dataset_;

    /**
     * Inherits from message_filters::SimpleFilter<M>
     * to use protected signalMessage function
     */
    template <class M>
    class BagSubscriber : public message_filters::SimpleFilter<M> {
    public:
        void newMessage(const boost::shared_ptr<M const>& msg) {
            this->signalMessage(msg);
        }
    };


    // Callback for synchronized messages for 3 cameras
    void callback(
        const sensor_msgs::PointCloud2::ConstPtr& point_cloud_0,
        const sensor_msgs::PointCloud2::ConstPtr& point_cloud_1,
        const sensor_msgs::PointCloud2::ConstPtr& point_cloud_2,
        const sensor_msgs::Image::ConstPtr& depth_image_0,
        const sensor_msgs::Image::ConstPtr& depth_image_1,
        const sensor_msgs::Image::ConstPtr& depth_image_2,
        const sensor_msgs::Image::ConstPtr& color_image_0,
        const sensor_msgs::Image::ConstPtr& color_image_1,
        const sensor_msgs::Image::ConstPtr& color_image_2) {
        MultiCalibrationData<3> calibrate_data;
        calibrate_data.set_camera_data(0, depth_image_0, color_image_0);
        calibrate_data.set_camera_data(1, depth_image_1, color_image_1);
        calibrate_data.set_camera_data(2, depth_image_2, color_image_2);
        MultiPointCloudData<3> point_cloud_data;
        point_cloud_data.set_camera_data(0, point_cloud_0);
        point_cloud_data.set_camera_data(1, point_cloud_1);
        point_cloud_data.set_camera_data(2, point_cloud_2);

        multi_calibration_dataset_.push_back(calibrate_data);
        multi_point_cloud_dataset_.push_back(point_cloud_data);
    }


    // PointCloud transfer
    template <typename PointT>
    std::shared_ptr<pcl::PointCloud<PointT>> Ros2Pcl(const sensor_msgs::PointCloud2::ConstPtr& input) {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);
        std::shared_ptr<pcl::PointCloud<PointT>> pcl_cloud(new pcl::PointCloud<PointT>);
        pcl::fromPCLPointCloud2<PointT>(pcl_pc2, *pcl_cloud);
        return pcl_cloud;
    }


    Eigen::Matrix4d ParseMatrix4d(const YAML::Node& matrix_node) {
        Eigen::Matrix4d output_matrix;
        output_matrix <<
            matrix_node[0][0].as<double>(), matrix_node[0][1].as<double>(), matrix_node[0][2].as<double>(), matrix_node[0][3].as<double>(),
            matrix_node[1][0].as<double>(), matrix_node[1][1].as<double>(), matrix_node[1][2].as<double>(), matrix_node[1][3].as<double>(),
            matrix_node[2][0].as<double>(), matrix_node[2][1].as<double>(), matrix_node[2][2].as<double>(), matrix_node[2][3].as<double>(),
            matrix_node[3][0].as<double>(), matrix_node[3][1].as<double>(), matrix_node[3][2].as<double>(), matrix_node[3][3].as<double>();
        return output_matrix;
    }


    // Load bag
    template<unsigned num_camera>
    void TransferData(
        const std::string& data_name,
        const std::string& filename,
        const std::string& io_config_path,
        const std::string& camera_pose_path,
        const std::string& dump_root_path,
        unsigned frame_start,
        unsigned frame_end) {
        // Initialization
        multi_calibration_dataset_.clear();
        multi_point_cloud_dataset_.clear();

        // Pose
        YAML::Node io_config = YAML::LoadFile(io_config_path);
        YAML::Node cam_pose_config = YAML::LoadFile(camera_pose_path);

        Eigen::Matrix4d cam_to_worlds[num_camera];
        for (auto i = 0; i < num_camera; ++i) {
            std::string cam_name = "cam-" + std::to_string(i);
            cam_to_worlds[i] = ParseMatrix4d(cam_pose_config[cam_name].as<YAML::Node>());
        }

        rosbag::Bag bag;
        bag.open(filename, rosbag::bagmode::Read);

        // Topics: PointCloud
        std::string point_cloud_topic_0 = io_config["cam-0"]["point_cloud_topic"].as<std::string>();
        std::string point_cloud_topic_1 = io_config["cam-1"]["point_cloud_topic"].as<std::string>();
        std::string point_cloud_topic_2 = io_config["cam-2"]["point_cloud_topic"].as<std::string>();
        // Topics: Color & Depth
        std::string depth_image_topic_0 = io_config["cam-0"]["depth_topic"].as<std::string>();
        std::string depth_image_topic_1 = io_config["cam-1"]["depth_topic"].as<std::string>();
        std::string depth_image_topic_2 = io_config["cam-2"]["depth_topic"].as<std::string>();
        std::string color_image_topic_0 = io_config["cam-0"]["color_topic"].as<std::string>();
        std::string color_image_topic_1 = io_config["cam-1"]["color_topic"].as<std::string>();
        std::string color_image_topic_2 = io_config["cam-2"]["color_topic"].as<std::string>();

        // PointCloud2 topics to load
        std::vector<std::string> topics;
        topics.push_back(point_cloud_topic_0);
        topics.push_back(point_cloud_topic_1);
        topics.push_back(point_cloud_topic_2);
        topics.push_back(depth_image_topic_0);
        topics.push_back(depth_image_topic_1);
        topics.push_back(depth_image_topic_2);
        topics.push_back(color_image_topic_0);
        topics.push_back(color_image_topic_1);
        topics.push_back(color_image_topic_2);

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        // Set up fake subscribers to capture images & point cloud
        BagSubscriber<sensor_msgs::PointCloud2> point_cloud_sub_0, point_cloud_sub_1, point_cloud_sub_2;
        BagSubscriber<sensor_msgs::Image> depth_image_sub_0, depth_image_sub_1, depth_image_sub_2;
        BagSubscriber<sensor_msgs::Image> color_image_sub_0, color_image_sub_1, color_image_sub_2;

        // Use approximated sync data; Cause exact sync is hard
        approx_sync_.reset(new ApproxTimeSynchronizer9(
            ApproxSyncPolicy9(60 /*q size*/),
            point_cloud_sub_0, point_cloud_sub_1, point_cloud_sub_2,
            depth_image_sub_0, depth_image_sub_1, depth_image_sub_2,
            color_image_sub_0, color_image_sub_1, color_image_sub_2
        ));

        approx_sync_->registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6, _7, _8, _9));

        // Load all messages
        BOOST_FOREACH(rosbag::MessageInstance const m, view) {
            // Topic list
            ROS_INFO("Topic: %s", m.getTopic().c_str());
            // PointCloud
            if (m.getTopic() == point_cloud_topic_0 || ("/" + m.getTopic() == point_cloud_topic_0)) {
                sensor_msgs::PointCloud2::ConstPtr point_cloud_0 = m.instantiate<sensor_msgs::PointCloud2>();
                if (point_cloud_0 != NULL)
                    point_cloud_sub_0.newMessage(point_cloud_0);
            }
            if (m.getTopic() == point_cloud_topic_1 || ("/" + m.getTopic() == point_cloud_topic_1)) {
                sensor_msgs::PointCloud2::ConstPtr point_cloud_1 = m.instantiate<sensor_msgs::PointCloud2>();
                if (point_cloud_1 != NULL)
                    point_cloud_sub_1.newMessage(point_cloud_1);
            }
            if (m.getTopic() == point_cloud_topic_2 || ("/" + m.getTopic() == point_cloud_topic_2)) {
                sensor_msgs::PointCloud2::ConstPtr point_cloud_2 = m.instantiate<sensor_msgs::PointCloud2>();
                if (point_cloud_2 != NULL)
                    point_cloud_sub_2.newMessage(point_cloud_2);
            }
            // Depth Image
            if (m.getTopic() == depth_image_topic_0 || ("/" + m.getTopic() == depth_image_topic_0)) {
                sensor_msgs::Image::ConstPtr depth_image_0 = m.instantiate<sensor_msgs::Image>();
                if (depth_image_0 != NULL)
                    depth_image_sub_0.newMessage(depth_image_0);
            }
            if (m.getTopic() == depth_image_topic_1 || ("/" + m.getTopic() == depth_image_topic_1)) {
                sensor_msgs::Image::ConstPtr depth_image_1 = m.instantiate<sensor_msgs::Image>();
                if (depth_image_1 != NULL)
                    depth_image_sub_1.newMessage(depth_image_1);
            }
            if (m.getTopic() == depth_image_topic_2 || ("/" + m.getTopic() == depth_image_topic_2)) {
                sensor_msgs::Image::ConstPtr depth_image_2 = m.instantiate<sensor_msgs::Image>();
                if (depth_image_2 != NULL)
                    depth_image_sub_2.newMessage(depth_image_2);
            }
            // Color Image
            if (m.getTopic() == color_image_topic_0 || ("/" + m.getTopic() == color_image_topic_0)) {
                sensor_msgs::Image::ConstPtr color_image_0 = m.instantiate<sensor_msgs::Image>();
                if (color_image_0 != NULL)
                    color_image_sub_0.newMessage(color_image_0);
            }
            if (m.getTopic() == color_image_topic_1 || ("/" + m.getTopic() == color_image_topic_1)) {
                sensor_msgs::Image::ConstPtr color_image_1 = m.instantiate<sensor_msgs::Image>();
                if (color_image_1 != NULL)
                    color_image_sub_1.newMessage(color_image_1);
            }
            if (m.getTopic() == color_image_topic_2 || ("/" + m.getTopic() == color_image_topic_2)) {
                sensor_msgs::Image::ConstPtr color_image_2 = m.instantiate<sensor_msgs::Image>();
                if (color_image_2 != NULL)
                    color_image_sub_2.newMessage(color_image_2);
            }
        }
        bag.close();

        // Write pcd and io_config here
        ROS_INFO("Start saving / preparing.");
        boost::filesystem::path dump_path(dump_root_path);
        dump_path /= data_name;  // add data name
        boost::filesystem::remove_all(dump_path);  // Clean

        for (auto frame_idx = 0; frame_idx < multi_calibration_dataset_.size(); ++frame_idx) {
            if (frame_idx < frame_start) continue;
            if (frame_idx >= frame_end) break;
            const auto save_frame_idx = frame_idx - frame_start;
            ROS_INFO("Saving frame %d.", frame_idx + 1);
            
            for (auto cam_idx = 0; cam_idx < num_camera; ++cam_idx) {
                // Color Image
                auto color_img_name = Easy3DViewer::FileNameVolumeDeform(dump_path, cam_idx, save_frame_idx, Easy3DViewer::color_img_file);
                auto color_img_ptr = cv_bridge::toCvCopy(
                    multi_calibration_dataset_[frame_idx].m_color_images[cam_idx], "bgr8");
                // Check existence && create recursively
                if (!boost::filesystem::is_directory(color_img_name.parent_path())) {
                    boost::filesystem::create_directories(color_img_name.parent_path());
                }
                cv::imwrite(color_img_name.string(), color_img_ptr->image);
                ROS_INFO("Saving to %s", color_img_name.string().c_str());
                // Depth Image
                auto depth_img_name = Easy3DViewer::FileNameVolumeDeform(dump_path, cam_idx, save_frame_idx, Easy3DViewer::depth_img_file);
                auto depth_img_ptr = cv_bridge::toCvCopy(multi_calibration_dataset_[frame_idx].m_depth_images[cam_idx]);
                 // Check existence && create recursively
                if (!boost::filesystem::is_directory(depth_img_name.parent_path())) {
                    boost::filesystem::create_directories(depth_img_name.parent_path());
                }
                cv::imwrite(depth_img_name.string(), depth_img_ptr->image);
                ROS_INFO("Saving to %s", depth_img_name.string().c_str());
            }
        }
        
        /**
         * cam{%d}: {
         *  Intrinsic:
         *  Extrinsic:
         * },
         * num_frame,
         * width,
         * height
         */
        boost::filesystem::path output_config_path = dump_path / "config.json";
        json output_json;
        
        for (auto cam_idx = 0; cam_idx < num_camera; ++cam_idx) {
            std::string cam_name = "cam-" + std::to_string(cam_idx);
            // Extrinsic
            json exstrinsic_json;
            Easy3DViewer::WriteMatrix<double, 4, 4>(
                cam_to_worlds[cam_idx], exstrinsic_json
            );
            output_json[cam_name]["extrinsic"] = exstrinsic_json;

            // Intrinsic
            json intrinsic_json; 
            unsigned count_ = 0;
            for (auto v : io_config[cam_name]["intrinsics"]) {
                intrinsic_json[count_++] = v.as<double>();
            }
            output_json[cam_name]["intrinsic"] = intrinsic_json;
        }

        // Other info
        output_json["image_cols"] = io_config["image_cols"].as<unsigned>();
        output_json["image_rows"] = io_config["image_rows"].as<unsigned>();
        std::ofstream file(output_config_path.string());
        file << output_json;
        std::cout << output_json;
        file.close();
        // Verify loading
        ROS_INFO("The size of multi-camera dataset is %ld.", multi_calibration_dataset_.size());
        ros::spin();  // Clear ros context
        return;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_depth_calibration");
    ros::NodeHandle nh("multi_depth_calibration");
    std::string data_name, bag_path, io_config_path, apriltag_path, camera_pose_path, vis_path, dump_root_path;
    unsigned frame_start, frame_end;
    // Parse configs
    data_name = std::string(argv[1]);
    std::cout << "data_name: " << data_name << std::endl;
    frame_start = (unsigned)std::stoi(std::string(argv[2]));
    frame_end = (unsigned)std::stoi(std::string(argv[3]));
    std::cout << "frame: " << frame_start << ", " << frame_end << std::endl;
    bag_path = std::string(argv[4]);
    std::cout << "bag_path: " << bag_path << std::endl; 
    io_config_path = std::string(argv[5]);
    std::cout << "io_config_path: " << io_config_path << std::endl;
    camera_pose_path = std::string(argv[6]);
    std::cout << "camera_pose_path: " << camera_pose_path << std::endl;
    vis_path = std::string(argv[7]);
    std::cout << "vis_path: " << vis_path << std::endl;
    dump_root_path = std::string(argv[8]);
    std::cout << "dump_root_path: " << dump_root_path << std::endl;

    auto& context = Easy3DViewer::Context::Instance();
    ROS_INFO("Save path is : %s", vis_path.c_str());
    context.setDir(vis_path, "frame");
    context.clearDir();
    
    // Transfer
    multicam_calibration::TransferData<multicam_calibration::num_camera>(
        data_name, 
        bag_path, 
        io_config_path, 
        camera_pose_path, 
        dump_root_path,
        frame_start, 
        frame_end);
    ros::shutdown();

    return 0;
}