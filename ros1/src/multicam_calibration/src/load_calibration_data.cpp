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
#include "context.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "types.h"
#include "multicam_apriltag_detector.hpp"
#include "calibrator.h"

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
		void newMessage(const boost::shared_ptr<M const> &msg) {
			this->signalMessage(msg);
		}
	};


	// Callback for synchronized messages for 3 cameras
	void callback(
		const sensor_msgs::PointCloud2::ConstPtr &point_cloud_0,
		const sensor_msgs::PointCloud2::ConstPtr &point_cloud_1,
		const sensor_msgs::PointCloud2::ConstPtr &point_cloud_2,
		const sensor_msgs::Image::ConstPtr &depth_image_0,
		const sensor_msgs::Image::ConstPtr &depth_image_1,
		const sensor_msgs::Image::ConstPtr &depth_image_2,
		const sensor_msgs::Image::ConstPtr &color_image_0,
		const sensor_msgs::Image::ConstPtr &color_image_1,
		const sensor_msgs::Image::ConstPtr &color_image_2) {
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
	std::shared_ptr<pcl::PointCloud<PointT>> Ros2Pcl(const sensor_msgs::PointCloud2::ConstPtr &input) {
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
	void LoadData(
		MultiCamApriltagDetector<num_camera>& detector,
		const std::string& filename, 
		const std::string& io_config_path, 
		const std::string& camera_pose_path,
		unsigned frame_transfer) {
		// Initialization
		multi_calibration_dataset_.clear();
		multi_point_cloud_dataset_.clear();
		
		// Pose
		YAML::Node io_config = YAML::LoadFile(io_config_path);
		YAML::Node cam_pose_config = YAML::LoadFile(camera_pose_path);
		Eigen::Matrix4d cam_to_worlds[num_camera];
        Eigen::Matrix4f cam_to_worlds_f[num_camera];
        for (auto i = 0; i < num_camera; ++i) {
            std::string cam_name = "cam" + std::to_string(i);
			Eigen::Matrix4d cam_to_world;
            cam_to_world = ParseMatrix4d(cam_pose_config[cam_name].as<YAML::Node>());
			cam_to_worlds[i] = cam_to_world;

			Eigen::Matrix4f cam_to_world_f; 
			cam_to_world_f = cam_to_world.cast<float>();
            cam_to_worlds_f[i] = cam_to_world_f;
        }

		rosbag::Bag bag;
		bag.open(filename, rosbag::bagmode::Read);

		// Topics: PointCloud
		std::string point_cloud_topic_0 = io_config["cam0"]["point_cloud_topic"].as<std::string>();
		std::string point_cloud_topic_1 = io_config["cam1"]["point_cloud_topic"].as<std::string>();
		std::string point_cloud_topic_2 = io_config["cam2"]["point_cloud_topic"].as<std::string>();
		// Topics: Color & Depth
		std::string depth_image_topic_0 = io_config["cam0"]["depth_topic"].as<std::string>();
		std::string depth_image_topic_1 = io_config["cam1"]["depth_topic"].as<std::string>();
		std::string depth_image_topic_2 = io_config["cam2"]["depth_topic"].as<std::string>();
		std::string color_image_topic_0 = io_config["cam0"]["color_topic"].as<std::string>();
		std::string color_image_topic_1 = io_config["cam1"]["color_topic"].as<std::string>();
		std::string color_image_topic_2 = io_config["cam2"]["color_topic"].as<std::string>();

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
		BOOST_FOREACH (rosbag::MessageInstance const m, view) {
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
		Calibrator<num_camera> calibrator(cam_to_worlds);

		auto &context = Easy3DViewer::Context::Instance();
		for (auto frame_idx = 0; frame_idx < multi_calibration_dataset_.size(); ++frame_idx) {
			if (frame_idx >= frame_transfer) break;
			
			ROS_INFO("Saving frame %d.", frame_idx + 1);
			context.open(frame_idx + 1);  // Saving from frame 1
			// PointCloud
			for (auto cam_idx = 0; cam_idx < num_camera; ++cam_idx) {
				char pcd_name_str[20];
				sprintf(pcd_name_str, "pcd_%d", cam_idx);
				std::string pcd_name(pcd_name_str);
				context.addPointCloud(pcd_name, pcd_name, cam_to_worlds_f[cam_idx], 0.2f);
				std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud 
					= Ros2Pcl<pcl::PointXYZRGB>(multi_point_cloud_dataset_[frame_idx].m_point_clouds[cam_idx]);
				pcl::io::savePCDFileASCII(context.at(pcd_name), *point_cloud);
			}

			// Tag detection
			pcl::PointCloud<pcl::PointXYZ>::Ptr detected_tags_per_cam[num_camera];
			for (auto cam_idx = 0; cam_idx < num_camera; ++cam_idx) {
				detected_tags_per_cam[cam_idx] = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
			}
			std::vector<CalibrationPairs> calibration_pair_vec;
			detector.process(
				multi_calibration_dataset_[frame_idx],
				detected_tags_per_cam,
				calibration_pair_vec);

			// Add into calibration
			calibrator.addCalibrationPair(calibration_pair_vec);

			// Save PointCloud
			for (auto cam_idx = 0; cam_idx < num_camera; ++cam_idx) {
				char tag_name_str[20];
				sprintf(tag_name_str, "tag_%d", cam_idx);
				std::string tag_name(tag_name_str);
				context.addPointCloud(tag_name, tag_name, cam_to_worlds_f[cam_idx], 0.2f);
				pcl::io::savePCDFileASCII(context.at(tag_name), *detected_tags_per_cam[cam_idx]);
			}	
			context.close();
		}

		// Calibration
		calibrator.runCalibration();
		// Save to first for test
		calibrator.getResults();
		calibrator.writeResults(camera_pose_path);

		// Verify loading
		ROS_INFO("The size of multi-camera dataset is %ld.", multi_calibration_dataset_.size());
		ros::spin();  // Clear ros context
		return;
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "multi_depth_calibration");
	ros::NodeHandle nh("multi_depth_calibration");
    std::string bag_path, io_config_path, apriltag_path, camera_pose_path;
    unsigned frame_transfer;
    // Parse configs
    if (argc > 1) {
        bag_path = std::string(argv[1]);
    }
    else {
        bag_path = "data/calibration.bag";
    }
    // Camera configuration
    if (argc > 2) {
        io_config_path = std::string(argv[2]);
    }
    else {
        io_config_path = "data/io.yaml";
    }
    if (argc > 3) {
        frame_transfer = (unsigned)std::stoi(std::string(argv[3]));
    }
    else {
        frame_transfer = 50;  // For caliration, we don't want to too much frame
    }
    if (argc > 4) {
        apriltag_path = std::string(argv[4]);
    }
    else {
        apriltag_path = "data/aprilgrid.yaml";
    }
    if (argc > 5) {
        camera_pose_path = std::string(argv[5]);
    }
    else {
        camera_pose_path = "data/camera_pose.yaml";
    }

	auto &context = Easy3DViewer::Context::Instance();
	auto file_dir = boost::filesystem::path(std::string(__FILE__));
	std::string save_path = file_dir.parent_path().string() + "/../../../public/test_data/Calibration/";
	ROS_INFO("Save path is : %s", save_path.c_str());
	context.setDir(save_path, "frame");
	context.clearDir();
	
	// Detector
	multicam_calibration::MultiCamApriltagDetector<multicam_calibration::num_camera> detector(
		io_config_path, apriltag_path);

	multicam_calibration::LoadData<multicam_calibration::num_camera>(
		detector, bag_path, io_config_path, camera_pose_path, frame_transfer);
	ros::shutdown();
	
	return 0;
}