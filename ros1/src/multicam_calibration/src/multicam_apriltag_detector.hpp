/* 
 * Created by Bernd Pfrommer, Kartik Mohta,
 * Modified by Haonan Chang, 01/08/2022
 */
#pragma once

#include <fstream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <apriltag_ros/apriltag_detector.h>
#include <yaml-cpp/yaml.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "camera.h"
#include "types.h"

namespace multicam_calibration {
	using sensor_msgs::ImageConstPtr;
    /*
     * \brief: Input is the RGB image, corner points in 3d space
     */
    template<unsigned num_camera>
	class MultiCamApriltagDetector {
	public:
		MultiCamApriltagDetector(
			const std::string& cam_config_path,
			const std::string& tag_config_path);

		void process(
            const MultiCalibrationData<num_camera>& multi_calibration_data,
            pcl::PointCloud<pcl::PointXYZ>::Ptr* tag_points_per_camera,
            std::vector<CalibrationPairs>& calibration_pair_vec 
			);

		//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		int target_tag_cols_, target_tag_rows_;
		float target_tag_size_, target_tag_spacing_ratio_;
		apriltag_ros::ApriltagDetector::Ptr detector_;
		std::ofstream outfile_;
		std::vector<bool> tag_find_[num_camera];  // If this tag is finded here

		// Camera: fixed to pinhole model & no distortion
		std::vector<std::vector<double>> camera_intrinsics_;
	};


	/*
     * \brief: From image points to world points
     */
    Eigen::Vector3d RayCast(
        const Point2<double>& image_point, const cv::Mat& depth_map, const std::vector<double>& intrinsic, const double depth_scale = 1000.0);

    /*
     * \brief: Here the depth_map should be 16UC1 and depth scale is default to be 1000.f 
     */
    float ReadDepthMap(const cv::Mat& depth_map, const float x, const float y, const float depth_scale = 1000.f);
}