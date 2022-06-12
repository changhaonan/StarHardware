/* 
 * Created by Bernd Pfrommer, Kartik Mohta
 * Modified by Haonan Chang, 01/08/2022
 */
#include "multicam_apriltag_detector.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <boost/range/irange.hpp>

namespace multicam_calibration {
	namespace {
		template <typename T>
		inline T get_param(ros::NodeHandle const& n,
			std::string const& param_name) {
			T param_value;
			if (!n.getParam(param_name, param_value))
				throw std::logic_error(param_name + " param not set!");
			return param_value;
		}
	}

    template<unsigned num_camera>
	MultiCamApriltagDetector<num_camera>::MultiCamApriltagDetector(
		const std::string& cam_config_path,
		const std::string& tag_config_path) {
		
		// Get aprilgrid params
		ROS_INFO("Parse tag.");
		YAML::Node tag_config = YAML::LoadFile(tag_config_path);
		const std::string target_type = tag_config["target_type"].as<std::string>();
		target_tag_cols_ = tag_config["tagCols"].as<unsigned>();
		target_tag_rows_ = tag_config["tagRows"].as<unsigned>();
		target_tag_size_ = tag_config["tagSize"].as<float>();
		target_tag_spacing_ratio_ = tag_config["tagSpacing"].as<float>();
		const std::string tf = tag_config["tagFamily"].as<std::string>(); // tag family
		const std::string detector_type = tag_config["detector_type"].as<std::string>();
		const int tag_border = tag_config["tag_border"].as<int>();

		// Get camera info
		YAML::Node cam_config = YAML::LoadFile(cam_config_path);
		camera_intrinsics_.clear();
		ROS_INFO("Parse camera.");
		for (auto cam_idx = 0; cam_idx < num_camera; ++cam_idx) {
			char cam_name_char[20];
			sprintf(cam_name_char, "cam%d", cam_idx);
			std::string cam_name(cam_name_char);
			std::vector<double> intrinsic = {
				cam_config[cam_name]["intrinsics"][0].as<double>(),
				cam_config[cam_name]["intrinsics"][1].as<double>(),
				cam_config[cam_name]["intrinsics"][2].as<double>(),
				cam_config[cam_name]["intrinsics"][3].as<double>()
			};
			camera_intrinsics_.push_back(intrinsic);
		}
		ROS_INFO("Parse finished.");

		if (tf != "36h11" && tf != "25h9" && tf != "16h5") {
			ROS_ERROR_STREAM("invalid tag family: " << tf);
			throw std::invalid_argument("invalid tag family!");
		}
		apriltag_ros::TagFamily tagFamily = apriltag_ros::TagFamily::tf36h11;
		if (tf == "25h9") {
			tagFamily = apriltag_ros::TagFamily::tf25h9;
		}
		else if (tf == "16h5") {
			tagFamily = apriltag_ros::TagFamily::tf16h5;
		}

		// const std::string detector_type = nh.param<std::string>("detector_type", "Umich");  // Umich as default
		if (detector_type == "Mit") {
			detector_ = apriltag_ros::ApriltagDetector::Create(
				apriltag_ros::DetectorType::Mit, tagFamily);
			detector_->set_black_border(tag_border);
		}
		else if (detector_type == "Umich") {
			detector_ = apriltag_ros::ApriltagDetector::Create(
				apriltag_ros::DetectorType::Umich, tagFamily);
			detector_->set_black_border(1);
		}
		else {
			ROS_ERROR_STREAM("invalid detector type: " << detector_type);
			throw std::invalid_argument("invalid detector type!");
		}
	}

    template<unsigned num_camera>
	void MultiCamApriltagDetector<num_camera>::process(
            const MultiCalibrationData<num_camera>& multi_calibration_data,
            pcl::PointCloud<pcl::PointXYZ>::Ptr* tag_points_per_camera,
            std::vector<CalibrationPairs>& calibration_pair_vec) {
		
		unsigned int const num_tags = target_tag_rows_ * target_tag_cols_;
		// Tag initialization
		std::vector<Eigen::Vector3d> features_per_cam[num_camera];

		for (auto cam_idx = 0; cam_idx < num_camera; ++cam_idx) {
			// Tag
			tag_find_[cam_idx].clear();
			tag_find_[cam_idx].resize(num_tags);
			for (auto tag_id = 0; tag_id < num_tags; ++tag_id) {
				tag_find_[cam_idx][tag_id] = false;
			}

			// Feature
			features_per_cam[cam_idx].clear();
			features_per_cam[cam_idx].resize(4 * num_tags);  // One tag has four corners
		}

		for (unsigned int cam_idx = 0; cam_idx < num_camera; cam_idx++) {
			cv_bridge::CvImageConstPtr const color_img_ptr = cv_bridge::toCvCopy(
				multi_calibration_data.m_color_images[cam_idx], sensor_msgs::image_encodings::MONO8);
			cv::Mat const color_img = color_img_ptr->image;

			cv_bridge::CvImagePtr depth_img_ptr = cv_bridge::toCvCopy(
				multi_calibration_data.m_depth_images[cam_idx]);
			cv::Mat const depth_img = depth_img_ptr->image;

			// detect & refine
			auto img_apriltags = detector_->Detect(color_img);
			//apriltag_ros::RefineApriltags(color_img, img_apriltags);
			// Store
			for (auto const& tag : img_apriltags)
			{
				int const id = tag.id;
				if ((size_t)id >= num_tags) {
					ROS_WARN_STREAM("tag with invalid id found: "
						<< id << " (check your calibration target!)");
					continue;
				}

				// Update logging
				tag_find_[cam_idx][id] = true;

				// Add corner points
				for (int k = 0; k < 4; ++k)
				{
					const auto& ip = tag.corners[k];
					Point2<double> image_point(ip.x, ip.y);
					Eigen::Vector3d point_in_camera = RayCast(
       					image_point, depth_img, camera_intrinsics_[cam_idx]);
					
					pcl::PointXYZ p;
					p.x = (float) point_in_camera(0);
					p.y = (float) point_in_camera(1);
					p.z = (float) point_in_camera(2);

					tag_points_per_camera[cam_idx]->push_back(p);

					// Add to feature
					features_per_cam[cam_idx][4 * id + k] = point_in_camera;
				}
			}

			ROS_INFO("Cam: %u has %lu points.", cam_idx, tag_points_per_camera[cam_idx]->size());
		}
		
		// Collect feature pairs
		for (auto tag_id = 0; tag_id < num_tags; ++tag_id) {
			for (auto cam_idx = 0; cam_idx < num_camera; ++cam_idx) {
				bool tag_found_this = tag_find_[cam_idx][tag_id];
				if (tag_found_this) {
					// Search for its pair
					for (auto next_idx = cam_idx + 1;  next_idx < num_camera; ++next_idx) {
						bool tag_found_next = tag_find_[next_idx][tag_id];
						if (tag_found_next) {
							for (auto k = 0; k < 4; ++k) {
								std::pair<int, int> cam_pair(cam_idx, next_idx);
								std::pair<Eigen::Vector3d, Eigen::Vector3d> feature_pair(
									features_per_cam[cam_idx][4 * tag_id + k],
									features_per_cam[next_idx][4 * tag_id + k]
								);
								calibration_pair_vec.push_back(
									CalibrationPairs(
										cam_pair, feature_pair
									)
								);
							}
						}
					}
				}
			}
		}
	}


	Eigen::Vector3d RayCast(
        const Point2<double>& image_point, const cv::Mat& depth_map, const std::vector<double>& intrinsic, const double depth_scale) {
        double x = image_point.x;
        double y = image_point.y;
        double depth = (double) ReadDepthMap(depth_map, (float)x, (float)y, (float)depth_scale);

		// ROS_INFO("ix: %f, iy: %f, depth: %f.", x, y, depth);
        // Raycast
        double fx = intrinsic[0];
        double fy = intrinsic[1];
        double cx = intrinsic[2];
        double cy = intrinsic[3];
		double p_x = (x - cx) / fx * depth;
        double p_y = (y - cy) / fy * depth;
        double p_z = depth;  // depth or -depth??
        // ROS_INFO("px: %f, py: %f, pz: %f.", p_x, p_y, p_z);
        return Eigen::Vector3d(p_x, p_y, p_z);
    }


    float ReadDepthMap(const cv::Mat& depth_map, const float x, const float y, const float depth_scale) {
        ROS_ASSERT(depth_map.type() == CV_16UC1);
        ROS_ASSERT(x >= 0 || x < depth_map.cols);
        ROS_ASSERT(y >= 0 || y < depth_map.rows);

        // Get 4 cornes: (l, u), (l, l), (r, l), (r, u)
        float depths[4] = {0.f, 0.f, 0.f, 0.f};
        float weights[4] = {1e-6f, 1e-6f, 1e-6f, 1e-6f};

        int x_l = floor(x); int x_r = ceil(x);
        int y_u = floor(y); int y_l = ceil(y);

		// It is at(y, x)!!
        depths[0] = float(depth_map.at<unsigned short>(y_u, x_l)) / depth_scale;
        depths[1] = float(depth_map.at<unsigned short>(y_l, x_l)) / depth_scale;
        depths[2] = float(depth_map.at<unsigned short>(y_l, x_r)) / depth_scale;
        depths[3] = float(depth_map.at<unsigned short>(y_u, x_r)) / depth_scale;

        weights[0] = (float(x_l) - x) *  (float(x_l) - x) + (float(y_u) - y) *  (float(y_u) - y) + 1e-6f;
        weights[1] = (float(x_l) - x) *  (float(x_l) - x) + (float(y_l) - y) *  (float(y_l) - y) + 1e-6f;
        weights[2] = (float(x_r) - x) *  (float(x_r) - x) + (float(y_l) - y) *  (float(y_l) - y) + 1e-6f;
        weights[3] = (float(x_r) - x) *  (float(x_r) - x) + (float(y_u) - y) *  (float(y_u) - y) + 1e-6f; 

		//Zero if depth is zero.
		for (auto i = 0; i < 4; ++i) {
			weights[i] = (abs(depths[i]) < 1e-6f)? 1e-6f: weights[i];
		}

        float weigth_sum = weights[0] + weights[1] + weights[2] + weights[3];
        weights[0] /= weigth_sum;
        weights[1] /= weigth_sum;
        weights[2] /= weigth_sum;
        weights[3] /= weigth_sum;

        return depths[0] * weights[0] + depths[1] * weights[1] + depths[2] * weights[2] + depths[3] * weights[3];
    }
	
	// Instance
	template class MultiCamApriltagDetector<3>;
}