/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */

#pragma once
#include <ostream>
#include <vector>
#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

namespace multicam_calibration {

	template <typename T>
	using vector_aligned = std::vector<T, Eigen::aligned_allocator<T>>;

	template <typename T>
	struct Point2 {
		T x, y;
		Point2() : x(0), y(0) {}
		Point2(const T _x, const T _y) : x(_x), y(_y) {}

		// Requires ths since it can be called with T = ceres::Jet which contains an
		// Eigen::Matrix
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			friend std::ostream& operator<<(std::ostream& stream,
				const Point2<T>& pt) {
			stream << "(" << pt.x << ", " << pt.y << ")";
			return stream;
		}
	};

	template <typename T>
	struct Point3 {
		T x, y, z;
		Point3() : x(0), y(0), z(0) {}
		Point3(const T _x, const T _y, const T _z) : x(_x), y(_y), z(_z) {}

		// Requires ths since it can be called with T = ceres::Jet which contains an
		// Eigen::Matrix
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			friend std::ostream& operator<<(std::ostream& stream,
				const Point3<T>& pt) {
			stream << "(" << pt.x << ", " << pt.y << ", " << pt.z << ")";
			return stream;
		}
	};

	using FrameWorldPoints = std::vector<Point3<double>>;
	using FrameImagePoints = std::vector<Point2<double>>;
	using FrameImageDepth = double;

	using CamWorldPoints = std::vector<FrameWorldPoints>;
	using CamImagePoints = std::vector<FrameImagePoints>;
	using CamImageDepths = std::vector<double>;

	using Pose = std::pair<Point3<double>, Point3<double>>;

	// Correspondence: (cam_idx_0, cam_idx_1), (feature_vector_in_0, feature_vector_in_1)
	using CalibrationPairs 
		= std::pair<std::pair<int, int>, std::pair<Eigen::Vector3d, Eigen::Vector3d>>;

	/*
	* \brief: 3D point of tag corners can be gotten
	*/
	template<unsigned num_camera>
	class MultiCalibrationData {
	public:
		sensor_msgs::Image::ConstPtr m_depth_images[num_camera];
		sensor_msgs::Image::ConstPtr m_color_images[num_camera];

		void set_camera_data(
			const unsigned cam_idx,
			const sensor_msgs::Image::ConstPtr depth_image,
			const sensor_msgs::Image::ConstPtr color_image) {
			m_depth_images[cam_idx] = depth_image;
			m_color_images[cam_idx] = color_image;
		}
	};

	template<unsigned num_camera>
	class MultiPointCloudData{
	public:
		sensor_msgs::PointCloud2::ConstPtr m_point_clouds[num_camera];

		void set_camera_data(
			const unsigned cam_idx,
			const sensor_msgs::PointCloud2::ConstPtr point_cloud) {
			m_point_clouds[cam_idx] = point_cloud;
		}
	};
}
