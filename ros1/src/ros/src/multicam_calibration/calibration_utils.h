#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <opencv2/core.hpp>
#include <boost/foreach.hpp>
#include <boost/range.hpp>
#include <boost/filesystem.hpp>

namespace multicam_calibration {
    Eigen::Matrix4d ParseMatrix4d(const YAML::Node& matrix_node) {
		Eigen::Matrix4d output_matrix;
		output_matrix << 
			matrix_node[0][0].as<double>(), matrix_node[0][1].as<double>(), matrix_node[0][2].as<double>(), matrix_node[0][3].as<double>(),
			matrix_node[1][0].as<double>(), matrix_node[1][1].as<double>(), matrix_node[1][2].as<double>(), matrix_node[1][3].as<double>(),
			matrix_node[2][0].as<double>(), matrix_node[2][1].as<double>(), matrix_node[2][2].as<double>(), matrix_node[2][3].as<double>(),
			matrix_node[3][0].as<double>(), matrix_node[3][1].as<double>(), matrix_node[3][2].as<double>(), matrix_node[3][3].as<double>();
		return output_matrix;
	}

    Eigen::Matrix4f ParseMatrix4f(const YAML::Node& matrix_node) {
		Eigen::Matrix4f output_matrix;
		output_matrix << 
			matrix_node[0][0].as<float>(), matrix_node[0][1].as<float>(), matrix_node[0][2].as<float>(), matrix_node[0][3].as<float>(),
			matrix_node[1][0].as<float>(), matrix_node[1][1].as<float>(), matrix_node[1][2].as<float>(), matrix_node[1][3].as<float>(),
			matrix_node[2][0].as<float>(), matrix_node[2][1].as<float>(), matrix_node[2][2].as<float>(), matrix_node[2][3].as<float>(),
			matrix_node[3][0].as<float>(), matrix_node[3][1].as<float>(), matrix_node[3][2].as<float>(), matrix_node[3][3].as<float>();
		return output_matrix;
	}

    // Pointcloud transformation
    /**
     * @brief Transform image into pointcloud, assuming no 
     * distortion model.
     * 
     * @param intrinsic 
     * @param depth_img Depth image is in (mm)
     * @param point_cloud Point cloud is in (m)
     */
    void Img2Pcd(
        const std::vector<float>& intrinsic,
        const cv::Mat& depth_img,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud,
        const float clip_near,
        const float clip_far,
        const bool z_axis_flip = false 
    ) {
        const auto width = depth_img.size().width;
        const auto height = depth_img.size().height;
        const auto fx = intrinsic[0];
        const auto fy = intrinsic[1];
        const auto cx = intrinsic[2];
        const auto cy = intrinsic[3];

        for (auto i = 0; i < width; ++i) {
            for (auto j = 0; j < height; ++j) {
                unsigned short depth_unsinged = depth_img.at<unsigned short>(j, i);
                float depth = float(depth_unsinged) / 1000.f;
                if (fabs(depth) < clip_near || fabs(depth) >= clip_far) continue;
                pcl::PointXYZ p;
                p.x = ((float) i - cx) / fx * depth;
                p.y = ((float) j - cy) / fy * depth;
                p.z = (z_axis_flip)? -depth : depth;
                point_cloud->points.push_back(p);
            }
        }

        point_cloud->resize(
            point_cloud->points.size()
        );

        printf("Point cloud size is %d.\n", point_cloud->points.size());
    }

    // Generate pointXYZNormal from PointXYZ cloud
    void PointXYZ2PointNormal(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud,
        const pcl::PointCloud<pcl::PointNormal>::Ptr& point_cloud_exp // Expanded with normal
    ) {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(point_cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        ne.compute(*cloud_normals);

        // Combine it to PointNormal
        pcl::concatenateFields(*point_cloud, *cloud_normals, *point_cloud_exp);
    }

    template<unsigned num_camera>
    void WriteCamPoseAsYaml(
        const std::string& output_name,
        const Eigen::Matrix4f* cam2worlds
    ) {
        YAML::Node config;
		for (auto i = 0; i < num_camera; ++i) {
			YAML::Node matrix;
			Eigen::Matrix4f cam_pose = cam2worlds[i];
			for (auto j = 0; j < 4; ++j) {
				YAML::Node row;
				for (auto k = 0; k < 4; ++k) {
					row[k] = cam_pose(j, k);
				}
				matrix[j] = row;
			}
			config["cam" + std::to_string(i)] = matrix;
		}

		std::ofstream ofs;
		ofs.open(output_name, std::ofstream::out | std::ofstream::trunc);
		ofs << config;
		ofs.close();
    }
}