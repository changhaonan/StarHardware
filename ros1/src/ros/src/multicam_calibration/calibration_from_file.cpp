#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
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
#include <pcl/common/transforms.h>
#include "context.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "types.h"
#include "calibrator.h"
#include "calibration_utils.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace multicam_calibration {
    // Global setting
	constexpr unsigned d_num_camera = 3;
    constexpr unsigned d_num_feature_per_frame = 8;

    // Generate pairs from file
    // In file:
    // i: (j, x, y, z), i: feature-id; j:cam-id 
    // point_clouds puts the feature for each camera
    template<unsigned num_camera, unsigned num_feature_frame>
    void ParseFeaturePairs(
        const json& feature_json_frame,
        std::vector<CalibrationPairs>& calibration_pair_vec,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr* point_clouds
    ) {
        const auto feature_list 
                = feature_json_frame.get<std::vector<std::vector<std::vector<double>>>>();
        for (auto feature_idx = 0; feature_idx < num_feature_frame; ++feature_idx) {
            const auto feature_poses = feature_list[feature_idx];
            for (auto i = 0; i < num_camera; ++i) {
                const std::vector<double> pose_i = feature_poses[i];
                if (fabs(pose_i[2]) < 1e-8) continue;  // Dropped feature
                Eigen::Vector3d pose_i_eigen(pose_i.data());

                for (auto j = i + 1; j < num_camera; ++j) {
                    const std::vector<double> pose_j = feature_poses[j];
                    if (fabs(pose_j[2]) < 1e-8) continue;  // Dropped feature
                    Eigen::Vector3d pose_j_eigen(pose_j.data());

                    // Create pair
                    CalibrationPairs calibration_pair(
                        std::pair<int, int>((int)i, (int)j), 
                        std::pair<Eigen::Vector3d, Eigen::Vector3d>(
                            pose_i_eigen,
                            pose_j_eigen
                        )
                    );
                    calibration_pair_vec.push_back(
                        calibration_pair
                    );
                }

                // Save feature for visualization
                pcl::PointXYZ p;
                p.x = pose_i[0];
                p.y = pose_i[1];
                p.z = pose_i[2];
                point_clouds[i]->points.push_back(p);
            }
        }

        // Resize point_clouds
        for (auto i = 0; i < d_num_camera; ++i)
            point_clouds[i]->resize(point_clouds[i]->points.size());
    }
}


int main(int argc, char** argv) {
    using namespace multicam_calibration;
    std::string feature_pose_path, io_config_path, apriltag_path, camera_pose_path;
    unsigned frame_transfer;
    // Parse configs
    if (argc > 1) {
        feature_pose_path = std::string(argv[1]);
    }
    else {
        feature_pose_path = "data/feature_poses.json";
    }
    // Camera configuration
    if (argc > 2) {
        io_config_path = std::string(argv[2]);
    }
    else {
        io_config_path = "data/io.yaml";
    }
    if (argc > 3) {
        camera_pose_path = std::string(argv[3]);
    }
    else {
        camera_pose_path = "data/camera_pose.yaml";
    }
    
	auto &context = WebViewer3D::Context::Instance();
	auto file_dir = boost::filesystem::path(std::string(__FILE__));
	std::string save_path = file_dir.parent_path().string() + "/../../../public/test_data/Calibration/";
	ROS_INFO("Save path is : %s", save_path.c_str());
	context.setDir(save_path, "frame");
	context.clearDir();

    // Initialization
    // Pose
    YAML::Node io_config = YAML::LoadFile(io_config_path);
    YAML::Node cam_pose_config = YAML::LoadFile(camera_pose_path);
    Eigen::Matrix4d cam_to_worlds[d_num_camera];
    Eigen::Matrix4f cam_to_worlds_f[d_num_camera];
    for (auto i = 0; i < d_num_camera; ++i) {
        std::string cam_name = "cam" + std::to_string(i);
        Eigen::Matrix4d cam_to_world;
        cam_to_world = ParseMatrix4d(cam_pose_config[cam_name].as<YAML::Node>());
        cam_to_worlds[i] = cam_to_world;

        Eigen::Matrix4f cam_to_world_f; 
        cam_to_world_f = cam_to_world.cast<float>();
        cam_to_worlds_f[i] = cam_to_world_f;
    }

    printf("Calibrator Initialization.\n");
    Calibrator<d_num_camera> calibrator(cam_to_worlds);

    // Read json file
    json feature_pose_json;
    std::ifstream input(feature_pose_path);
    input >> feature_pose_json;
    
    unsigned num_frame_labeled = 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_clouds[d_num_camera];
    for (auto i = 0; i < d_num_camera; ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(
            new pcl::PointCloud<pcl::PointXYZ>()
        );
        point_clouds[i] = pcd;
    }

    for (auto frame_idx = 0; frame_idx < num_frame_labeled; ++frame_idx) {
        printf("Begin parsing frame %d.\n", frame_idx);
        std::vector<CalibrationPairs> calibration_pair_vec;
        ParseFeaturePairs<d_num_camera, d_num_feature_per_frame>(
            feature_pose_json[frame_idx],
            calibration_pair_vec,
            point_clouds
        ); 
        calibrator.addCalibrationPair(calibration_pair_vec);
    }

    calibrator.runCalibration();
    // Save to first for test
    calibrator.getResults();
    calibrator.writeResults(camera_pose_path);
    
    // Visualization Features
    for (auto i = 0; i < d_num_camera; ++i) {
        pcl::transformPointCloud(*(point_clouds[i]), *(point_clouds[i]), calibrator.getCamPose(i));
        std::string save_pcd_name = "data/feature_" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileASCII(save_pcd_name, *(point_clouds[i]));
    }
    
    printf("Calibration is finished.\n");
    return 0;
}