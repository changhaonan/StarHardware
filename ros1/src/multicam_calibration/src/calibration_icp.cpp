#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "context.hpp"
#include "calibration_utils.h"

namespace multicam_calibration {
    // Global setting
	constexpr unsigned d_num_camera = 3;

    void CalibrationICP(
        Eigen::Matrix4f* cam_to_worlds,
        pcl::PointCloud<pcl::PointNormal>::Ptr* point_clouds) {
        // Apply transform
        for (auto i = 0; i < d_num_camera; ++i) {
            pcl::transformPointCloud(*point_clouds[i], *point_clouds[i], cam_to_worlds[i]);
        } 
        
        for (auto i = 0; i < d_num_camera; ++i) {
            for (auto j = i + 1; j < d_num_camera; ++j) {
                pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
                // Set the input source and target
                icp.setInputTarget(point_clouds[i]);
                icp.setInputSource(point_clouds[j]);

                // Set parameterization
                icp.setMaxCorrespondenceDistance(0.05);
                icp.setMaximumIterations(50);
                icp.setTransformationEpsilon(1e-8);
                icp.setEuclideanFitnessEpsilon(1);
                icp.align(*point_clouds[j]);  // Move j to align i

                // Get transform & Update
                Eigen::Matrix4f transformation = icp.getFinalTransformation();
                std::cout << "Relative transform is:" << std::endl;
                std::cout << transformation << std::endl;
                cam_to_worlds[j] = transformation * cam_to_worlds[j];
            }
        }
    }

}

int main(int argc, char** argv) {
    using namespace multicam_calibration;
    std::string depth_img_path, io_config_path, apriltag_path, camera_pose_path;
    unsigned frame_transfer;
    // Parse configs
    if (argc > 1) {
        depth_img_path = std::string(argv[1]);
    }
    else {
        depth_img_path = "data/test_data/calibration/";
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
    
	auto &context = Easy3DViewer::Context::Instance();
	auto file_dir = boost::filesystem::path(std::string(__FILE__));
	std::string save_path = file_dir.parent_path().string() + "/../../../public/test_data/Calibration/";

	context.setDir(save_path, "frame");
	context.clearDir();

    // Initialization
    // Extrinisics & Initrinsics
    YAML::Node io_config = YAML::LoadFile(io_config_path);
    YAML::Node cam_pose_config = YAML::LoadFile(camera_pose_path);
    Eigen::Matrix4f cam_to_worlds[d_num_camera];
    std::vector<std::vector<float>> intrinsics;
    for (auto i = 0; i < d_num_camera; ++i) {
        std::string _cam_name = "cam" + std::to_string(i);
        Eigen::Matrix4f cam_to_world = ParseMatrix4f(cam_pose_config[_cam_name].as<YAML::Node>());
        cam_to_worlds[i] = cam_to_world;

        std::vector<float> intrinsic = {
            io_config[_cam_name]["intrinsics"][0].as<float>(),
            io_config[_cam_name]["intrinsics"][1].as<float>(),
            io_config[_cam_name]["intrinsics"][2].as<float>(),
            io_config[_cam_name]["intrinsics"][3].as<float>()
        };
        intrinsics.push_back(intrinsic);
    }

    printf("Calibrator Initialized\n");
    // Create point cloud
    const float clip_near = 0.1;
    const float clip_far = 1.0;
    const std::string img_dir_path_root = "Easy3DViewer/ros/data/test_data/test";
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_clouds[d_num_camera];
    pcl::PointCloud<pcl::PointNormal>::Ptr point_clouds_exp[d_num_camera];
    // Pointcloud initialization
    for (auto i = 0; i < d_num_camera; ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(
            new pcl::PointCloud<pcl::PointXYZ>()
        );
        point_clouds[i] = pcd;

        pcl::PointCloud<pcl::PointNormal>::Ptr pcd_normal(
            new pcl::PointCloud<pcl::PointNormal>()
        );
        point_clouds_exp[i] = pcd_normal;
    }

    for (auto i = 0; i < d_num_camera; ++i) {
        // Zero-frame alignment
        std::ostringstream number_str;
        number_str << std::setw(2) << std::setfill('0') << std::to_string(i);
        std::string img_depth_name = depth_img_path + "cam-" + number_str.str() + "/frame-000000.depth.png";
        cv::Mat img_depth = cv::imread(img_depth_name, cv::IMREAD_UNCHANGED);
        // Generate point cloud
        Img2Pcd(
            intrinsics[i],
            img_depth,
            point_clouds[i],
            clip_near,
            clip_far);

        // Generate point cloud with normal
        PointXYZ2PointNormal(point_clouds[i], point_clouds_exp[i]);
    }
    
    // Save point cloud for visualization before calibration
    for (auto i = 0; i < d_num_camera; ++i) {
        std::string save_pcd_name = "data/before_pcd_" + std::to_string(i) + ".pcd";
        // Transform
        pcl::transformPointCloud(*(point_clouds[i]), *(point_clouds[i]), cam_to_worlds[i]);
        pcl::io::savePCDFileASCII(save_pcd_name, *(point_clouds[i]));
    }   

    // Calibration based on point cloud
    CalibrationICP(cam_to_worlds, point_clouds_exp); 

    // Save output
    printf("Calibration finished.\n");
    WriteCamPoseAsYaml<d_num_camera>(
        camera_pose_path,
        cam_to_worlds
    );

    // Save point cloud for visualization after calibration
    for (auto i = 0; i < d_num_camera; ++i) {
        std::string save_pcd_name = "data/after_pcd_" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileASCII(save_pcd_name, *(point_clouds_exp[i]));
    }

    return 0;
}