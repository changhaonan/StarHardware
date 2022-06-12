#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "types.h"

namespace multicam_calibration {
    
    // Intrinsic
	struct CameraIntrinsics {
		CameraIntrinsics() {
			intrinsics.resize(4);
			distortion_coeffs.resize(4);
			resolution.resize(2);
		}
		std::vector<double> distortion_coeffs;
		std::vector<double> intrinsics; // K Matrix
		std::vector<int>    resolution;
		std::string distortion_model;
		std::string camera_model;
	};
	std::ostream& operator<<(std::ostream& os, const CameraIntrinsics& ci);

    
    // Extrinsic
    using  CameraExtrinsics = Eigen::Matrix<double, 4, 4>;
	CameraExtrinsics zeros();
	CameraExtrinsics identity();
	bool isNonZero(const CameraExtrinsics& T);
	typedef std::vector<CameraExtrinsics, Eigen::aligned_allocator<CameraExtrinsics> > CameraExtrinsicsVec;


    // Camera raycast & project
    /*
     * \brief: From world points to image points
     */
    // template<typename T>
    // Point2<T> Project(const Point3<T>& world_point, const CameraIntrinsics& cam_it);    
}