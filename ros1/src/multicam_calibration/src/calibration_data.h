/* Created by Haonan Chang, 01/07/2022
 */
#pragma once

#include <Eigen/StdVector>
#include <vector>
#include <string>
#include <ostream>
#include <ros/ros.h>
#include "types.h"
#include "camera.h"


namespace multicam_calibration {
    /*
    * \brief: Only extrinsic will be calibrated
    * * Intrinsic is fixed in this version
    */
	struct CalibrationData {
		typedef std::vector<CalibrationData, Eigen::aligned_allocator<CalibrationData> > CalibDataVec;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		std::string       name;
		CameraIntrinsics  intrinsics;
		CameraExtrinsics  T_cam_imu;
		CameraExtrinsics  T_cn_cnm1;
		std::string       rostopic;
		int               tagCount{ 0 };
		int               rotateCode{ -1 };
		int               flipCode{ 2 };
		bool              active{ true };
		// 
		static CalibDataVec parse_cameras(const ros::NodeHandle& nh);
	};
	using CalibDataVec = CalibrationData::CalibDataVec;
	std::ostream& operator<<(std::ostream& os, const CalibrationData& cd);
}
