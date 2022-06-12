#include "camera.h"

namespace multicam_calibration {

    // Intrinsic related
    std::ostream& operator<<(std::ostream& os, const CameraIntrinsics& ci) {
		os << "camera mod: " << ci.camera_model << std::endl;
		os << "intrinsics: " << ci.intrinsics[0] << " " << ci.intrinsics[1]
			<< " " << ci.intrinsics[2] << " " << ci.intrinsics[3] << std::endl;
		os << "distortion model: " << ci.distortion_model << std::endl;
		os << "distortion coeffs: ";
		for (const auto& d : ci.distortion_coeffs) {
			os << " " << d;
		}
		os << std::endl;
		os << "resolution:" << ci.resolution[0] << " x " << ci.resolution[1];
		return (os);
	}


    // Extrinsic related
    CameraExtrinsics zeros() {
		return (CameraExtrinsics::Zero());
	}

	CameraExtrinsics identity() {
		return (CameraExtrinsics::Identity());
	}

	bool isNonZero(const CameraExtrinsics& T) {
		return ((T(0, 0) + T(0, 1) + T(0, 2) + T(0, 3) +
			T(1, 0) + T(1, 1) + T(1, 2) + T(1, 3) +
			T(2, 0) + T(2, 1) + T(2, 2) + T(2, 3) +
			T(3, 0) + T(3, 1) + T(3, 2) + T(3, 3)) != 0);
	}


    // template<typename T>
    // Point2<T> Project(const Point3<T>& world_point, const CameraIntrinsics& cam_it) {
    //     ROS_ERROR("To be implemented.");
    // }
}