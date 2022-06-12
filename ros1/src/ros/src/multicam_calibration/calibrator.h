/*
 * \brief: In calibration, we want to align 3D grids
 */
#include "context.hpp"
#include <opencv2/core/eigen.hpp>
#include "types.h"
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

namespace multicam_calibration {
    
    /* 
     * \brief: Calibration over 3d poses error.
     *  Params: [rx, ry, rz, tx, ty, tz]_i,
     *  Residuals: 3 * num_feature_pairs
     */
    template<unsigned num_camera>
    class Calibrator {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        Calibrator(const Eigen::Matrix4d* poses);
        void addCalibrationPair(const std::vector<CalibrationPairs>& feature_pairs);
        void initializeVariables(std::vector<double>* param_ptr);
		void setupOptimizationProblem(ceres::Problem* prob, std::vector<double>* vars);
        void runCalibration();

    public:
        const Eigen::Matrix4d* getCamPose() const { return cam_poses_.data(); }
        const Eigen::Matrix4d getCamPose(unsigned cam_idx) const {
            return cam_poses_[cam_idx];
        }
        void writeResults(const std::string& output_name);
        void getResults();
    private:
        std::vector<Eigen::Matrix4d>  cam_poses_; // Absolute cam poses w.r.t. world
    	std::vector<double>           params_; // All the parameters that need to be optimized

    private:
        ceres::ResidualBlockId residual_id_;

    private:
        // Debug information
        // void showCameraStatus() const;
        std::vector<CalibrationPairs> paired_features_;

    public: 
        // Debug method
        void debugFeature(double** param);
        void removeOutlier(const double threshold = 1.0);

    private: 
        std::vector<bool> outlier_mask_;
        unsigned outlier_count_;
    };

}