/*
 * \brief: 
 */
#include "utils.h"
#include "calibrator.h"
#include <boost/range/irange.hpp>
#include <ros/ros.h>
#include <fstream>

namespace multicam_calibration {
	
	using utils::Mat;
	using utils::Vec;
	using utils::DynVec;
	using utils::rotation_matrix;
	using boost::irange;

    /*	
	 * \brief: Frame Residual will use the camera model to raycast image corners to world points. 
	 * Difference will be the 3d points difference.
	 */
    template<unsigned num_camera>
	struct FrameResidual 
    {
		// Input is the list of world points and image points for the current frame in
		// each of the cameras
		FrameResidual(
			const std::vector<CalibrationPairs>& paired_features,
			const std::vector<bool>& outlier_mask
		):
			paired_features_(paired_features),
			outlier_mask_(outlier_mask) {}

        /*
         * params: [
             [pose],
             [pose]
         ]
         */
		template <typename T>
		bool operator()(T const* const* params, T* residual) const
		{
            // Compute transformation of each camera
            Mat<T, 3, 3> R_cameras[num_camera];
            Vec<T, 3> t_cameras[num_camera];

            // R_cameras[0] = Mat<T, 3, 3>::Identity();
            // t_cameras[0] = Vec<T, 3>::Zero();

            for (auto cam_idx = 0; cam_idx < num_camera; ++cam_idx) {
                const Vec<T, 3> R_vec_cam = Eigen::Map<const Vec<T, 3>>(&params[cam_idx][0]);
                const Mat<T, 3, 3>  R_cam = rotation_matrix(R_vec_cam);
                const Vec<T, 3>     t_cam = Eigen::Map<const Vec<T, 3>>(&params[cam_idx][3]);

                // Relative transform is applyed to the right
                R_cameras[cam_idx] = R_cam;
                t_cameras[cam_idx] = t_cam;
            }
			
			unsigned int residual_count = 0;
            for (const auto i : irange<size_t>(0, paired_features_.size())) {
				if (outlier_mask_[i]) continue;  // Pass outlier

                const auto cam_id_pair = paired_features_[i].first;
                const auto feature_pair = paired_features_[i].second;

				Vec<T, 3> feature_x = {
					T{ feature_pair.first(0) },
					T{ feature_pair.first(1) },
					T{ feature_pair.first(2) }
				};
				Vec<T, 3> feature_y = {
					T{ feature_pair.second(0) },
					T{ feature_pair.second(1) },
					T{ feature_pair.second(2) }
				};

                const auto feature_in_cam_x = R_cameras[cam_id_pair.first] * feature_x + t_cameras[cam_id_pair.first];
                const auto feature_in_cam_y = R_cameras[cam_id_pair.second] * feature_y + t_cameras[cam_id_pair.second];

                for (auto j : irange<size_t>(0, 3)) {
                    residual[residual_count++] = feature_in_cam_x(j) - feature_in_cam_y(j);
                }
            }
            
			return true;
		}

	private:
		const std::vector<CalibrationPairs> paired_features_;
		const std::vector<bool> outlier_mask_;
	};

    /*
     * \brief: Tool function
     */
    static std::vector<double> transform_to_rvec_tvec(const Eigen::Matrix4d& tf) {
		std::vector<double> v;
		Eigen::AngleAxisd axisAngle(tf.block<3, 3>(0, 0));
		Eigen::Vector3d rvec = axisAngle.axis() * axisAngle.angle();
		v.push_back(rvec(0));
		v.push_back(rvec(1));
		v.push_back(rvec(2));
		// Translation
		Eigen::Vector3d T = tf.block<3, 1>(0, 3);
		v.push_back(T(0));
		v.push_back(T(1));
		v.push_back(T(2));
		return (v);
	}

    /*
     * \brief: 
     */
    static std::vector<double*>
		params_to_blocks(std::vector<double>& params, const unsigned num_camera) {
		std::vector<double*> v;
		unsigned int offset = 0;

        // Insert poses
		for (const auto cam_idx : boost::irange(0u, num_camera)) {
			v.push_back(&params[offset]); // Extrinisic
			offset += 6;
		}
		return (v);
	}

	template<unsigned num_camera>
	Calibrator<num_camera>::Calibrator(
		const Eigen::Matrix4d* poses
	) {
		cam_poses_.resize(num_camera);
		for (auto i : boost::irange(num_camera)) {
			cam_poses_[i] = poses[i];
		}
	}

	template<unsigned num_camera>
	void Calibrator<num_camera>::addCalibrationPair(const std::vector<CalibrationPairs>& feature_pairs) {
		const unsigned num_features = feature_pairs.size();
		for (auto i : boost::irange(0u, num_features)) {
			paired_features_.push_back(feature_pairs[i]);
		}
	}

    template<unsigned num_camera>
	void Calibrator<num_camera>::initializeVariables(std::vector<double>* param_ptr) {
		std::vector<double>& params = *param_ptr;
        
		// Initialize camera poses
		for (const auto cam_idx : boost::irange(0u, num_camera)) {
			std::vector<double> rvec_tvec = transform_to_rvec_tvec(cam_poses_[cam_idx]);
			params.insert(params.end(), rvec_tvec.begin(), rvec_tvec.end());
		}
	}

	template<unsigned num_camera>
	void Calibrator<num_camera>::getResults() {
		const std::vector<double>& p = params_;
		for (unsigned int cam_idx = 0; cam_idx < num_camera; cam_idx++) {
			Eigen::Matrix4d cam_pose = Eigen::Matrix4d::Identity();
			unsigned int off = 6 * cam_idx;
			const Vec<double, 3> rvec = Eigen::Map<const Vec<double, 3>>(&p[off]);
			Mat<double, 3, 3>       R = rotation_matrix(rvec);
			const Vec<double, 3>    t = Eigen::Map<const Vec<double, 3>>(&p[off + 3]);

			cam_pose.block<3, 3>(0, 0) = R;
			cam_pose.block<3, 1>(0, 3) = t;
			cam_pose(3, 3) = 1.0;

			cam_poses_[cam_idx] = cam_pose;
		}
	}

    template<unsigned num_camera>
	void Calibrator<num_camera>::runCalibration() {
		if (paired_features_.empty()) {
			ROS_ERROR("no data to run on!");
			return;
		}
        
		params_.clear();
		initializeVariables(&params_);
		ceres::Problem problem;
		setupOptimizationProblem(&problem, &params_);
		std::cout << "Num params: " << params_.size() << std::endl;
		std::cout << "Num residuals: " << problem.NumResiduals() << std::endl;
		// Evaluation
		ROS_INFO("Evaluation before calibration.");
		ROS_INFO("Has %d feature.", paired_features_.size());
		// std::vector<ceres::ResidualBlockId> to_eval;
        // to_eval.push_back(residual_id_);
        // ceres::Problem::EvaluateOptions eval_options;
        // eval_options.residual_blocks = to_eval;
        // double total_cost = 0.0;
        // std::vector<double> evaluated_residuals;
        // problem.Evaluate(eval_options, &total_cost, &evaluated_residuals, nullptr, nullptr);
        // for (auto i = 0; i < evaluated_residuals.size(); i++)
        //     std::cout << i << ": " << evaluated_residuals[i] << std::endl;

		ceres::Solver::Options options;
		options.minimizer_progress_to_stdout = true;
		options.max_num_iterations = 30;
		options.num_threads = 4;
		options.function_tolerance = 1e-12;
		options.parameter_tolerance = 1e-12;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		std::cout << summary.FullReport() << std::endl;
	}

    template<unsigned num_camera>
	void Calibrator<num_camera>::setupOptimizationProblem(ceres::Problem* prob, std::vector<double>* vars) {
		std::vector<double>& params = *vars;
		ceres::Problem& problem = *prob;

		// Remove outlier first
		removeOutlier();  // Default parameter here

        auto cost_function 
			= new ceres::DynamicAutoDiffCostFunction<FrameResidual<num_camera>>(
            new FrameResidual<num_camera>(paired_features_, outlier_mask_));

        std::vector<double*> v = params_to_blocks(params, num_camera);

        // Extrinsics blocks
        for (const auto iext : irange(0u, num_camera)) {
            (void)iext;
            cost_function->AddParameterBlock(6);
        }
        
        // 3d error
        cost_function->SetNumResiduals(3 * (paired_features_.size() - outlier_count_));
        residual_id_ = problem.AddResidualBlock(cost_function, nullptr, v);  // No kernel

        // Fix the first extrinsics: fix to identity
        problem.SetParameterBlockConstant(v[0]);

		// Debug
		// debugFeature(v.data());
	}

    template<unsigned num_camera>
	void Calibrator<num_camera>::writeResults(const std::string& output_name) {
		YAML::Node config;
		for (auto i : boost::irange(0u, num_camera)) {
			YAML::Node matrix;
			Eigen::Matrix4d cam_pose = getCamPose(i);
			for (auto j : boost::irange(0u, 4u)) {
				YAML::Node row;
				for (auto k : boost::irange(0u, 4u)) {
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

	template<unsigned num_camera>
	void Calibrator<num_camera>::debugFeature(double** param) {

		// Compute transformation of each camera
		Mat<double, 3, 3> R_cameras[num_camera];
		Vec<double, 3> t_cameras[num_camera];
		
		for (auto cam_idx = 0; cam_idx < num_camera; ++cam_idx) {
			const Vec<double, 3> R_vec_cam = Eigen::Map<const Vec<double, 3>>(&param[cam_idx][0]);
			const Mat<double, 3, 3>  R_cam = rotation_matrix(R_vec_cam);
			const Vec<double, 3>     t_cam = Eigen::Map<const Vec<double, 3>>(&param[cam_idx][3]);

			// Relative transform is applyed to the right
			R_cameras[cam_idx] = R_cam;
			t_cameras[cam_idx] = t_cam;
		}

		for (auto i : boost::irange<size_t>(0, paired_features_.size())) {
			auto cam_id_pair = paired_features_[i].first;
			auto feature_pair = paired_features_[i].second;
			auto id_x = cam_id_pair.first;
			auto id_y = cam_id_pair.second;
			Eigen::Vector3d feature_x = feature_pair.first;
			Eigen::Vector3d feature_y = feature_pair.second;
			// Check error
			// Rotate to world
			Eigen::Vector3d feature_x_1 = cam_poses_[id_x].block(0, 0, 3, 3) * feature_x + cam_poses_[id_x].block(0, 3, 3, 1);
			Eigen::Vector3d feature_y_1 = cam_poses_[id_y].block(0, 0, 3, 3) * feature_y + cam_poses_[id_y].block(0, 3, 3, 1);

			ROS_INFO("Id: (%d, %d).", cam_id_pair.first, cam_id_pair.second);
			ROS_INFO("error:(%f, %f, %f).", 
				feature_x_1(0) - feature_y_1(0), 
				feature_x_1(1) - feature_y_1(1),
				feature_x_1(2) - feature_y_1(2));
			ROS_INFO("p1: (%f, %f, %f). po: (%f, %f, %f)", feature_x_1(0), feature_x_1(1), feature_x_1(2), feature_x(0), feature_x(1), feature_x(2));
			ROS_INFO("p2: (%f, %f, %f). po: (%f, %f, %f)", feature_y_1(0), feature_y_1(1), feature_y_1(2), feature_y(0), feature_y(1), feature_y(2));
		}
	}

	template<unsigned num_camera>
	void Calibrator<num_camera>::removeOutlier(const double threshold) {

		outlier_count_ = 0;
		outlier_mask_.clear();
		outlier_mask_.resize(paired_features_.size());
		
		for (auto i : boost::irange<size_t>(0, paired_features_.size())) {
			auto cam_id_pair = paired_features_[i].first;
			auto feature_pair = paired_features_[i].second;
			auto id_x = cam_id_pair.first;
			auto id_y = cam_id_pair.second;
			Eigen::Vector3d feature_x = feature_pair.first;
			Eigen::Vector3d feature_y = feature_pair.second;
			// Check error
			// Rotate to world
			Eigen::Vector3d feature_x_1 = cam_poses_[id_x].block(0, 0, 3, 3) * feature_x + cam_poses_[id_x].block(0, 3, 3, 1);
			Eigen::Vector3d feature_y_1 = cam_poses_[id_y].block(0, 0, 3, 3) * feature_y + cam_poses_[id_y].block(0, 3, 3, 1);
			
			Eigen::Vector3d error = feature_x_1 - feature_y_1;
			if (error.norm() > threshold) {
				outlier_mask_[i] = true;
				outlier_count_++;
			}
			else {
				outlier_mask_[i] = false;
			}
		}

		ROS_INFO("%d pairs are outlier.", outlier_count_);
	}

    // Instances
    template class Calibrator<3>;
}