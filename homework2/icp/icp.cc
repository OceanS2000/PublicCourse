// Copyright @2019 Pony AI Inc. All rights reserved.

#include "homework2/icp/icp.h"

#include <Eigen/SVD>
#include <Eigen/Dense>

#include "common/utils/math/math_utils.h"
#include "common/utils/math/transform/transform.h"

Icp::Icp(const PointCloud &src_pc, const PointCloud &target_pc) {
  src_points_ = Eigen::MatrixXd::Zero(3, src_pc.points.size());
  for (int i = 0; i < src_pc.points.size(); ++i) {
    src_points_.col(i) = src_pc.points[i];
  }
  transformed_src_points_ = src_points_;

  target_points_ = Eigen::MatrixXd::Zero(3, target_pc.points.size());
  for (int i = 0; i < target_pc.points.size(); ++i) {
    target_points_.col(i) = target_pc.points[i];
  }
  target_pc_knn_ = std::make_unique<KdTree>(&target_points_);
}

bool Icp::RunIteration() {
  int iter = 0;
  while (iter < max_iteration_) {
    int num_correspondence = FindCorrespondence(max_correspondence_distance_);
    if (num_correspondence == 0) {
      LOG(WARNING) << "Fail to find any correspondences, exit.";
      return false;
    }
    const double transform_delta = EstimatePose();
    if (transform_delta < 0.0) {
      LOG(WARNING) << "ICP fail to converge, exit. transform_delta < 0.0";
      return false;
    }
    if (transform_delta < kDefaultMinTransformDelta) {
      const Eigen::Quaterniond quaternion(rotation_);
      LOG(INFO) << "ICP converges at iter: " << iter
                << ". T: " << translation_.transpose()
                << ", R/P/Y: " << math::RadianToDegree(math::transform::GetRoll(quaternion))
                << "," << math::RadianToDegree(math::transform::GetPitch(quaternion))
                << "," << math::RadianToDegree(math::transform::GetYaw(quaternion));
      return true;
    }
    ++iter;
  }
  const Eigen::Quaterniond quaternion(rotation_);
  LOG(WARNING) << "ICP fail to converge within the required iteration. "
               << ". T: " << translation_.transpose()
               << ", R/P/Y: " << math::RadianToDegree(math::transform::GetRoll(quaternion))
               << "," << math::RadianToDegree(math::transform::GetPitch(quaternion))
               << "," << math::RadianToDegree(math::transform::GetYaw(quaternion));
  return false;
}

/*
 *
 * Implement this function. Try to find the correspondences by using the Kd-Tree.
 *
 */
int Icp::FindCorrespondence(double max_correspondence_distance) {
  correspondences_.clear();
  std::vector<int> correspondence_applicants;
  std::vector<double> correspondence_distances;
//  for (auto point : src_points_.colwise()) { This does not work with stable Egien releases
  for (size_t i = 0, nCols = static_cast<size_t>(transformed_src_points_.cols()); i < nCols; ++i) {
    const auto &point = transformed_src_points_.col(i);
    int applicants_number =
        target_pc_knn_->RadiusSearch(point,
                                     max_correspondence_distance,
                                     &correspondence_applicants,
                                     &correspondence_distances);
    if (applicants_number == 0) continue;
    correspondences_.emplace_back(i, correspondence_applicants[0], correspondence_distances[0]);
  }
  return static_cast<int>(correspondences_.size()); // Here the return value is somehow unsafe...
}

/*
 *
 * Implement this function. Estimate R|T given correspondences.
 *
 */
double Icp::EstimatePose() {
  const size_t active_pt_num = correspondences_.size();
  if (active_pt_num < min_num_correspondence_) {
    return -1.0;
  }

  // 1. Construct source/target point matrix from select correspondences;
  // std::cout << "Iteration Begin! " << active_pt_num << std::endl; DEBUGGGGGG
  Eigen::Matrix3Xd active_src_pt = Eigen::MatrixXd::Zero(3, active_pt_num);
  Eigen::Matrix3Xd active_target_pt = Eigen::MatrixXd::Zero(3, active_pt_num);
  for (int i = 0; i < active_pt_num; ++i) {
    active_src_pt.col(i) = transformed_src_points_.col(correspondences_[i].query_index);
    active_target_pt.col(i) = target_points_.col(correspondences_[i].match_index);
  }

  // 2. Find the centroid and demean source/target point matrix;
  auto src_centroid = active_src_pt.rowwise().sum() / active_pt_num;
  auto target_centroid = active_target_pt.rowwise().sum() / active_pt_num;
  for (int i = 0; i < active_pt_num; ++i) {
    active_src_pt.col(i) -= src_centroid;
    active_target_pt.col(i) -= target_centroid;
  }

  // 3. Follow the proof in handout and estimate R|T for this iteration;
  Eigen::Matrix3d covariance_mat = active_src_pt * (active_target_pt.transpose());
  Eigen::JacobiSVD<Eigen::Matrix3d> covariance_svd(covariance_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);

  auto covariance_VUT = (covariance_svd.matrixV() * covariance_svd.matrixU().transpose()).determinant();
  Eigen::Matrix3d rotation_cur_iter =
      covariance_svd.matrixV()
          * (Eigen::Matrix3d() << 1, 0, 0, 0, 1, 0, 0, 0, covariance_VUT).finished()
          * covariance_svd.matrixU().transpose();
  Eigen::Vector3d translation_cur_iter = target_centroid - (rotation_cur_iter * src_centroid);
  // std::cout << rotation_cur_iter << std::endl << "[ " << translation_cur_iter << " ]" << std::endl; DEBUG

  // 4. Transform source pointcloud by using estimated R|T
  transformed_src_points_ = (rotation_cur_iter * transformed_src_points_);
  for (size_t i = 0, nCol = static_cast<size_t>(transformed_src_points_.cols()); i < nCol; ++i)
    transformed_src_points_.col(i) += translation_cur_iter;

  // 5. Update accumulated rotation_ and translation_.
  rotation_ = rotation_cur_iter * rotation_;
  translation_ = translation_cur_iter + translation_;

  return std::max((rotation_cur_iter - Eigen::Matrix3d::Identity()).norm(),
                  translation_cur_iter.norm());
}
