// Copyright @2019 Pony AI Inc. All rights reserved.

#pragma once

#include <Eigen/Core>

#include "common/utils/knn/knn_nanoflann.h"
#include "homework2/pointcloud.h"

class Icp {
 public:
  struct Correspondence {
    int query_index = 0;
    int match_index = 0;
    double sqr_distance = 0.0;

    Correspondence(int query_index, int match_index, double sqr_distance)
        : query_index(query_index), match_index(match_index), sqr_distance(sqr_distance) {}
  };

  Icp(const PointCloud& src_pc, const PointCloud& target_pc);
  virtual ~Icp() = default;

  // Run ICP algorithm until it converges or reaches its max_iteration_.
  bool RunIteration();
  // Find correspondences according to max_correspondence_distance.
  int FindCorrespondence(double max_correspondence_distance);
  // Estimate the optimum transform(R|T) given correspondences.
  double EstimatePose();

  void set_max_iteration(int max_iteration) { max_iteration_ = max_iteration; }

  void set_max_correspondence_distance(double max_correspondence_distance) {
    max_correspondence_distance_ = max_correspondence_distance;
  }

  void set_min_num_correspondence(int min_num_correspondence) {
    min_num_correspondence_ = min_num_correspondence;
  }

  const Eigen::MatrixXd src_points() const { return src_points_; }
  const Eigen::MatrixXd target_points() const { return target_points_; }
  const Eigen::MatrixXd transformed_src_points() const { return transformed_src_points_; }

 private:
  using KdTree = utils::KnnNanoflann<utils::NanoflannAdaptor::EigenMatrix<double>, double>;

  static constexpr int kDefaultMaxIteration = 100;
  static constexpr int kDefaultMinNumCorrespondences = 1;
  static constexpr double kDefaultMinTransformDelta = 1e-4;

  int max_iteration_ = kDefaultMaxIteration;
  int min_num_correspondence_ = kDefaultMinNumCorrespondences;
  double max_correspondence_distance_ = -1.0;

  Eigen::MatrixXd src_points_;
  Eigen::MatrixXd target_points_;
  // Save a copy of source points as transformed points.
  Eigen::MatrixXd transformed_src_points_;

  // Accumulated rotation and translation.
  Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_ = Eigen::Vector3d::Zero();

  // Data structure used for searching k nearest neighbors.
  std::unique_ptr<KdTree> target_pc_knn_;

  // Save the correspondences for each scan match iteration.
  std::vector<Correspondence> correspondences_;
};
