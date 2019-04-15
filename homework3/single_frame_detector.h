// Copyright @2019 Pony AI Inc. All rights reserved.

#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "common/utils/knn/knn_nanoflann.h"
#include "common/utils/math/vec2d.h"
#include "homework2/pointcloud.h"
#include "homework3/obstacle.h"

using KdTree = utils::KnnNanoflann<utils::NanoflannAdaptor::EigenMatrix<double>, double>;

class SingleFrameDetector {
 public:
  explicit SingleFrameDetector(const std::string& data_dir);

  void GetGroundAndObstacles(
      const PointCloud& point_cloud,
      std::vector<Eigen::Vector3d>* ground_points,
      std::vector<Obstacle>* obstacles);

 protected:
  Eigen::MatrixXd reference_pointcloud_;
  std::unique_ptr<KdTree> reference_knn_;
};
