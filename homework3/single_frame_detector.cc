// Copyright @2019 Pony AI Inc. All rights reserved.

#include "homework3/single_frame_detector.h"
#include "common/utils/file/path.h"

namespace {

void PrintLog() {
  LOG(INFO) << "Ground points have been detected.";
}

}  // namespace

SingleFrameDetector::SingleFrameDetector(const std::string& data_dir) {
  const std::string& ground_pointcloud_file = file::path::Join(data_dir, "lidar_ground.txt");
  CHECK(file::path::Exists(ground_pointcloud_file));
  PointCloud ground_pointcloud = ReadPointCloudFromTextFile(ground_pointcloud_file);

  const std::vector<Eigen::Vector3d>& reference_pointcloud = ground_pointcloud.points;
  reference_pointcloud_ = Eigen::MatrixXd::Zero(3, reference_pointcloud.size());
  for (int i = 0; i < reference_pointcloud.size(); ++i) {
    reference_pointcloud_.col(i) = reference_pointcloud[i];
  }

  reference_knn_ = std::make_unique<KdTree>(&reference_pointcloud_);
}

void SingleFrameDetector::GetGroundAndObstacles(
    const PointCloud& point_cloud,
    std::vector<Eigen::Vector3d>* ground_points,
    std::vector<Obstacle>* obstacles) {
  CHECK(ground_points != nullptr);
  CHECK(obstacles != nullptr);

  std::vector<int> reference_applicants;
  std::vector<double> reference_distances;
  for (const Eigen::Vector3d& point : point_cloud.points) {
    auto const& point_in_world = point_cloud.rotation * point + point_cloud.translation;
    reference_knn_->RadiusSearch(point_in_world,
                                 1.0,
                                 &reference_applicants,
                                 &reference_distances);
    if (reference_applicants.empty()) continue;

    auto const& reference_point = reference_pointcloud_.col(reference_applicants[0]);
    if (std::abs(reference_point.z() - point_in_world.z()) <= 0.42)
      ground_points->push_back(point_in_world);
  }
  // TODO(you): Run flood fill or other algorithms to get the polygons for the obstacles.
  // Still, I provide a fake implementation, please replace it.
  PrintLog();
  obstacles->emplace_back();
  const double x = point_cloud.translation.x();
  const double y = point_cloud.translation.y();
  obstacles->back().polygon.emplace_back(x + 10, y);
  obstacles->back().polygon.emplace_back(x + 9, y);
  obstacles->back().polygon.emplace_back(x + 9, y + 1);
  obstacles->back().floor = point_cloud.translation.z() - 1.3;
  obstacles->back().ceiling = point_cloud.translation.z();
  obstacles->back().id = "Fake obstacle";
}
