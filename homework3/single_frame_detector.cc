// Copyright @2019 Pony AI Inc. All rights reserved.

#include <map>
#include <iostream>
#include "homework3/single_frame_detector.h"
#include "common/utils/file/path.h"

#define SCALE 0.5
#define MAGIC_SIZE 1000
#define GROUND_THRESHOLD 0.45
#define SKY_THRESHOLD 6.0

#define XMOVE 1100
#define YMOVE 1000
namespace {

class UnionFound {
 public:
  UnionFound();
  ~UnionFound();

  void Union (int x1, int y1, int x2, int y2);
  void Union (const Eigen::Vector2i& x, const Eigen::Vector2i& y) {
    return Union(x.x(), x.y(), y.x(), y.y()); };

  Eigen::Vector2i FindFather (int x, int y);
  Eigen::Vector2i FindFather (const Eigen::Vector2i& p) {
    return FindFather(p.x(), p.y());
  };

 protected:
  Eigen::Vector2i* set_[MAGIC_SIZE][MAGIC_SIZE];
};

UnionFound::UnionFound() {
  for(int i = 1; i < MAGIC_SIZE; i++)
    for(int j = 1; j < MAGIC_SIZE; j++)
      set_[i][j] = new Eigen::Vector2i{i,j};
}

UnionFound::~UnionFound() {
  for(int i = 1; i < MAGIC_SIZE; i++) {
    for (int j = 1; j < MAGIC_SIZE; j++) {
      delete set_[i][j];
      set_[i][j] = nullptr;
    }
  }
}

void UnionFound::Union(int x1, int y1, int x2, int y2) {
  Eigen::Vector2i a = FindFather(x1, y1);
  Eigen::Vector2i b = FindFather(x2, y2);

  if (a == b) return;
  *set_[b.x()][b.y()] = *set_[a.x()][a.y()];
}

Eigen::Vector2i UnionFound::FindFather(int x, int y) {
  std::vector<Eigen::Vector2i> path;
  Eigen::Vector2i p = *set_[x][y];

  while (p.x() != x || p.y() != y) {
    path.emplace_back(x, y);
    x = p.x();
    y = p.y();
    p = *set_[x][y];
  }

  //for (const auto& q : path) {
  //  *set_[q.x()][q.y()] = *set_[x][y];
  //}

  return Eigen::Vector2i {x, y};
}

struct Vec2iCmp{
  bool operator()(const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) {
    if (lhs.x() == rhs.x()) return lhs.y() < rhs.y();
    else                    return lhs.x() < rhs.x();
  }
};

Eigen::Vector2i GetReducedXY(Eigen::Vector3d p) {
  int reduced_x = static_cast<int>(std::floor(p.x() / SCALE + XMOVE));
  int reduced_y = static_cast<int>(std::floor(p.y() / SCALE - YMOVE));
  DCHECK(reduced_x >= 0 && reduced_y >= 0) << "Segment Fault may be caused!";
  return Eigen::Vector2i{reduced_x, reduced_y};
}

std::vector<Eigen::Vector2i> GetBoundary(const std::vector<Eigen::Vector2i> p) {
  int min_x = INT_MAX, min_y=INT_MAX, max_x = INT_MIN, max_y = INT_MIN;
  for(const auto k : p) {
    if(k.x() < min_x) min_x = k.x();
    if(k.x() > max_x) max_x = k.x();
    if(k.y() < min_y) min_y = k.y();
    if(k.y() > max_y) max_y = k.y();
  }
  std::vector<Eigen::Vector2i> bound;
  bound.emplace_back(min_x, min_y);
  bound.emplace_back(min_x, max_y);
  bound.emplace_back(max_x, max_y);
  bound.emplace_back(max_x, min_y);
  return bound;
}

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
  UnionFound obstacles_detector;
  std::vector<Eigen::Vector2i> obstacles_applicants;

  for (const Eigen::Vector3d& point : point_cloud.points) {
    auto const& point_in_world = point_cloud.rotation * point + point_cloud.translation;
    reference_knn_->RadiusSearch(point_in_world,
                                 2.0,
                                 &reference_applicants,
                                 &reference_distances);
    if (reference_applicants.empty()) continue;

    auto const& reference_point = reference_pointcloud_.col(reference_applicants[0]);
    const double actual_height = std::abs(reference_point.z() - point_in_world.z());
    if (actual_height <= GROUND_THRESHOLD)
      ground_points->push_back(point_in_world);
    else if (actual_height <= SKY_THRESHOLD) {
      Eigen::Vector2i reduced_xy = GetReducedXY(point_in_world);
      std::cout << reduced_xy.x() << " , " << reduced_xy.y() << std::endl;
      obstacles_applicants.push_back(reduced_xy);
      for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j)
          obstacles_detector.Union(reduced_xy, reduced_xy + Eigen::Vector2i{i,j});
    }
  }

  std::map<Eigen::Vector2i, std::vector<Eigen::Vector2i>, Vec2iCmp> obstacle_points;

  for (const auto& obstacle_applicant : obstacles_applicants) {
    Eigen::Vector2i obstacle_root = obstacles_detector.FindFather(obstacle_applicant);
    auto search = obstacle_points.find(obstacle_root);

    if (search == obstacle_points.end()) {
      obstacle_points.emplace(obstacle_root, std::vector<Eigen::Vector2i>{obstacle_root});
    } else {
      search->second.emplace_back(obstacle_applicant);
    }
  }
  std::cout << obstacles_applicants.size() << std::endl;
  std::cout << obstacle_points.size() << std::endl;

  PrintLog();

  auto key = obstacle_points.cbegin();
  for (int id = 1, nOb = obstacle_points.size(); id <= nOb; ++id) {
    obstacles->emplace_back();

    for (const auto& p : GetBoundary(key->second))
      obstacles->back().polygon.emplace_back((p.x() - XMOVE) * SCALE, (p.y() + YMOVE) * SCALE);

    obstacles->back().floor = point_cloud.translation.z() - 1.3;
    obstacles->back().ceiling = point_cloud.translation.z();
    obstacles->back().id = "Fake obstacle " + std::to_string(id);
    key++;
  }
}
