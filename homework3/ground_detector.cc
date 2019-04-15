//
// Created by Ao Shen on 19-4-13.
//

#include <queue>
#include <map>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <Eigen/Core>

#include "common/utils/file/file.h"
#include "common/utils/file/path.h"
#include "homework2/pointcloud.h"

#define SCALE 2.0
DEFINE_string(pony_data_dir, "", "The path of pony data");
DEFINE_string(lidar_device, "VelodyneDevice32c", "");

typedef std::pair<int, int> INT_PAIR;
typedef std::map<INT_PAIR, std::pair<double, bool> > SEARCH_TRIPLET;

namespace {

void check_point(SEARCH_TRIPLET& search_set,
                 const INT_PAIR& search_point,
                 double reference_height,
                 std::queue<Eigen::Vector3d>* candidate_queue,
                 std::vector<Eigen::Vector3d>* ground_pointcloud) {
  CHECK(candidate_queue != nullptr);
  CHECK(ground_pointcloud != nullptr);
  auto search = search_set.find(search_point);
  if (search == search_set.end()) return;
  if (search->second.second) return;

  search->second.second = true;
  const double actual_height = search->second.first;
  if (std::abs(actual_height - reference_height) <= 0.4) {
    ground_pointcloud->emplace_back(search_point.first / SCALE, search_point.second / SCALE, actual_height);
    candidate_queue->emplace(search_point.first, search_point.second, actual_height);
  }
}

}

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  const std::string pointcloud_dir = file::path::Join(FLAGS_pony_data_dir, FLAGS_lidar_device);
  CHECK(file::path::Exists(pointcloud_dir)) << pointcloud_dir << "doesn't exist!";

  std::map<INT_PAIR, std::pair<double, bool> > minimum_heights_in_world;
  std::queue<Eigen::Vector3d> ground_points;
  PointCloud ground_pointcloud = {};

  auto pointcloud_files = file::path::FindFilesWithPrefixSuffix(pointcloud_dir, "", "txt");
  std::sort(pointcloud_files.begin(), pointcloud_files.end(), file::path::Compare);
  for (const auto& file : pointcloud_files) {
    const PointCloud pointCloud = ReadPointCloudFromTextFile(file);
    CHECK(!pointCloud.points.empty());

    // minimum_heights_in_world.reserve(pointCloud.points.size());
    for (const auto& point : pointCloud.points) {
      Eigen::Vector3d point_in_world = (pointCloud.rotation * point) + pointCloud.translation;
      int reduced_x = static_cast<int>(std::floor(point_in_world.x() * SCALE));
      int reduced_y = static_cast<int>(std::floor(point_in_world.y() * SCALE));
      INT_PAIR reduced_xy{reduced_x, reduced_y};

      auto search = minimum_heights_in_world.find(reduced_xy);
      if (search != minimum_heights_in_world.end()) {
        double* min_height = &(search->second.first);
        const double point_height = point_in_world.z();
        if (*min_height > point_height) *min_height = point_height;
      } else {
        minimum_heights_in_world.emplace(reduced_xy,
                                         std::pair<double, bool>{point_in_world.z(), false});
      }
    }

    auto reduced_x = static_cast<int>(std::floor(pointCloud.translation.x() * SCALE));
    auto reduced_y = static_cast<int>(std::floor(pointCloud.translation.y() * SCALE));
    INT_PAIR initial_ground_point{reduced_x, reduced_y};
    auto search = minimum_heights_in_world.lower_bound(initial_ground_point);
    ground_points.emplace(reduced_x, reduced_y, search->second.first);
    ground_pointcloud.points.emplace_back(reduced_x / SCALE, reduced_y / SCALE, search->second.first);
    search->second.second = true;
    std::cout << initial_ground_point.first << " " << initial_ground_point.second <<
              std::endl << search->second.first << std::endl;
  }

  while (!ground_points.empty()) {
    auto ground_point = ground_points.front();
    check_point(minimum_heights_in_world,
                INT_PAIR{ground_point[0] + 1, ground_point[1]},
                ground_point[2],
                &ground_points,
                &ground_pointcloud.points);
    check_point(minimum_heights_in_world,
                INT_PAIR{ground_point[0] - 1, ground_point[1]},
                ground_point[2],
                &ground_points,
                &ground_pointcloud.points);
    check_point(minimum_heights_in_world,
                INT_PAIR{ground_point[0], ground_point[1] + 1},
                ground_point[2],
                &ground_points,
                &ground_pointcloud.points);
    check_point(minimum_heights_in_world,
                INT_PAIR{ground_point[0], ground_point[1] - 1},
                ground_point[2],
                &ground_points,
                &ground_pointcloud.points);
    ground_points.pop();
  }

  const std::string& ground_pointcloud_file = file::path::Join(FLAGS_pony_data_dir, "lidar_ground.txt");
  WritePointcloudToTextFile(ground_pointcloud, ground_pointcloud_file);

  return 0;
}