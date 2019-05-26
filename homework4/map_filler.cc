//
// Created by Ao Shen on 19-5-1.
//

#include "common/proto/map.pb.h"
#include "common/utils/file/file.h"
#include "common/utils/file/path.h"
#include "common/utils/knn/knn_nanoflann.h"
#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(map_dir, "", "Directory path of map file");

using KdTree =
    utils::KnnNanoflann<utils::NanoflannAdaptor::EigenMatrix<double>, double>;

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  std::string proto_dir; // Directory path of the output to be written
  interface::map::Map map_data;
  if (FLAGS_map_dir.empty()) {
    CHECK(file::ReadFileToProto("/home/ocean/PublicCourse/homework4/map/grid2/map_proto.txt", &map_data))
        << "Map data doesn't exist!";
    proto_dir = "/home/ocean/PublicCourse/homework4/map/grid2/";
  } else {
    CHECK(file::ReadFileToProto(
        file::path::Join(FLAGS_map_dir, "map_proto.txt"), &map_data))
        << "Map data `map_proto.txt` in " << FLAGS_map_dir << " doesn't exist!";
    proto_dir = FLAGS_map_dir;
  }

  int nLanes = map_data.lane_size();
  Eigen::MatrixXd lane_start = Eigen::MatrixXd::Zero(3, nLanes);
  Eigen::MatrixXd lane_end = Eigen::MatrixXd::Zero(3, nLanes);
  std::unique_ptr<KdTree> succ_finder, prec_finder;

  // Find start and end point of lanes
  for (int i = 0; i < nLanes; ++i) {
    auto lane = map_data.mutable_lane(i);
    int lane_length = lane->central_line().point_size();

    const auto &start_point = lane->central_line().point(0);
    lane_start.col(i) =
        Eigen::Vector3d{start_point.x(), start_point.y(), start_point.z()};
    const auto &end_point = lane->central_line().point(--lane_length);
    lane_end.col(i) =
        Eigen::Vector3d{end_point.x(), end_point.y(), end_point.z()};
    std::cout << "Lane " << i << " finished!" << std::endl;;
  }

  /*
   * Using KdTree to detect lanes that have same start/end point.
   * If one lane starts at a point another lane ends, then this is a successor.
   */
  succ_finder = std::make_unique<KdTree>(&lane_start);
  prec_finder = std::make_unique<KdTree>(&lane_end);
  std::vector<int> lane_applicants;
  std::vector<double> lane_distances;
  constexpr double NEAR_THRESHOLD = 0.1; // If two lanes are with this near they are connected.

  for (int i = 0, nCol = lane_start.cols(); i < nCol; ++i) {
    const auto& end_point = lane_end.col(i);
    int applicants_number =
        succ_finder->RadiusSearch(end_point, NEAR_THRESHOLD,
                                  &lane_applicants,
                                  &lane_distances);
    if (applicants_number == 0) {
      LOG(ERROR) << "Lane " << i << " seems to be a dead end!";
      continue;
    }
    for(const int j : lane_applicants) {
      auto new_succ = map_data.mutable_lane(i)->add_successor();
      *new_succ = map_data.lane(j).id();
    }
  }

  for (int i = 0, nCol = lane_end.cols(); i < nCol; ++i) {
    const auto& start_point = lane_start.col(i);
    int applicants_number =
        prec_finder->RadiusSearch(start_point, NEAR_THRESHOLD,
                                  &lane_applicants,
                                  &lane_distances);
    if (applicants_number == 0) {
      LOG(ERROR) << "Lane " << i << " seems to start from nowhere!";
      continue;
    }
    for (const int j : lane_applicants) {
      auto new_prec = map_data.mutable_lane(i)->add_predecessor();
      *new_prec = map_data.lane(j).id();
    }
  }


  CHECK(file::WriteProtoToTextFile(map_data,
                                   file::path::Join(proto_dir, "processed_map_proto.txt")))
      << "Write failed!";
}
