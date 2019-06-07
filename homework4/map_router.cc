//
// created by Ao Shen at 2019-05-01
//

#include "common/proto/geometry.pb.h"
#include "common/proto/map.pb.h"
#include "common/proto/route.pb.h"
#include "common/utils/file/path.h"
#include "common/utils/knn/knn_nanoflann.h"
#include "homework4/map/map_lib.h"
#include <Eigen/Core>
#include <cmath>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <map>
#include <queue>

DEFINE_string(map_dir, "", "Directory path of map file");
DEFINE_string(path_dir, "", "Dierectory path of route requests");

namespace {
using interface::route::Route;
using KdTree =
    utils::KnnNanoflann<utils::NanoflannAdaptor::EigenMatrix<double>, double>;

using namespace interface::map;

class LaneLocate {
  /*
   * Wraper class to locate a point to nearest lane.
   *
   * This implementation expolit the fact that in the map given the segment of
   * the proto lane are very short (of length about 1m). So it may not fit in
   * more general case.
   */
public:
  LaneLocate(const Map &map);
  int Locate(const interface::geometry::Point2D &point) const;
  const Lane &GetLane(int id) const { return _map_data.lane(id); };
  std::vector<int> GetSuccessor(int id) const;

private:
  const Map &_map_data;            // should not own the map
  std::vector<int> _lane_boundary; // Divider of point in each lane in KdTree
  std::unique_ptr<Eigen::MatrixXd> _points;
};

std::unique_ptr<Route> FindRoute(std::string route_request_file,
                                 LaneLocate *locater);

// ==== Implementation begin ====
LaneLocate::LaneLocate(const Map &map) : _map_data(map) {
  std::cout << "Constructed! ";
  int nSegments = 0, nLanes = map.lane_size();
  _lane_boundary.reserve(nLanes);
  for (int i = 0; i < nLanes; ++i) {
    nSegments += map.lane(i).central_line().point_size();
    _lane_boundary.push_back(nSegments);
  }

  _points = std::make_unique<Eigen::MatrixXd>(2, nSegments);
  nSegments = 0;
  for (int i = 0; i < nLanes; ++i)
    for (const auto &cpoint : map.lane(i).central_line().point()) {
      _points->col(nSegments) = Eigen::Vector2d{cpoint.x(), cpoint.y()};
      nSegments += 1;
    }
  std::cout << _points->size() << std::endl;
  //  _point_finder = new KdTree(&points); Segmentation fault?
}

int LaneLocate::Locate(const interface::geometry::Point2D &point) const {
  std::cout << "Begin locate!" << std::endl;
  std::vector<int> candidates_id;
  std::vector<double> candidates_distance;
  Eigen::Vector2d point_2d {point.x(), point.y()};
  std::cout << point_2d << std::endl;
  double min_distance = 1e5;
  int min_index = 0;
  for (int i = 0, nCol = _points->cols(); i < nCol; ++i) {
    Eigen::Vector2d diff =
        _points->col(i) - point_2d;
    double distance = std::sqrt(diff.x() * diff.x() + diff.y() * diff.y());
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  candidates_id.push_back(min_index);
  std::cout << "ID: " << candidates_id.front() << std::endl;
  CHECK(!candidates_id.empty());

  // Using binary search to locate the point in the lane
  int lo = -1, hi = _lane_boundary.size() - 1;
  while (hi - lo > 1) {
    int mid = (lo + hi) / 2;
    std::cout << mid << std::endl;
    if (_lane_boundary[mid] > candidates_id[0])
      hi = mid;
    else if (_lane_boundary[mid] < candidates_id[0])
      lo = mid;
    else if (_lane_boundary[mid] == candidates_id[0])
      return mid;
  }
  return hi;
}

std::vector<int> LaneLocate::GetSuccessor(int id) const {
  /*
   * By no means should id of lanes in a map photo be in the format 'L%d'. In
   * more general case it should use a std::map to find the assocation of lane
   * id with its place in the map. Here I do use such expoliting on the data
   * since my time is limited.
   */
  std::cout << "Successors? ";
  std::vector<int> successors;
  for (const auto &succ_lanes : _map_data.lane(id).successor()) {
    int succ_id = std::stoi(succ_lanes.id().substr(1));
    successors.push_back(succ_id);
  }
  std::cout << successors.size() << std::endl;
  return successors;
}

std::unique_ptr<Route> FindRoute(std::string route_request_file,
                                 LaneLocate *locater) {
  std::cout << "Find begin!" << std::endl;
  auto route = std::make_unique<Route>();
  CHECK(file::ReadTextFileToProto(route_request_file, route.get()))
      << "Reading from " << route_request_file << " failed!";
  int start_lane = locater->Locate(route->start_point());
  int end_lane = locater->Locate(route->end_point());

  // Using bread-first search to locate the shortest lane path.
  std::queue<int> search;
  std::map<int, int> father;
  std::vector<int> lane_route;

  std::cout << "search begin" << std::endl;
  search.push(start_lane);
  int current = start_lane;
  bool search_finished = false;
  while ((!search.empty()) && (!search_finished)) {
    std::cout << "current0 " << current;
    current = search.front();
    std::cout << " current1 " << current << std::endl;

    std::vector<int> candidates = locater->GetSuccessor(current);
    std::cout << candidates.size() << std::endl;
    search.pop();
    for (auto candidate : candidates) {
      std::cout << candidate << std::endl;
      if (father.find(candidate) == father.end()) {
        father.insert(std::make_pair(candidate, current));
        search.push(candidate);
      }
      // if (candidate == end_lane)
      //  search_finished = true;
      std::cout << "candidate finished" << std::endl;
    }
  }
  std::cout << "search end" << std::endl;

  CHECK(current == end_lane) << "There seems to be no route in the request!";
  while (current != start_lane) {
    current = father.find(current)->second;
    lane_route.push_back(current);
  }

  bool if_passed = (route->start_point().x() <
                    locater->GetLane(start_lane).central_line().point(0).x());
  for (const auto &point :
       locater->GetLane(start_lane).central_line().point()) {
    if (if_passed != (route->start_point().x() < point.x())) {
      auto new_point = route->add_route_point();
      interface::geometry::Point2D point2d;
      point2d.set_x(point.x());
      point2d.set_y(point.y());
      *new_point = point2d;
    }
  }
  lane_route.pop_back();
  for (auto lane = lane_route.rbegin(); lane != lane_route.rend(); lane++) {
    for (const auto &point : locater->GetLane(*lane).central_line().point()) {
      auto new_point = route->add_route_point();
      interface::geometry::Point2D point2d;
      point2d.set_x(point.x());
      point2d.set_y(point.y());
      *new_point = point2d;
    }
  }
  if_passed = (route->end_point().x() <
               locater->GetLane(end_lane).central_line().point(0).x());
  for (const auto &point : locater->GetLane(end_lane).central_line().point()) {
    if (if_passed == (route->end_point().x() < point.x())) {
      auto new_point = route->add_route_point();
      interface::geometry::Point2D point2d;
      point2d.set_x(point.x());
      point2d.set_y(point.y());
      *new_point = point2d;
    }
  }

  return route;
}
// ==== Implementation end ====

} // namespace

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  interface::map::Map map_data;
  if (FLAGS_map_dir.empty()) {
    CHECK(file::ReadFileToProto(
        "/home/ocean/PublicCourse/homework4/map/grid2/processed_map_proto.txt",
        &map_data))
        << "Map data doesn't exist!";
  } else {
    CHECK(file::ReadFileToProto(
        file::path::Join(FLAGS_map_dir, "processed_map_proto.txt"), &map_data))
        << "Map data `map_proto.txt` in " << FLAGS_map_dir << " doesn't exist!";
  }
  std::vector<std::string> route_requests;
  LaneLocate *locater = new LaneLocate(map_data);

  if (FLAGS_path_dir.empty()) {
    LOG(FATAL) << "Route request doesn't exist!";
    return -1;
  }
  std::string request_path = file::path::Join(FLAGS_path_dir, "route_request_4.txt");
  std::unique_ptr<Route> route = FindRoute(request_path, locater);
  /*
  route_requests = file::path::FindFilesWithPrefixSuffix(
      FLAGS_path_dir, "route_request", ".txt");

  std::cout << "Read finished!" << std::endl;
  for (const auto &request_path : route_requests) {
    std::unique_ptr<Route> route = FindRoute(request_path, locater);

    // an ugly code to get the name of the route file
    // length of "route_request" is 13
    std::string route_suffix = file::path::Basename(request_path).substr(13);
    CHECK(file::WriteProtoToTextFile(
        *route,
        file::path::Join(FLAGS_path_dir, "route_result" + route_suffix)))
        << "Write to route file failed!";
  }
  */
  delete locater;
  return 0;
}
