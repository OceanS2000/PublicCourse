//
// Created by Ao Shen on 19-06-13
// Many of map_meta.cc is copied here since we need more data about the map.
//

#include "pnc/agents/sao_agent/map_index.h"

#include <Eigen/Core>
#include "common/utils/common/optional.h"
#include "common/utils/knn/knn_nanoflann.h"

namespace sao_agent {
using interface::map::Map;
using interface::map::TrafficLight;
using utils::none;
using utils::Optional;
using KdTree =
    utils::KnnNanoflann<utils::NanoflannAdaptor::EigenMatrix<double>, double>;

namespace {
static inline void UpdateMinMaxValue(double& min_x_, double& max_x_,
                                     double& min_y_, double& max_y_,
                                     double x, double y) {
  min_x_ = std::min(x, min_x_);
  max_x_ = std::max(x, max_x_);
  min_y_ = std::min(y, min_y_);
  max_y_ = std::max(y, max_y_);
}
}  // namespace

void MapIndex::BuildLaneMap() {
  int nLanes = map_.lane_size();
  Eigen::MatrixXd lane_start = Eigen::MatrixXd::Zero(3, nLanes);
  Eigen::MatrixXd lane_end = Eigen::MatrixXd::Zero(3, nLanes);
  std::unique_ptr<KdTree> succ_finder, pred_finder;

  // Find start and end point of lanes
  for (int i = 0; i < nLanes; ++i) {
    auto lane = map_.mutable_lane(i);
    int lane_length = lane->central_line().point_size();

    const auto& start_point = lane->central_line().point(0);
    lane_start.col(i) =
        Eigen::Vector3d{start_point.x(), start_point.y(), start_point.z()};
    const auto& end_point = lane->central_line().point(--lane_length);
    lane_end.col(i) =
        Eigen::Vector3d{end_point.x(), end_point.y(), end_point.z()};
  }

  /*
   * Using KdTree to detect lanes that have same start/end point.
   * If one lane starts at a point another lane ends, then this is a successor.
   */
  succ_finder = std::make_unique<KdTree>(&lane_start);
  pred_finder = std::make_unique<KdTree>(&lane_end);
  std::vector<int> lane_applicants;
  std::vector<double> lane_distances;
  // If two lanes are with this near they are connected.
  constexpr double NEAR_THRESHOLD = 0.2;

  for (int i = 0, nCol = lane_start.cols(); i < nCol; ++i) {
    const auto& end_point = lane_end.col(i);
    int applicants_number = succ_finder->RadiusSearch(
        end_point, NEAR_THRESHOLD, &lane_applicants, &lane_distances);
    if (applicants_number == 0) {
      LOG(ERROR) << "Lane " << i << " seems to be a dead end!";
      continue;
    }
    for (const int j : lane_applicants) {
      auto new_succ = map_.mutable_lane(i)->add_successor();
      *new_succ = map_.lane(j).id();
      succ_map_[i].push_back(j);
    }
  }

  for (int i = 0, nCol = lane_end.cols(); i < nCol; ++i) {
    const auto& start_point = lane_start.col(i);
    int applicants_number = pred_finder->RadiusSearch(
        start_point, NEAR_THRESHOLD, &lane_applicants, &lane_distances);
    if (applicants_number == 0) {
      LOG(ERROR) << "Lane " << i << " seems to start from nowhere!";
      continue;
    }
    for (const int j : lane_applicants) {
      auto new_prec = map_.mutable_lane(i)->add_predecessor();
      *new_prec = map_.lane(j).id();
      pred_map_[i].push_back(j);
    }
  }
}

void MapIndex::BuildCentralLineSegmentIndex() {
  double min_x = min_x_ - 10.0;
  double min_y = min_y_ - 10.0;
  double max_x = max_x_ + 10.0;
  double max_y = max_y_ + 10.0;
  double dimension = 10;
  central_line_segment_index_ = std::make_unique<CentralLineSegmentIndex>(
      min_x, min_y, max_x, max_y, dimension);
  for (const auto& lane : map_.lane()) {
    const auto& central_line = lane.central_line();
    for (int i = 0; i + 1 < central_line.point_size(); i++) {
      math::Vec2d current_point(central_line.point(i).x(),
                                central_line.point(i).y());
      math::Vec2d next_point(central_line.point(i + 1).x(),
                             central_line.point(i + 1).y());
      central_line_segment_index_->GetMutable(current_point.x, current_point.y)
          ->push_back(pnc::map::Segment(current_point, next_point));
    }
  }
}

void MapIndex::BuildTrafficLightIndex() {
  double min_x = min_x_ - 10.0;
  double min_y = min_y_ - 10.0;
  double max_x = max_x_ + 10.0;
  double max_y = max_y_ + 10.0;
  double dimension = 10;
  traffic_light_index_ = std::make_unique<TrafficLightIndex>(
      min_x, min_y, max_x, max_y, dimension);
  for (const auto& traffic_light : map_.traffic_light()) {
    const auto& stop_line = traffic_light.stop_line();
    CHECK(stop_line.point_size() == 2);
    math::Vec2d start_point(stop_line.point(0).x(), stop_line.point(0).y());
    math::Vec2d end_point(stop_line.point(1).x(), stop_line.point(1).y());
    math::Vec2d mid_point = math::Vec2d((start_point.x + end_point.x) / 2,
                                        (start_point.y + end_point.y) / 2);
    traffic_light_index_->GetMutable(mid_point.x, mid_point.y)
        ->push_back(traffic_light);
  }
}

void MapIndex::BuildPointLaneIndex() {
  double min_x = min_x_ - 10.0;
  double min_y = min_y_ - 10.0;
  double max_x = max_x_ + 10.0;
  double max_y = max_y_ + 10.0;
  constexpr double dimension = 10.0;
  point_lane_index_ =
      std::make_unique<PointLaneIndex>(min_x, min_y, max_x, max_y, dimension);
  for (int i = 0, nLane = map_.lane_size(); i < nLane; ++i) {
    const auto& central_line = map_.lane(i).central_line();
    for (int j = 0, nPoint = central_line.point_size(); j < nPoint; ++j) {
      math::Vec2d current_point(central_line.point(j).x(),
                                central_line.point(j).y());
      point_lane_index_->GetMutable(current_point.x, current_point.y)
          ->emplace_back(std::make_pair(current_point, i));
    }
  }
}

int MapIndex::GetPointLane(double x, double y) {
  std::vector<const PointLanePairVector*> vec_lanes =
      point_lane_index_->GetAllInCircle(x, y, 3.0);
  double min_dis = std::numeric_limits<double>::max();
  double dis;
  int res = -1;

  for (const PointLanePairVector* v : vec_lanes) {
    for (const PointLanePair p : *v) {
      dis = std::sqrt((p.first.x - x) * (p.first.x - x) +
                      (p.first.y - y) * (p.first.y - y));
      if (dis < min_dis) {
        min_dis = dis;
        res = p.second;
      }
    }
  }
  return res;
}

void MapIndex::GetMinMaxValue() {
  for (const auto& lane : map_.lane()) {
    for (const auto& point : lane.central_line().point()) {
      UpdateMinMaxValue(min_x_, max_x_, min_y_, max_y_, point.x(), point.y());
    }
    for (const auto& point : lane.left_bound().boundary().point()) {
      UpdateMinMaxValue(min_x_, max_x_, min_y_, max_y_, point.x(), point.y());
    }
    for (const auto& point : lane.right_bound().boundary().point()) {
      UpdateMinMaxValue(min_x_, max_x_, min_y_, max_y_, point.x(), point.y());
    }
  }
}

// Optional<pnc::map::Segment> MapIndex::GetNearestSegment(double x, double y) {
//   std::vector<const CentralLineSegmentVector*> vec_segments =
//       central_line_segment_index_->GetAllInCircle(x, y, 5.0);

//   Optional<pnc::map::Segment> result = utils::none;
//   double min_dis = std::numeric_limits<double>::max();

//   for (const CentralLineSegmentVector* v : vec_segments) {
//     for (const pnc::map::Segment& seg : *v) {
//       double dis = seg.DistanceToPoint(math::Vec2d(x, y));
//       if (dis < min_dis) {
//         min_dis = dis;
//         result = seg;
//       }
//     }
//   }
//   return result;
// }

Optional<TrafficLight> MapIndex::GetRelevantTrafficLight(double x, double y) {
  const auto& current_lane = map_.lane(GetPointLane(x, y)).central_line();
  int sz = current_lane.point_size();
  double x_ = current_lane.point(sz - 1).x();
  double y_ = current_lane.point(sz - 1).y();
  std::vector<const TrafficLightVector*> vec_segments =
      traffic_light_index_->GetAllInCircle(x_, y_, 1.0);
  Optional<TrafficLight> result = utils::none;
  double min_dis = std::numeric_limits<double>::max();
  for (const TrafficLightVector* v : vec_segments) {
    for (const TrafficLight& traffic_light : *v) {
      const auto& stop_line = traffic_light.stop_line();
      math::Vec2d start_point(stop_line.point(0).x(), stop_line.point(0).y());
      math::Vec2d end_point(stop_line.point(1).x(), stop_line.point(1).y());
      pnc::map::Segment seg(start_point, end_point);
      double dis = seg.DistanceToPoint(math::Vec2d(x, y));
      if (dis < min_dis) {
        min_dis = dis;
        result = traffic_light;
      }
    }
  }
  if (min_dis > 15) return utils::none;
  return result;
}
}  // namespace sao_agent
