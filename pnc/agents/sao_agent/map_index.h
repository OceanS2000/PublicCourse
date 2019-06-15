//
// Created by Ao Shen on 19-06-13
// Many of map_meta.cc is copied here since we need more data about the map.
//

#ifndef PUBLICCOURSE_MAP_INDEX_H
#define PUBLICCOURSE_MAP_INDEX_H

#include "common/proto/geometry.pb.h"
#include "common/proto/map.pb.h"
#include "common/utils/common/optional.h"
#include "common/utils/index/grid_index.h"
#include "pnc/map/segment.h"

namespace sao_agent {
using utils::Optional;
class MapIndex {
 public:
  explicit MapIndex(const interface::map::Map& map) : map_(map) {
    GetMinMaxValue();
    BuildLaneMap();
    BuildCentralLineSegmentIndex();
    BuildTrafficLightIndex();
    BuildPointLaneIndex();
  }

  int GetPointLane(double x, double y);
  int GetPointLane(const math::Vec2d& point) {
    return GetPointLane(point.x, point.y);
  }

  const interface::map::Map& GetMap() { return map_; }
  interface::map::Map* GetMutableMap() {
    return &map_;
  }

  std::vector<int> LaneSucc(int lane) { return succ_map_.at(lane); }
  std::vector<int> LanePred(int lane) { return pred_map_.at(lane); }

  Optional<pnc::map::Segment> GetNearestSegment(double x, double y);
  Optional<interface::map::TrafficLight> GetRelevantTrafficLight(double x,
                                                                 double y);

 protected:
  void BuildLaneMap();
  void BuildCentralLineSegmentIndex();
  void BuildTrafficLightIndex();
  void BuildPointLaneIndex();
  void GetMinMaxValue();

 private:
  using LaneMap = std::map<int, std::vector<int> >;
  LaneMap succ_map_, pred_map_;

  using CentralLineSegmentVector = std::vector<pnc::map::Segment>;
  using CentralLineSegmentIndex = utils::GridIndex<CentralLineSegmentVector>;
  std::unique_ptr<CentralLineSegmentIndex> central_line_segment_index_;

  using TrafficLightVector = std::vector<interface::map::TrafficLight>;
  using TrafficLightIndex = utils::GridIndex<TrafficLightVector>;
  std::unique_ptr<TrafficLightIndex> traffic_light_index_;

  using PointLanePair = std::pair<math::Vec2d, int>;
  using PointLanePairVector = std::vector<PointLanePair>;
  using PointLaneIndex = utils::GridIndex<PointLanePairVector>;
  std::unique_ptr<PointLaneIndex> point_lane_index_;

  double min_x_ = std::numeric_limits<double>::max();
  double min_y_ = std::numeric_limits<double>::max();
  double max_x_ = std::numeric_limits<double>::min();
  double max_y_ = std::numeric_limits<double>::min();

  interface::map::Map map_;
};
}  // namespace sao_agent

#endif  // PUBLICCOURSE_MAP_INDEX_H
