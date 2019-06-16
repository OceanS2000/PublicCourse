//
// Created by Ao Shen on 19-6-13.
//

#ifndef PUBLICCOURSE_SAO_AGENT_H
#define PUBLICCOURSE_SAO_AGENT_H

#include "pnc/simulation/vehicle_agent.h"
#include "pnc/simulation/vehicle_agent_factory.h"

#include <glog/logging.h>
#include <memory>
#include <string>
#include <vector>

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/map.pb.h"
#include "common/proto/route.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "pnc/agents/sao_agent/constants.h"
#include "pnc/agents/sao_agent/helpers.h"
#include "pnc/agents/sao_agent/map_index.h"

namespace sao_agent {
using namespace interface::agent;

class SaoVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit SaoVehicleAgent(const std::string& name)
    : VehicleAgent(name), vec_pid_(10, -10, iteration_time), wheel_pid_(10, -10, iteration_time) {
    green_counter_ = 0;
    vec_pid_.Tone(V::P, V::I, V::D);
    wheel_pid_.Tone(W::P, W::I, W::D);
    wheel_pid_.SetGoal(0.0);
  }

  void Initialize(const AgentStatus& status) override {
    if (status.route_status().is_new_request())
      current_route_ = FindRoute(status.vehicle_status().position(),
                                 status.route_status().destination());
    map_index_ = std::make_unique<sao_agent::MapIndex>(map_lib().map_proto());
    history_ = std::make_unique<std::vector<PerceptionStatus> >();
  }

  interface::control::ControlCommand RunOneIteration(
      const AgentStatus& agent_status) override;

 private:
  double GetThrottle(double acc);
  double GetBrake(double acc);

  math::Vec3d GetRouteHeading(double x, double y);
  math::Vec3d GetRouteHeading(const interface::geometry::Vector3d& place) {
    return GetRouteHeading(place.x(), place.y());
  }

  using Map = interface::map::Map;
  using Route = std::vector<interface::geometry::Point2D>;
  std::unique_ptr<Route> FindRoute(const interface::geometry::Vector3d& start,
                                   const interface::geometry::Point3D& dest) {
    return FindRoute(start.x(), start.y(), dest.x(), dest.y());
  }
  std::unique_ptr<Route> FindRoute(double start_x, double start_y,
                                   double dest_x, double dest_y);
  std::unique_ptr<MapIndex> map_index_;
  std::unique_ptr<Route> current_route_;
  std::unique_ptr<math::Vec3d> current_destination_;

  using VehicleHistory = std::vector<PerceptionStatus>;
  std::unique_ptr<VehicleHistory> history_;

  PID vec_pid_, wheel_pid_;
  int green_counter_;

  // For debugging only
  // int passed_point_ = 0;
};
}  // namespace sao_agent

#endif  // PUBLICCOURSE_SAO_AGENT_H
