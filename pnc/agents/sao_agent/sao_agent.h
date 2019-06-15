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
#include "pnc/agents/sao_agent/map_index.h"

namespace sao_agent {
using namespace interface::agent;

class SaoVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit SaoVehicleAgent(const std::string& name) : VehicleAgent(name) {}

  void Initialize(const AgentStatus& status) override {
    if (status.route_status().is_new_request())
      current_route_ = FindRoute(status.vehicle_status().position(),
                                 status.route_status().destination());
    map_index_ = std::make_unique<sao_agent::MapIndex>(map_lib().map_proto());
  }

  interface::control::ControlCommand RunOneIteration(
      const AgentStatus& agent_status) override;

 private:
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
};
}  // namespace sao_agent

#endif  // PUBLICCOURSE_SAO_AGENT_H
