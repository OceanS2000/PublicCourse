//
// Created by Ao Shen on 19-6-13.
//

#include "pnc/agents/sao_agent/sao_agent.h"

#include <map>
#include <queue>

namespace sao_agent {

std::unique_ptr<std::vector<interface::geometry::Point2D> >
SaoVehicleAgent::FindRoute(double start_x, double start_y, double dest_x,
                           double dest_y) {
  const auto& map = map_index_->GetMap();
  auto route = std::make_unique<Route>();
  auto start_lane = map_index_->GetPointLane(start_x, start_y);
  auto dest_lane = map_index_->GetPointLane(dest_x, dest_y);

  route->emplace_back(dest_x, dest_y);
  if (start_lane == dest_lane) {
    route->emplace_back(start_x, start_y);
    return route;
  }

  std::queue<int> search;
  std::map<int, int> route_lane;
  route_lane[start_lane] = -1;
  search.push(start_lane);
  while (!search.empty()) {
    int cur = search.front();
    search.pop();
    if (cur == dest_lane) break;

    for (const auto nxt : map_index_->LaneSucc(cur)) {
      if (route_lane.find(nxt) == route_lane.end()) {
        route_lane[nxt] = cur;
        search.push(nxt);
      }
    }
  }

  for (int cur = dest_lane; cur != start_lane; cur = route_lane[cur]) {
    if (cur == dest_lane) {
      const auto& start = map.lane(cur).central_line().point(0);
      route->emplace_back(start.x(), start.y());
      continue;
    }
    const auto & cur_points = map.lane(cur).central_line().point();
    for (const auto point = cur_points.rbegin();
         point != cur_points.rend(); ++point) {
      route->emplace_back(point->x(), point->y());
    }
  }

  route->emplace_back(start_x, start_y);
  return route;
}

interface::control::ControlCommand SaoVehicleAgent::RunOneIteration(
    const interface::agent::AgentStatus& agent_status) {
  if (agent_status.route_status().is_new_request())
    current_route_ = FindRoute(agent_status.vehicle_status().position(),
                               agent_status.route_status().destination());

  if (!agent_status.simulation_status().is_alive())
    return interface::control::ControlCommand();

  return interface::control::ControlCommand();
}
}  // namespace sao_agent
