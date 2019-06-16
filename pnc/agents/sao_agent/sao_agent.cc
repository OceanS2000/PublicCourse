//
// Created by Ao Shen on 19-6-13.
//

#include "pnc/agents/sao_agent/sao_agent.h"

#include <map>
#include <queue>

#include "common/utils/common/optional.h"

namespace sao_agent {
using utils::none;
using utils::Optional;

std::unique_ptr<std::vector<interface::geometry::Point2D> >
SaoVehicleAgent::FindRoute(double start_x, double start_y, double dest_x,
                           double dest_y) {
  const auto& map = map_index_->GetMap();
  auto route = std::make_unique<Route>();
  auto start_lane = map_index_->GetPointLane(start_x, start_y);
  auto dest_lane = map_index_->GetPointLane(dest_x, dest_y);

  route->push_back(Point2D(dest_x, dest_y));
  if (start_lane == dest_lane) {
    route->push_back(Point2D(start_x, start_y));
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
      const auto& lane_start = map.lane(cur).central_line().point(0);
      route->push_back(Point2D(lane_start));
    } else {
      const auto& lane_points = map.lane(cur).central_line().point();
      for (auto p = lane_points.rbegin(); p != lane_points.rend(); ++p)
        route->push_back(Point2D(*p));
    }
  }

  route->push_back(Point2D(start_x, start_y));
  return route;
}

interface::control::ControlCommand SaoVehicleAgent::RunOneIteration(
    const interface::agent::AgentStatus& agent_status) {
  interface::control::ControlCommand res;
  const VehicleStatus& current_vcs = agent_status.vehicle_status();
  const PerceptionStatus& current_pes = agent_status.perception_status();

  history_->push_back(agent_status.vehicle_status());
  if (history_->size() > history_length) history_->erase(history_->begin());

  if (agent_status.route_status().is_new_request())
    current_route_ = FindRoute(agent_status.vehicle_status().position(),
                               agent_status.route_status().destination());

  const auto& current_pos = agent_status.vehicle_status().position();
  const auto& current_route_p = *(--current_route_->end());
  PublishVariable("checkpoint_distance",
                  std::to_string(Distance(current_pos, current_route_p)));
  if (Distance(current_pos, current_route_p) < checkpoint_threshold &&
      current_route_->size() > 2) {
    current_route_->pop_back();
    // passed_point_++;
  }
  //  PublishVariable("passed_points", std::to_string(passed_point_));

  double desired_vec = vec_limit;
  double current_vec = CalcVelocity(current_vcs.velocity());
  math::Vec3d desired_head = GetRouteHeading(current_pos);
  PublishVariable("desired_x", std::to_string(desired_head.x));
  PublishVariable("desired_y", std::to_string(desired_head.y));

  ///////////////////////// Red Lights //////////////////////////////////
  const auto& rele_traffic_light =
      map_index_->GetRelevantTrafficLight(current_pos);

  if (green_counter_ > 0) {
    green_counter_--;
    PublishVariable("Ignore redlight", std::to_string(green_counter_),
                    utils::display::Color::Green());
  } else {
    if (rele_traffic_light != none) {
      for (const auto& light : current_pes.traffic_light()) {
        for (const auto& slight : light.single_traffic_light_status())
          if (slight.id().id() == rele_traffic_light->id().id()) {
            if (slight.color() == interface::map::Bulb::RED)
              desired_vec = -10.0;
            else
              green_counter_ = greenlight_pass_time;
          }
      }
    }
  }
  ////////////////////////////////////////////////////////////////////////

  ///////////////////////// Obstacles ////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////

  vec_pid_.SetGoal(desired_vec);
  double desired_acc = vec_pid_.RunOneIteration(current_vec);
  if (Distance(current_pos, agent_status.route_status().destination()) <
      goal_near_threshold) {
    desired_acc = -10.0;
    PublishVariable("car state", "near dest (hard break)");
  } else if (rele_traffic_light != none) {
    PublishVariable("car state", "found red light");
  } else
    PublishVariable("car state", "normal");
  if (desired_acc > 0)
    res.set_throttle_ratio(GetThrottle(desired_acc));
  else
    res.set_brake_ratio(GetBrake(desired_acc));
  PublishVariable("acc", std::to_string(desired_acc));

  double head_sin = CrossProd(current_vcs.velocity(), desired_head);
  double desired_wheel;
  if (false)
    desired_wheel = wheel_pid_.RunOneIteration(0.0);
  else
    desired_wheel = wheel_pid_.RunOneIteration(head_sin);
  res.set_steering_angle(-desired_wheel);
  PublishVariable("rotate", std::to_string(desired_wheel));
  return res;
}

math::Vec3d SaoVehicleAgent::GetRouteHeading(double x, double y) {
  int sz = current_route_->size();
  // if (sz < 3) {
  //   return math::Vec3d(current_route_->at(0).x() - current_route_->at(1).x(),
  //                      current_route_->at(0).y() - current_route_->at(1).y(),
  //                      0.0);
  // }
  if (sz < 9)
    sz = 0;
  else
    sz = sz - 9;
  const auto& next = current_route_->at(sz);
  return math::Vec3d(next.x() - x, next.y() - y, 0.0);
}

double SaoVehicleAgent::GetThrottle(double acc) {
  return std::min(throttle_limit, acc);
}
double SaoVehicleAgent::GetBrake(double acc) { return std::min(1.0, -acc); }
}  // namespace sao_agent
