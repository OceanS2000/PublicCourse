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
  current_destination_ = std::make_unique<math::Vec3d>(dest_x, dest_y, 0.0);

  route->push_back(Point2D(dest_x, dest_y));
  //  if (start_lane == dest_lane) {
  //  route->push_back(Point2D(start_x, start_y));
  //  return route;
  //}

  std::queue<int> search;
  std::map<int, int> route_lane;
  std::map<int, bool> inqueue;
  search.push(start_lane);
  while (!search.empty()) {
    int cur = search.front();
    search.pop();

    for (const auto nxt : map_index_->LaneSucc(cur)) {
      if (inqueue.find(dest_lane) != inqueue.end()) break;
      if (inqueue.find(nxt) == inqueue.end()) {
        route_lane[nxt] = cur;
        search.push(nxt);
        inqueue[nxt] = true;
      }
    }
  }

  int cur = dest_lane;
  if (cur == start_lane) {
    const auto& lane_start = map.lane(cur).central_line().point(0);
    route->push_back(Point2D(lane_start));
    cur = route_lane[cur]; }
  for (; cur != start_lane; cur = route_lane[cur]) {
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

  history_->push_back(current_pes);
  if (history_->size() > history_length) history_->erase(history_->begin());

  if (agent_status.route_status().is_new_request())
    current_route_ = FindRoute(agent_status.vehicle_status().position(),
                               agent_status.route_status().destination());

  const auto& current_pos = agent_status.vehicle_status().position();
  const auto& current_route_p = *(--current_route_->end());
  const auto& current_lane = map_index_->GetMap()
                                 .lane(map_index_->GetPointLane(current_pos))
                                 .central_line();

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
            if (slight.color() != interface::map::Bulb::GREEN)
              desired_vec = -10.0;
            else if (Distance(current_pos, *(--current_lane.point().cend())) <
                     1)
              green_counter_ = greenlight_pass_time;
          }
      }
    }
  }
  ////////////////////////////////////////////////////////////////////////

  ///////////////////////// Obstacles ////////////////////////////////////
  math::Vec3d current_pos_ = Vec3d(current_pos);
  math::Vec3d current_vec_ = Vec3d(current_vcs.velocity());
  math::Vec3d current_vec_orth = current_vec_.OuterProd(math::Vec3d(0, 0, 1));
  current_vec_.Normalize();
  math::Vec3d current_critical_pos = current_pos_ + (current_vec*0.02) * current_vec_;
  for (const auto& object : current_pes.obstacle()) {
    std::vector<interface::geometry::Point3D> object_points;
    for (const auto& p : object.polygon_point()) object_points.push_back(p);
    math::Vec3d object_center = Center(object_points);
    math::Vec3d object_diff = object_center - current_pos_;
    double object_x = object_diff.InnerProd(current_vec_);
    double object_y = object_diff.InnerProd(current_vec_orth);
    double critical_distance = object_center.DistanceToPoint(current_critical_pos);

    if (object.type() == interface::perception::CAR) {
      PublishVariable("Car", object.id());
      if ((object_y < 20) && (object_y > -1) && (object_x < 40)) {
        if (object_x < 0) {
          PublishVariable("Decision", "behind");
          continue;
        }
        if (rele_traffic_light != utils::none) {
          desired_vec = -10.0;
          PublishVariable("Decision", "at crossing");
        }
        if (object_x > 15) {
          desired_vec = std::min(-2.0, desired_vec);
          PublishVariable("Decision", "slowing");
        } else {
          desired_vec = -20.0;
          PublishVariable("Decision", "brake");
        }
      }
    } else {
      // Normal obstacles
      if (critical_distance < 20.0) desired_vec = std::min(-1.0, desired_vec);
      if (critical_distance < 10.0) desired_vec = std::min(-10.0, desired_vec);
    }
  }
  ////////////////////////////////////////////////////////////////////////

  math::Vec3d dest_diff = *current_destination_ - current_pos_;
  double dest_distance = dest_diff.InnerProd(current_vec_);
  if (( dest_distance < 50 ) && ( dest_distance > 0 )) {
    desired_vec = std::min(0.3, desired_vec);
    PublishVariable("car state", "near dest");
  }

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
  //if (false)
  //  desired_wheel = wheel_pid_.RunOneIteration(0.0);
  //else
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
