// Copyright @2018 Pony AI Inc. All rights reserved.

#include "perception/perception.h"

interface::perception::PerceptionObstacles Perception::OneIteration(const PointCloud& pointcloud) {
  interface::perception::PerceptionObstacles perception_result;
  // Add a mocked up obstacle.
  {
    auto* obstacle = perception_result.add_obstacle();
    obstacle->set_type(interface::perception::ObjectType::CAR);
    obstacle->set_heading(2.5);
    obstacle->set_speed(10.0);
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(-2959.18);
      polygon_point->set_y(6759.15);
      polygon_point->set_z(-15.77);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(-2955.04);
      polygon_point->set_y(6756.34);
      polygon_point->set_z(-15.774);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(-2953.64);
      polygon_point->set_y(6758.41);
      polygon_point->set_z(-15.774);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(-2957.77);
      polygon_point->set_y(6761.22);
      polygon_point->set_z(-15.774);
    }
    obstacle->set_height(2.69);
    obstacle->set_id("c83");
  }

  // Add a mocked up obstacle.
  {
    auto* obstacle = perception_result.add_obstacle();
    obstacle->set_type(interface::perception::ObjectType::CAR);
    obstacle->set_heading(-1.0);
    obstacle->set_speed(10.0);
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(972.10);
      polygon_point->set_y(1084.33);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(971.20);
      polygon_point->set_y(1088.79);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(969.00);
      polygon_point->set_y(1088.34);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(969.91);
      polygon_point->set_y(1083.89);
      polygon_point->set_z(-8.18);
    }
    obstacle->set_height(1.56);
    obstacle->set_id("c84");
  }

  // Add a mocked up obstacle.
  {
    auto* obstacle = perception_result.add_obstacle();
    obstacle->set_type(interface::perception::ObjectType::PEDESTRIAN);
    obstacle->set_heading(0.0);
    obstacle->set_speed(10.0);
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(967.10);
      polygon_point->set_y(1084.3295967224144);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(965.19);
      polygon_point->set_y(1088.79);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(964.01);
      polygon_point->set_y(1088.34);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(964.91);
      polygon_point->set_y(1083.89);
      polygon_point->set_z(-8.18);
    }
    obstacle->set_height(1.56);
    obstacle->set_id("c88");
  }

  LOG(INFO) << "Perception done.";
  return perception_result;
}