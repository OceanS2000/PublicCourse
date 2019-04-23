// Copyright @2018 Pony AI Inc. All rights reserved.

#include "perception/evaluation_viewer.h"

EvaluationViewer::EvaluationViewer(Options options, QWidget* parent,
                                   const std::string& eval_result_file)
    : PainterWidgetBase(options, parent) {
  CHECK(file::path::Exists(eval_result_file));
  CHECK(file::ReadTextFileToProto(eval_result_file, &eval_result_));

  default_prism_style_.show_vertices = true;
  default_prism_style_.show_edge = true;
  default_prism_style_.show_plane = true;
  default_prism_style_.vertice_style.point_size = 4.0;
  default_prism_style_.edge_style.line_width = 2;
  default_prism_style_.plane_style.alpha = 0.2;

  default_point_style_.point_size = 1.0;
  default_point_style_.point_color = utils::display::Color::Green();
  startTimer(100);
}

void EvaluationViewer::InitializeGlPainter() {
  gl_painter_ = std::make_unique<utils::display::OpenglPainter>(gl_context(), font_renderer());
  gl_painter_->SetupOpenGL();
}

void EvaluationViewer::keyPressEvent(QKeyEvent* event) {
  const int key = event->key();
  switch (key) {
    case Qt::Key_N: {
      frame_index_ = (++frame_index_) % eval_result_.frame_result_size();
      const std::string pointcloud_file = eval_result_.frame_result(frame_index_).log_file_path();
      CHECK(file::path::Exists(pointcloud_file)) << pointcloud_file << " doesn't exist!";
      // Read pointcloud and evaluation results.
      const PointCloud pointcloud = ReadPointCloudFromTextFile(pointcloud_file, true);
      CHECK(!pointcloud.points.empty()) << "Invalid pointcloud file.";
      points_.clear();
      points_.reserve(pointcloud.points.size());
      for (const auto& point : pointcloud.points) {
        Eigen::Vector3d point_in_world = pointcloud.rotation * point + pointcloud.translation;
        points_.emplace_back(point_in_world.x(), point_in_world.y(), point_in_world.z());
      }
      ParsePerceptionFrameResult(eval_result_.frame_result(frame_index_));

      if (frame_index_ == 0) {
        painter_widget_controller_->MutableCamera()->UpdateCenter(
            pointcloud.translation.x(), pointcloud.translation.y(), pointcloud.translation.z());
      }
    }
      break;
    case Qt::Key_D:
      // Draw detected polygon.
      show_detected_polygon_ = !show_detected_polygon_;
      break;
    case Qt::Key_L:
      // Draw labeled polygon.
      show_labled_polygon_ = !show_labled_polygon_;
      break;
  }
}

void EvaluationViewer::Paint3D() {
  if (points_.empty()) {
    return;
  }
  // Draw pointcloud.
  gl_painter()->DrawPoints<math::Vec3d>(
      utils::ConstArrayView<math::Vec3d>(points_.data(), points_.size()), default_point_style_);
  if (show_detected_polygon_) {
    DrawPolygonInfo(detected_polygon_, utils::display::Color::Red());
  }
  if (show_labled_polygon_) {
    DrawPolygonInfo(labeled_polygon_, utils::display::Color::Yellow());
  }
  // Draw evaluation info text.
  font_renderer()->DrawText2D(eval_result_.frame_result(frame_index_).log_file_path(),
                              math::Vec2d(20.0, 20.0),
                              utils::display::Color::Yellow(),
                              "Courier", 16);
  std::string score_info;
  std::vector<std::string> type_name{"UNKNOWN", "CAR", "PEDESTRIAN", "CYCLIST"};
  for (const auto& tp : eval_result_.frame_result(frame_index_).true_positive()) {
    score_info += strings::Format(
        "Type: {}, Correct detection: {} ", type_name[tp.type()], tp.count());
  }
  font_renderer()->DrawText2D(score_info,
                              math::Vec2d(20.0, 40.0),
                              utils::display::Color::Yellow(),
                              "Courier", 16);
}

void EvaluationViewer::ParsePerceptionFrameResult(
    const interface::perception::PerceptionFrameResult& eval_result) {
  labeled_polygon_.clear();
  detected_polygon_.clear();
  int index = 0;
  for (const auto& labeled_polygon_info : eval_result.labeled_polygon()) {
    labeled_polygon_.emplace_back(ParseEffectivePolygonInfo(labeled_polygon_info));
    auto velocity = eval_result.labeled_velocity(index++);
    labeled_polygon_.back().heading = std::atan2(velocity.y(), velocity.x());
    labeled_polygon_.back().speed = std::hypot(velocity.y(), velocity.x());
  }
  index = 0;
  for (const auto& detected_polygon_info : eval_result.detected_polygon()) {
    detected_polygon_.emplace_back(ParseEffectivePolygonInfo(detected_polygon_info));
    auto velocity = eval_result.detected_velocity(index++);
    detected_polygon_.back().heading = std::atan2(velocity.y(), velocity.x());
    detected_polygon_.back().speed = std::hypot(velocity.y(), velocity.x());
  }
}

EvaluationViewer::PolygonInfo EvaluationViewer::ParseEffectivePolygonInfo(
    const interface::perception::EffectivePolygonInfo& polygon_info_pb) {
  PolygonInfo polygon_info;
  polygon_info.id = polygon_info_pb.id();
  polygon_info.floor = polygon_info_pb.floor();
  polygon_info.ceiling = polygon_info_pb.ceiling();
  polygon_info.type = polygon_info_pb.type();
  CHECK_GT(polygon_info_pb.point_size(), 0);
  for (const auto& point : polygon_info_pb.point()) {
    polygon_info.polygon.emplace_back(point.x(), point.y());
  }
  return polygon_info;
}

void EvaluationViewer::DrawPolygonInfo(const std::vector<PolygonInfo>& polygon_info_vector,
                                       const utils::display::Color& color) {
  for (const auto& polygon_info : polygon_info_vector) {
    font_renderer()->DrawText3D(
        strings::Format("{}|{}|{}", polygon_info.id, polygon_info.type, polygon_info.speed),
        math::Vec3d(polygon_info.polygon[0].x,
                    polygon_info.polygon[0].y,
                    polygon_info.ceiling + 0.25),
        color, "Arial", 12);

    default_prism_style_.set_color(color);
    gl_painter()->DrawPrism<math::Vec2d>(
        utils::ConstArrayView<math::Vec2d>(polygon_info.polygon.data(),
                                           polygon_info.polygon.size()),
        polygon_info.ceiling, polygon_info.floor, default_prism_style_);
    math::Vec3d start_point(polygon_info.GetCenter(), polygon_info.ceiling);
    math::Vec3d end_point(polygon_info.GetCenter() +
        math::Vec2d(std::cos(polygon_info.heading), std::sin(polygon_info.heading)),
        polygon_info.ceiling);
    gl_painter()->DrawArrow(start_point, end_point, {});
  }
}
