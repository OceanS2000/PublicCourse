// Copyright @2019 Pony AI Inc. All rights reserved.

#include "homework2/icp/icp_viewer.h"

IcpViewer::IcpViewer(Options options, QWidget* parent,
                     const PointCloud& src_pc,
                     const PointCloud& target_pc)
    : PainterWidgetBase(options, parent), src_pc_(src_pc), target_pc_(target_pc) {
  src_point_style_.point_size = 1.0;
  src_point_style_.point_color = utils::display::Color::Yellow();
  target_point_style_.point_size = 1.0;
  target_point_style_.point_color = utils::display::Color::Red();

  icp_ = std::make_unique<Icp>(src_pc_, target_pc_);
  icp_->set_max_correspondence_distance(5.0);
  icp_->set_max_iteration(1);

  startTimer(100);
}

void IcpViewer::InitializeGlPainter() {
  gl_painter_ = std::make_unique<utils::display::OpenglPainter>(gl_context(), font_renderer());
  gl_painter_->SetupOpenGL();
  // Update camera center.
  painter_widget_controller_->MutableCamera()->UpdateCenter(0.0, 0.0, 0.0);
}

void IcpViewer::keyPressEvent(QKeyEvent* event) {
  const int key = event->key();
  switch (key) {
    case Qt::Key_N:
      icp_->RunIteration();
      break;
  }
}

void IcpViewer::Paint3D() {
  gl_painter()->DrawPoints<double>(
      utils::ConstArrayView<double>(icp_->transformed_src_points().data(),
                                    icp_->transformed_src_points().cols() * 3),
      src_point_style_);

  gl_painter()->DrawPoints<double>(
      utils::ConstArrayView<double>(icp_->target_points().data(),
                                    icp_->target_points().cols() * 3),
      target_point_style_);
}
