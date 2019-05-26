// Copyright @2019 Pony AI Inc. All rights reserved.

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <QtWidgets/QApplication>

#include "common/utils/display/painter_widget_base.h"
#include "homework2/icp/icp.h"

class IcpViewer : public utils::display::PainterWidgetBase {
 public:
  using Options = utils::display::PainterWidgetBase::Options;

  IcpViewer(Options options, QWidget* parent,
            const PointCloud& src_pc,
            const PointCloud& target_pc);

  ~IcpViewer() override = default;

 protected:
  // Qt event handlers.
  void timerEvent(QTimerEvent* /*event*/) override {
    update();
  }

  utils::display::OpenglPainter* gl_painter() override {
    return gl_painter_.get();
  }

  void Initialize() override {};

  void InitializeGlPainter() override;

  void keyPressEvent(QKeyEvent* event) override;

  void Paint3D() override;

 private:
  const PointCloud& src_pc_;
  const PointCloud& target_pc_;
  std::unique_ptr<Icp> icp_;

  std::unique_ptr<utils::display::OpenglPainter> gl_painter_;
  utils::display::OpenglPainter::PointStyle src_point_style_;
  utils::display::OpenglPainter::PointStyle target_point_style_;

  DISALLOW_COPY_MOVE_AND_ASSIGN(IcpViewer);
};
