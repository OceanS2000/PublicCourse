// Copyright @2019 Pony AI Inc. All rights reserved.

#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common/proto/perception.pb.h"
#include "common/utils/common/defines.h"
#include "common/utils/common/optional.h"
#include "homework2/pointcloud.h"

class Perception {
 public:
  Perception() = default;

  interface::perception::PerceptionObstacles OneIteration(const PointCloud& pointcloud);
                                                          
  // You can feel free to add some private variables to record tracking information. You can match
  // the obstacles you gotten in the current iteration with tracks to get their velocity.
  DISALLOW_COPY_MOVE_AND_ASSIGN(Perception);
};

