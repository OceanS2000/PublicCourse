// Copyright @2018 Pony AI Inc. All rights reserved.

#include "pnc/map/map_lib.h"

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "common/utils/file/path.h"

#include <iostream>

namespace pnc {
namespace map {

MapLib::MapLib() {
  const std::string root = file::path::GetProjectRootPath();
  CHECK(file::ReadFileToProto(file::path::Join(root, "pnc/map/grid3/map_proto.txt"), &map_data_));
}

}  // namespace map
}  // namespace pnc
