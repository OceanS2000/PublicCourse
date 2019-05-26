package(default_visibility = ["//visibility:public"])

cc_library(
    name = "icp",
    srcs = ["icp.cc"],
    hdrs = ["icp.h"],
    deps = [
        "//common/utils/knn:knn_nanoflann",
        "//common/utils/math",
        "//common/utils/math/transform",
        "//homework2:pointcloud",
        "@eigen//:eigen",
        "@glog//:glog",
    ],
)

cc_library(
    name = "icp_viewer",
    srcs = ["icp_viewer.cc"],
    hdrs = ["icp_viewer.h"],
    deps = [
        ":icp",
        "//common/utils/display:painter_widget_base",
        "@glog//:glog",
        "@qt//:qt_widgets",
    ],
)

cc_binary(
    name = "icp_viewer_main",
    srcs = ["icp_viewer_main.cc"],
    deps = [
        ":icp_viewer",
    ],
)
