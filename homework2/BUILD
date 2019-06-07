package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud",
    srcs = ["pointcloud.cc"],
    hdrs = ["pointcloud.h"],
    deps = [
        "@eigen//:eigen",
        "@glog//:glog",
    ],
)

cc_binary(
    name = "pointcloud_reader_main",
    srcs = ["pointcloud_reader_main.cc"],
    deps = [
        ":pointcloud",
    ],
)
