package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_visualizer_main",
    srcs = ["map_visualizer_main.cc"],
    deps = [
        "//common/utils/file",
        "//homework4/display:main_window",
        "//homework4/map:map_lib",
        "@gflags",
        "@glog",
    ],
)

cc_binary(
    name = "map_filler",
    srcs = ["map_filler.cc"],
    deps = [
        "//common/proto:cc_map_proto",
        "//common/utils/file",
        "//common/utils/knn:knn_nanoflann",
        "@eigen//:eigen",
        "@gflags",
        "@glog",
    ],
)

cc_binary(
    name = "map_router",
    srcs = ["map_router.cc"],
    deps = [
        "//common/proto:cc_map_proto",
        "//common/proto:cc_route_proto",
        "//common/proto:cc_geometry_proto",
        "//common/utils/file",
        "//common/utils/knn:knn_nanoflann",
        "//homework4/map:map_lib",
        "@eigen//:eigen",
        "@gflags",
        "@glog",
    ],
)
