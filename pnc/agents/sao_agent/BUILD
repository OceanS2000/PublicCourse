package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sao_agent",
    srcs = ["sao_agent.cc"],
    hdrs = ["sao_agent.h"],
    deps = [
        ":map_index",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_route_proto",
        "//common/proto:cc_simulation_proto",
        "//common/utils/common:optional",
        "//pnc/map:segment",
        "//pnc/simulation:vehicle_agent",
        "//pnc/simulation:vehicle_agent_factory",
        "@glog",
    ],
)

cc_library(
    name = "map_index",
    srcs = ["map_index.cc"],
    hdrs = ["map_index.h"],
    deps = [
        "//common/proto:cc_geometry_proto",
        "//common/proto:cc_map_proto",
        "//common/utils/common:optional",
        "//common/utils/index:grid_index",
        "//common/utils/knn:knn_nanoflann",
        "//pnc/map:segment",
        "@glog",
    ],
)
