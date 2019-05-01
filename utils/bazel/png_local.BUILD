licenses(["notice"])  # Zlib/libpng(BSD/MIT-like)

package(default_visibility = ["//visibility:private"])

cc_library(
    name = "png",
    srcs = [
        "lib/libpng.so"
    ],
    hdrs = [
        "include/png.h",
        "include/pngconf.h",
    ],
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@zlib//:zlib",
    ],
)