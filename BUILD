# -*- python -*-
#
# Copyright 2022 Stéphane Caron

package(default_visibility = ["//visibility:public"])

config_setting(
    name = "linux",
    constraint_values = ["@platforms//os:linux"],
)

config_setting(
    name = "osx",
    constraint_values = ["@platforms//os:osx"],
)

config_setting(
    name = "pi64_config",
    values = {
        "cpu": "aarch64",
    }
)

filegroup(
    name = "all",
    srcs = [
        "@upkie//spines:bullet_spine",
        "//pink_balancer:pink_balancer"
    ],
    visibility = ["//visibility:public"]
)