# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# SPDX-License-Identifier: Apache-2.0

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "pink_balancer",
    srcs = [
        "main.py",
        "wheel_balancer.py",
        "whole_body_controller.py",
    ],
    main = "main.py",

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],

    data = [
        ":config",
    ] + select({
        "//:pi64_config": ["@upkie//spines:pi3hat_spine"],
        "//conditions:default": [],
    }),
    deps = [
        "@upkie//upkie/config",
        "@upkie//upkie/observers/base_pitch",
        "@upkie//upkie/utils:clamp",
        "@upkie//upkie/utils:exceptions",
        "@upkie//upkie/utils:filters",
        "@upkie//upkie/utils:raspi",
        "@upkie//upkie/utils:spdlog",
        "@vulp//:python",
    ],
)
