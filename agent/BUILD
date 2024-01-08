# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("@pip_upkie//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "agent",
    srcs = ["agent.py"],

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],

    deps = [
        "@upkie//upkie/envs",
        "@upkie//upkie/utils:raspi",
        requirement("loop_rate_limiters"),
    ],
)
