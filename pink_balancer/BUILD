# -*- python -*-
#
# Copyright 2024 Inria

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "pink_balancer",
    main = "main.py",
    srcs = glob(["*.py"]),

    # Enable `from X import y` rather than `from agents.agent_name.X import y`
    # so that the agent can be run indifferently via Python or Bazel.
    imports = ["."],
)
