# -*- python -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

load("@rules_python//python:pip.bzl", "pip_parse")

def parse_deps():
    """
    Parse PyPI packages to a dedicated external repository.

    This function intended to be loaded and called from your WORKSPACE.
    """
    pip_parse(
        name = "pip_upkie_template",
        requirements_lock = Label("//tools/workspace/pip_upkie_template:requirements_lock.txt"),
    )
