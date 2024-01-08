# -*- python -*-
#
# Copyright 2022 Stéphane Caron

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def upkie_repository():
    """
    Clone repository from GitHub and make its targets available for binding.
    """
    git_repository(
        name = "upkie",
        remote = "https://github.com/tasts-robots/upkie.git",
        commit = "3bf4c14fb30448c8f7cc29e2c5d565be854769e1",
        shallow_since = "1698070414 +0200",
    )
