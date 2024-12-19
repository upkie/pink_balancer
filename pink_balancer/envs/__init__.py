#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

import gymnasium as gym
from upkie.envs.upkie_base_env import UpkieBaseEnv


def register() -> None:
    """!
    Register Upkie-Pinocchio environments with Gymnasium.
    """
    envs = (("UpkieCrouch", UpkieCrouch),)
    for env_name, env_class in envs:
        gym.envs.registration.register(
            id=f"{env_name}-v{env_class.version}",
            entry_point=f"upkie_pinocchio.envs:{env_name}",
        )


__all__ = [
    "UpkieCrouch",
    "register",
]
