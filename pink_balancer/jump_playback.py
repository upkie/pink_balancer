#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

import gin


@gin.configurable
class JumpPlayback:
    def __init__(
        self,
        initial_height: float,
        duration: float,
        cooldown: float,
    ):
        """
        Jump open-loop trajectory playback.

        Args:
            initial_height: Initial foot height (increased by crouching).
            duration: Jump duration in seconds.
            cooldown: Cooldown duration in seconds.
        """
        self.cooldown = cooldown
        self.duration = duration
        self.initial_height = initial_height
        self.t = 0.0

    @property
    def is_over(self) -> bool:
        return self.t >= self.duration + self.cooldown

    def cycle(self, dt: float) -> float:
        self.t += dt
        height = self.initial_height * max(0.0, 1.0 - self.t / self.duration)
        print(f"jump playback now at {self.t=}, {height=}")
        return height
