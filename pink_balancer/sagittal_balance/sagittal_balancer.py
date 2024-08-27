#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

import abc


class SagittalBalancer(abc.ABC):
    @abc.abstractmethod
    def compute_ground_velocity(
        self,
        target_ground_velocity: float,
        observation: dict,
        dt: float,
    ) -> float:
        """
        Compute a new ground velocity.

        Args:
            target_ground_velocity: Target ground velocity in [m] / [s].
            observation: Latest observation dictionary.
            dt: Time in [s] until next cycle.

        Returns:
            New ground velocity, in [m] / [s].
        """

    @abc.abstractmethod
    def log(self) -> dict:
        """
        Log internal state to a dictionary.

        Returns:
            Log data as a dictionary.
        """
        return {
            "error": self.error,
            "gains": self.gains.__dict__,
            "ground_velocity": self.ground_velocity,
            "integral_error_velocity": self.integral_error_velocity,
            "target_ground_position": self.target_ground_position,
        }
