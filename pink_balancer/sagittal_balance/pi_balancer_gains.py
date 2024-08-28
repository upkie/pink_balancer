#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023-2024 Inria

import gin


@gin.configurable
class PIBalancerGains:
    """PI balancer gains."""

    pitch_damping: float
    pitch_stiffness: float
    position_damping: float
    position_stiffness: float

    def __init__(
        self,
        pitch_damping: float,
        pitch_stiffness: float,
        position_damping: float,
        position_stiffness: float,
    ):
        """Initialize a new set of gains.

        Args:
            pitch_damping: Pitch damping gain.
            pitch_stiffness: Pitch stiffness gain.
            position_damping: Position damping gain.
            position_stiffness: Position stiffness gain.
        """
        self.pitch_damping = pitch_damping
        self.pitch_stiffness = pitch_stiffness
        self.position_damping = position_damping
        self.position_stiffness = position_stiffness

    def set(
        self,
        pitch_damping: float,
        pitch_stiffness: float,
        position_damping: float,
        position_stiffness: float,
    ) -> None:
        """Set gains in one function call.

        Args:
            pitch_damping: Pitch error (normalized) damping gain.
                Corresponds to the proportional term of the velocity PI
                controller, equivalent to the derivative term of the
                acceleration PD controller.
            pitch_stiffness: Pitch error (normalized) stiffness gain.
                Corresponds to the integral term of the velocity PI
                controller, equivalent to the proportional term of the
                acceleration PD controller.
            position_damping: Position error (normalized) damping gain.
                Corresponds to the proportional term of the velocity PI
                controller, equivalent to the derivative term of the
                acceleration PD controller.
            position_stiffness: Position error (normalized) stiffness gain.
                Corresponds to the integral term of the velocity PI
                controller, equivalent to the proportional term of the
                acceleration PD controller.
        """
        self.pitch_damping = pitch_damping
        self.pitch_stiffness = pitch_stiffness
        self.position_damping = position_damping
        self.position_stiffness = position_stiffness

    def __repr__(self):
        """Descriptive string representation."""
        return (
            "PIBalancerGains("
            f"pitch_damping={self.pitch_damping}, "
            f"pitch_stiffness={self.pitch_stiffness}, "
            f"position_damping={self.position_damping}, "
            f"position_stiffness={self.position_stiffness}, "
        )
