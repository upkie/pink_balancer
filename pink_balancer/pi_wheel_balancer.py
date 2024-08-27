#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

import gin
import numpy as np
from upkie.exceptions import FallDetected
from upkie.utils.clamp import clamp, clamp_abs
from upkie.utils.filters import low_pass_filter

from .balancing_gains import BalancingGains
from .wheel_balancer import WheelBalancer


@gin.configurable
class PIWheelBalancer(WheelBalancer):
    """
    Balancing by proportional-integrative feedback of the base pitch error and
    ground position error to wheel velocities.

    Attributes:
        air_return_period: Cutoff period for resetting integrators while the
            robot is in the air, in [s].
        error: Two-dimensional vector of ground position and base pitch errors.
        gains: Velocity controller gains.
        max_integral_error_velocity: Maximum integral error velocity, in
            [m] / [s].
        max_target_distance: Maximum distance from the current ground position
            to the target, in [m].
        target_ground_position: Target ground sagittal position in [m].
    """

    air_return_period: float
    error: np.ndarray
    gains: BalancingGains
    max_integral_error_velocity: float
    max_target_distance: float
    target_ground_position: float

    def __init__(
        self,
        air_return_period: float,
        fall_pitch: float,
        max_integral_error_velocity: float,
        max_target_distance: float,
    ):
        """
        Initialize balancer.

        Args:
            air_return_period: Cutoff period for resetting integrators while
                the robot is in the air, in [s].
            max_integral_error_velocity: Maximum integral error velocity, in
                [m] / [s].
            max_target_distance: Maximum distance from the current ground
                position to the target, in [m].
        """
        super().__init__()
        self.air_return_period = air_return_period
        self.error = np.zeros(2)
        self.fall_pitch = fall_pitch
        self.gains = BalancingGains()
        self.max_integral_error_velocity = max_integral_error_velocity
        self.max_target_distance = max_target_distance
        self.target_ground_position = 0.0

    def process_joystick_buttons(self, observation: dict) -> None:
        """
        Process joystick buttons.

        Args:
            observation: Latest observation.
        """
        super().process_joystick_buttons(observation)
        ground_position = observation["wheel_odometry"]["position"]
        try:
            if observation["joystick"]["cross_button"]:
                # When the user presses the reset button, we assume there is no
                # contact for sure (or we are in a situation where spinning the
                # wheels is dangerous?) and thus perform a hard rather than
                # soft reset of both integrators.
                self.integral_error_velocity = 0.0  # [m] / [s]
                self.target_ground_position = ground_position
        except KeyError:
            pass

    def compute_ground_velocity(self, observation: dict, dt: float) -> float:
        """
        Compute a new ground velocity.

        Args:
            observation: Latest observation.
            dt: Time in [s] until next cycle.

        Returns:
            New ground velocity, in [m] / [s].
        """
        pitch = observation["base_orientation"]["pitch"]
        if abs(pitch) > self.fall_pitch:
            self.integral_error_velocity = 0.0  # [m] / [s]
            self.ground_velocity = 0.0  # [m] / [s]
            raise FallDetected(f"Base angle {pitch=:.3} rad denotes a fall")

        ground_position = observation["wheel_odometry"]["position"]
        floor_contact = observation["floor_contact"]["contact"]

        target_pitch: float = 0.0  # [rad]
        error = np.array(
            [
                self.target_ground_position - ground_position,
                target_pitch - pitch,
            ]
        )
        self.error = error

        if not floor_contact:
            self.integral_error_velocity = low_pass_filter(
                self.integral_error_velocity, self.air_return_period, 0.0, dt
            )
            # We don't reset self.target_ground_velocity: either takeoff
            # detection is a false positive and we should resume close to the
            # pre-takeoff state, or the robot is really in the air and the user
            # should stop smashing the joystick like a bittern ;p
            self.target_ground_position = low_pass_filter(
                self.target_ground_position,
                self.air_return_period,
                ground_position,
                dt,
            )
        else:  # floor_contact:
            ki = np.array(
                [
                    self.gains.position_stiffness,
                    self.gains.pitch_stiffness,
                ]
            )
            self.integral_error_velocity += ki.dot(error) * dt
            self.integral_error_velocity = clamp_abs(
                self.integral_error_velocity, self.max_integral_error_velocity
            )
            self.target_ground_position += self.target_ground_velocity * dt
            self.target_ground_position = clamp(
                self.target_ground_position,
                ground_position - self.max_target_distance,
                ground_position + self.max_target_distance,
            )

        kp = np.array(
            [
                self.gains.position_damping,
                self.gains.pitch_damping,
            ]
        )

        # Non-minimum phase trick: as per control theory's book, the proper
        # feedforward velocity should be ``+self.target_ground_velocity``.
        # However, it is with resolute purpose that it sends
        # ``-self.target_ground_velocity`` instead!
        #
        # Try both on the robot, you will see the difference :)
        #
        # This hack is not purely out of "esprit de contradiction". Changing
        # velocity is a non-minimum phase behavior (to accelerate forward, the
        # ZMP of the LIPM needs to move backward at first, then forward), and
        # our feedback can't realize that (it only takes care of balancing
        # around a stationary velocity).
        #
        # What's left? Our integrator! If we send the opposite of the target
        # velocity (or only a fraction of it, although 100% seems to do a good
        # job), Upkie will immediately start executing the desired non-minimum
        # phase behavior. The error will then grow and the integrator catch up
        # so that ``upkie_trick_velocity - self.integral_error_velocity``
        # converges to its proper steady state value (the same value ``0 -
        # self.integral_error_velocity`` would have converged to if we had no
        # feedforward).
        #
        # Unconvinced? Try it on the robot. You will feel Upkie's trick ;)
        #
        upkie_trick_velocity = -self.target_ground_velocity

        self.ground_velocity = (
            upkie_trick_velocity - kp.dot(error) - self.integral_error_velocity
        )
        self.ground_velocity = clamp_abs(
            self.ground_velocity, self.max_ground_velocity
        )
        return self.ground_velocity

    def log(self) -> dict:
        """
        Log internal state to a dictionary.

        Returns:
            Log data as a dictionary.
        """
        log_dict = super().log()
        log_dict.update(
            {
                "error": self.error,
                "gains": self.gains.__dict__,
                "ground_velocity": self.ground_velocity,
                "integral_error_velocity": self.integral_error_velocity,
                "target_ground_position": self.target_ground_position,
            }
        )
        return log_dict
