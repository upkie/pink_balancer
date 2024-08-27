#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

"""Keep Upkie up using its wheels."""

from typing import Tuple

import gin
import numpy as np
from numpy.typing import NDArray
from upkie.utils.clamp import clamp_abs
from upkie.utils.filters import abs_bounded_derivative_filter

from .sagittal_balance import PIBalancer


@gin.configurable
class WheelBalancer:
    """
    Base class for wheel balancers.

    Attributes:
        ground_velocity: Sagittal velocity in [m] / [s].
        integral_error_velocity: Integral term contributing to the sagittal
            velocity, in [m] / [s].
        max_ground_velocity: Maximum commanded ground velocity no matter what,
            in [m] / [s].
        max_target_accel: Maximum acceleration for the ground target, in
            [m] / [s]². Does not affect the commanded ground velocity.
        max_target_velocity: Maximum velocity for the ground target, in
            [m] / [s]. Indirectly affects the commanded ground velocity.
        target_ground_velocity: Target ground sagittal velocity in [m] / [s].
        target_yaw_velocity: Target yaw velocity in [rad] / [s].
        turning_deadband: Joystick axis value between 0.0 and 1.0 below which
            legs stiffen but the turning motion doesn't start.
        turning_probability: Probability that the user wants to turn based on
            the joystick axis value.
        turning_decision_time: Minimum duration in [s] for the turning
            probability to switch from zero to one and converesly.
        wheel_radius: Wheel radius in [m].
    """

    ground_velocity: float
    integral_error_velocity: float
    max_target_accel: float
    max_target_velocity: float
    target_ground_velocity: float
    target_yaw_velocity: float
    turning_deadband: float
    turning_decision_time: float
    turning_probability: float
    wheel_radius: float

    def __init__(
        self,
        max_ground_velocity: float,
        max_target_accel: float,
        max_target_velocity: float,
        max_yaw_accel: float,
        max_yaw_velocity: float,
        turning_deadband: float,
        turning_decision_time: float,
        wheel_radius: float,
    ):
        """
        Initialize balancer.

        Args:
            max_ground_velocity: Maximum commanded ground velocity no matter
                what, in [m] / [s].
            max_target_accel: Maximum acceleration for the ground target, in
                [m] / [s]². This bound does not affect the commanded ground
                velocity.
            max_target_velocity: Maximum velocity for the ground target, in
                [m] / [s]. This bound indirectly affects the commanded ground
                velocity.
            max_yaw_accel: Maximum yaw angular acceleration in [rad] / [s]².
            max_yaw_velocity: Maximum yaw angular velocity in [rad] / [s].
            turning_deadband: Joystick axis value between 0.0 and 1.0 below
                which legs stiffen but the turning motion doesn't start.
            turning_decision_time: Minimum duration in [s] for the turning
                probability to switch from zero to one and converesly.
            wheel_radius Wheel: radius in [m].
        """
        assert 0.0 <= turning_deadband <= 1.0
        self.max_target_accel = max_target_accel
        self.max_target_velocity = max_target_velocity
        self.max_yaw_accel = max_yaw_accel
        self.max_yaw_velocity = max_yaw_velocity
        self.sagittal_balancer = PIBalancer(
            max_ground_velocity=max_ground_velocity
        )
        self.target_ground_velocity = 0.0
        self.target_yaw_velocity = 0.0
        self.turning_deadband = turning_deadband
        self.turning_decision_time = turning_decision_time
        self.turning_probability = 0.0
        self.wheel_radius = wheel_radius

    def log(self) -> dict:
        """
        Log internal state to a dictionary.

        Returns:
            Log data as a dictionary.
        """
        log_dict = self.sagittal_balancer.log()
        log_dict.update(
            {
                "target_ground_velocity": self.target_ground_velocity,
                "target_yaw_velocity": self.target_yaw_velocity,
            }
        )
        return log_dict

    def process_joystick_buttons(self, observation: dict) -> None:
        """
        Process joystick buttons.

        Args:
            observation: Latest observation.
        """

    def cycle(self, observation: dict, dt: float) -> None:
        """
        Compute a new ground velocity.

        Args:
            observation: Latest observation.
            dt: Time in [s] until next cycle.

        Returns:
            New ground velocity, in [m] / [s].
        """
        self.process_joystick_buttons(observation)
        self.update_target_ground_velocity(observation, dt)
        self.update_target_yaw_velocity(observation, dt)

        ground_velocity = self.sagittal_balancer.compute_ground_velocity(
            self.target_ground_velocity, observation, dt
        )
        left_velocity, right_velocity = self.get_wheel_velocities(
            ground_velocity,
            observation["height_controller"]["position_right_in_left"],
        )
        servo_action = {
            "left_wheel": {
                "position": np.nan,
                "velocity": left_velocity,
            },
            "right_wheel": {
                "position": np.nan,
                "velocity": right_velocity,
            },
        }
        return {"servo": servo_action}

    def get_wheel_velocities(
        self,
        ground_velocity: float,
        position_right_in_left: NDArray[float],
    ) -> Tuple[float, float]:
        """
        Get left and right wheel velocities.

        Args:
            position_right_in_left: Translation from the left contact frame
                to the right contact frame, expressed in the left contact
                frame. Equivalently, linear coordinates of the pose of the
                right contact frame with respect to the left contact frame.

        Returns:
            Tuple with `left_wheel_velocity` (left wheel velocity in rad/s) and
            `right_wheel_velocity` (right wheel velocity in rad/s).

        Note:
            For now we assume that the two wheels are parallel to the ground,
            so that the rotation from one frame to the other is the identity.
        """
        # Sagittal translation
        left_wheel_velocity: float = +ground_velocity / self.wheel_radius
        right_wheel_velocity: float = -ground_velocity / self.wheel_radius

        # Yaw rotation
        contact_radius = 0.5 * np.linalg.norm(position_right_in_left)
        yaw_to_wheel = contact_radius / self.wheel_radius
        left_wheel_velocity += yaw_to_wheel * self.target_yaw_velocity
        right_wheel_velocity += yaw_to_wheel * self.target_yaw_velocity

        return left_wheel_velocity, right_wheel_velocity

    def update_target_ground_velocity(
        self, observation: dict, dt: float
    ) -> None:
        """
        Update target ground velocity from joystick input.

        Args:
            observation: Latest observation.
            dt: Time in [s] until next cycle.

        Note:
            The target ground velocity is commanded by both the left axis and
            right trigger of the joystick. When the right trigger is unpressed,
            the commanded velocity is set from the left axis, interpolating
            from 0 to 50% of its maximum configured value. Pressing the right
            trigger increases it further up to 100% of the configured value.
        """
        try:
            axis_value = observation["joystick"]["left_axis"][1]
            trigger_value = observation["joystick"]["right_trigger"]  # -1 to 1
            boost_value = clamp_abs(0.5 * (trigger_value + 1.0), 1.0)  # 0 to 1
            max_velocity = 0.5 * (1.0 + boost_value) * self.max_target_velocity
            unfiltered_velocity = -max_velocity * axis_value
        except KeyError:
            unfiltered_velocity = 0.0
        self.target_ground_velocity = abs_bounded_derivative_filter(
            self.target_ground_velocity,
            unfiltered_velocity,
            dt,
            self.max_target_velocity,
            self.max_target_accel,
        )

    def update_target_yaw_velocity(self, observation: dict, dt: float) -> None:
        """
        Update target yaw velocity from joystick input.

        Args:
            observation: Latest observation.
            dt: Time in [s] until next cycle.
        """
        try:
            joystick_value = observation["joystick"]["right_axis"][0]
        except KeyError:
            joystick_value = 0.0
        joystick_abs = abs(joystick_value)
        joystick_sign = np.sign(joystick_value)

        turning_intent = joystick_abs / self.turning_deadband
        self.turning_probability = abs_bounded_derivative_filter(
            self.turning_probability,
            turning_intent,  # might be > 1.0
            dt,
            max_output=1.0,  # output is <= 1.0
            max_derivative=1.0 / self.turning_decision_time,
        )

        velocity_ratio = (joystick_abs - self.turning_deadband) / (
            1.0 - self.turning_deadband
        )
        velocity_ratio = max(0.0, velocity_ratio)
        velocity = self.max_yaw_velocity * joystick_sign * velocity_ratio
        turn_hasnt_started = abs(self.target_yaw_velocity) < 0.01
        turn_not_sure_yet = self.turning_probability < 0.99
        if turn_hasnt_started and turn_not_sure_yet:
            velocity = 0.0
        self.target_yaw_velocity = abs_bounded_derivative_filter(
            self.target_yaw_velocity,
            velocity,
            dt,
            self.max_yaw_velocity,
            self.max_yaw_accel,
        )
        if abs(self.target_yaw_velocity) > 0.01:  # still turning
            self.turning_probability = 1.0
