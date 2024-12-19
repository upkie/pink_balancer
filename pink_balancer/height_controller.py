#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023-2024 Inria

import gin
import meshcat_shapes
import numpy as np
import pink
import pinocchio as pin
from numpy.typing import NDArray
from pink.tasks import FrameTask, PostureTask
from pink.utils import custom_configuration_vector


@gin.configurable
class HeightController:
    """Compute leg inverse kinematics.

    Attributes:
        knees_forward: Set to True to bend knees forward rather than backward.
        max_crouch_height: Maximum distance along the vertical axis that the
            robot goes down while crouching, in [m].
        robot: Robot model used for inverse kinematics.
        target_position_wheel_in_rest: Target position in the rest frame.
        tasks: Dictionary of inverse kinematics tasks.
        transform_rest_to_world: Rest frame pose for each end effector.
    """

    height_difference: float = 0.0
    knees_forward: bool
    max_crouch_height: float
    max_crouch_velocity: float
    max_height_difference: float
    robot: pin.RobotWrapper
    target_height: float = 0.0
    target_position_wheel_in_rest: dict[NDArray[float]]
    tasks: dict
    transform_rest_to_world: dict

    def __init__(
        self,
        robot,
        knees_forward: bool,
        max_crouch_height: float,
        max_crouch_velocity: float,
        max_height_difference: float,
        visualizer,
    ):
        """Create controller.

        Args:
            knees_forward: Set to True to bend knees forward rather than
                backward.
            max_crouch_height: Maximum distance along the vertical axis that
                the robot goes down while crouching, in [m].
            max_crouch_velocity: Maximum vertical velocity in [m] / [s].
            max_height_difference: Maximum height difference between the two
                wheel contact points, in [m].
            visualize: If true, open a MeshCat visualizer on the side.
        """
        tasks = {
            "left_contact": FrameTask(
                "left_contact",
                position_cost=[0.1, 0.0, 0.1],  # [cost] / [m]
                orientation_cost=0.0,  # [cost] / [rad]
                lm_damping=10.0,
            ),
            "right_contact": FrameTask(
                "right_contact",
                position_cost=[0.1, 0.0, 0.1],  # [cost] / [m]
                orientation_cost=0.0,  # [cost] / [rad]
                lm_damping=10.0,
            ),
            "posture": PostureTask(
                cost=1e-3,  # [cost] / [rad]
            ),
        }
        sign = -1.0 if knees_forward else +1.0
        tasks["posture"].set_target(
            custom_configuration_vector(
                robot,
                left_hip=(-sign * 0.1),
                left_knee=(+sign * 0.2),
                right_hip=(+sign * 0.1),
                right_knee=(-sign * 0.2),
            )
        )
        transform_rest_to_world = {
            "left_contact": np.zeros((4, 4)),
            "right_contact": np.zeros((4, 4)),
        }
        neutral_configuration = pink.Configuration(
            robot.model, robot.data, robot.q0
        )
        for target in ["left_contact", "right_contact"]:
            transform_target_to_world = (
                neutral_configuration.get_transform_frame_to_world(target)
            )
            tasks[target].set_target(transform_target_to_world)
            transform_rest_to_world[target] = transform_target_to_world

        if visualizer is not None:
            viewer = visualizer.viewer
            meshcat_shapes.frame(viewer["left_contact_target"], opacity=0.5)
            meshcat_shapes.frame(viewer["right_contact_target"], opacity=0.5)
            meshcat_shapes.frame(viewer["left_contact"], opacity=1.0)
            meshcat_shapes.frame(viewer["right_contact"], opacity=1.0)

        self.max_crouch_height = max_crouch_height
        self.max_crouch_velocity = max_crouch_velocity
        self.max_height_difference = max_height_difference
        self.target_position_wheel_in_rest = {
            k: np.zeros(3) for k in ("left_contact", "right_contact")
        }
        self.tasks = tasks
        self.transform_rest_to_world = transform_rest_to_world

    def update_target_height(
        self,
        dt: float,
        vertical_velocity: float,
        slant_velocity: float,
    ) -> None:
        """Update target base height from joystick inputs.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.
        """
        self.target_height = np.clip(
            a=self.target_height + vertical_velocity * dt,
            a_min=0.0,
            a_max=self.max_crouch_height,
        )
        self.height_difference = np.clip(
            a=self.height_difference + slant_velocity * dt,
            a_min=-self.max_height_difference,
            a_max=self.max_height_difference,
        )

        for wheel in ("left_contact", "right_contact"):
            offset = (
                self.height_difference
                if wheel == "right_contact"
                else -self.height_difference
            )
            offset /= 2

            # Clamp target heights
            self.target_position_wheel_in_rest[wheel][2] = np.clip(
                self.target_height + offset, 0.0, self.max_crouch_height
            )

    def update_ik_targets(self, dt: float) -> None:
        """Update IK frame targets from individual target positions.

        Args:
            dt: Duration in seconds until next cycle.
        """
        for target in ["left_contact", "right_contact"]:
            transform_common_to_rest = pin.SE3(
                rotation=np.eye(3),
                translation=self.target_position_wheel_in_rest[target],
            )
            transform_target_to_common = pin.SE3(
                rotation=np.eye(3),
                translation=np.zeros(3),
            )
            transform_target_to_world = (
                self.transform_rest_to_world[target]
                * transform_common_to_rest
                * transform_target_to_common
            )
            self.tasks[target].set_target(transform_target_to_world)

    def update_visualizer(self, visualizer):
        transform_left_to_world = self.tasks[
            "left_contact"
        ].transform_target_to_world
        transform_right_to_world = self.tasks[
            "right_contact"
        ].transform_target_to_world
        left_contact_target = transform_left_to_world.np
        right_contact_target = transform_right_to_world.np
        viewer = self.visualizer.viewer
        viewer["left_contact_target"].set_transform(left_contact_target)
        viewer["right_contact_target"].set_transform(right_contact_target)

    def cycle(self, observation: dict, dt: float) -> dict:
        """Compute leg motion by differential inverse kinematics.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action and some internal state for logging.
        """
        self.update_target_height(observation, dt)
        self.update_ik_targets(dt)
        return self.tasks.values()
