#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023-2024 Inria

from typing import Optional

import gin
import numpy as np
import pink
import pinocchio as pin
import upkie_description
from jump_playback import JumpPlayback
from numpy.typing import NDArray
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask
from pink.utils import custom_configuration_vector
from pink.visualization import start_meshcat_visualizer
from upkie.utils.clamp import clamp


def observe_configuration(
    observation, configuration, servo_layout
) -> NDArray[float]:
    """
    Compute configuration vector from a new observation.

    Args:
        observation: Observation dictionary.
        configuration: Previous configuration.
        servo_layout: Robot servo layout.

    Returns:
        Configuration vector from observation.
    """
    q = configuration.q.copy()
    for joint, servo in servo_layout.items():
        if "configuration_index" not in servo:
            continue
        i = servo["configuration_index"]
        q[i] = observation["servo"][joint]["position"]
    return q


def serialize_to_servo_action(configuration, velocity, servo_layout) -> dict:
    """
    Serialize robot state for the spine.

    Args:
        configuration: Robot configuration.
        velocity: Robot velocity in tangent space.
        servo_layout: Robot servo layout.

    Returns:
        Dictionary of position and velocity targets for each joint.
    """
    target = {}
    model = configuration.model
    tau_max = model.effortLimit
    for joint_name, servo in servo_layout.items():
        joint_id = model.getJointId(joint_name)
        joint = model.joints[joint_id]
        target[joint_name] = {
            "position": configuration.q[joint.idx_q],
            "velocity": velocity[joint.idx_v],
            "maximum_torque": tau_max[joint.idx_v],
        }
    return target


def add_target_frames(visualizer):
    import meshcat_shapes

    viewer = visualizer.viewer
    meshcat_shapes.frame(viewer["left_contact_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["right_contact_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["left_contact"], opacity=1.0)
    meshcat_shapes.frame(viewer["right_contact"], opacity=1.0)


@gin.configurable
class HeightController:
    """
    Compute leg inverse kinematics.

    Attributes:
        max_crouch_height: Maximum distance along the vertical axis that the
            robot goes down while crouching, in [m].
        robot: Robot model used for inverse kinematics.
        target_position_wheel_in_rest: Target position in the rest frame.
        tasks: Dictionary of inverse kinematics tasks.
        transform_rest_to_world: Rest frame pose for each end effector.
    """

    max_crouch_height: float
    max_crouch_velocity: float
    robot: pin.RobotWrapper
    target_position_wheel_in_rest: NDArray[float]
    tasks: dict
    transform_rest_to_world: dict

    def __init__(
        self,
        max_crouch_height: float,
        max_crouch_velocity: float,
        visualize: bool,
    ):
        """
        Create controller.

        Args:
            max_crouch_height: Maximum distance along the vertical axis that
                the robot goes down while crouching, in [m].
            max_crouch_velocity: Maximum vertical velocity in [m] / [s].
            visualize: If true, open a MeshCat visualizer on the side.
        """
        robot = upkie_description.load_in_pinocchio(root_joint=None)
        configuration = pink.Configuration(robot.model, robot.data, robot.q0)
        servo_layout = {
            "left_hip": {
                "bus": 2,
                "configuration_index": 0,
                "id": 4,
            },
            "left_knee": {
                "bus": 2,
                "configuration_index": 1,
                "id": 5,
            },
            "left_wheel": {
                "bus": 2,
                "id": 6,
            },
            "right_hip": {
                "bus": 1,
                "configuration_index": 3,
                "id": 1,
            },
            "right_knee": {
                "bus": 1,
                "configuration_index": 4,
                "id": 2,
            },
            "right_wheel": {
                "bus": 1,
                "id": 3,
            },
        }
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
        tasks["posture"].set_target(
            custom_configuration_vector(robot, left_knee=0.2, right_knee=-0.2)
        )

        visualizer = None
        if visualize:
            visualizer = start_meshcat_visualizer(robot)
            add_target_frames(visualizer)

        self.__initialized = False
        self.configuration = configuration
        self.jump_playback = None
        self.last_velocity = np.zeros(robot.nv)
        self.max_crouch_height = max_crouch_height
        self.max_crouch_velocity = max_crouch_velocity
        self.robot = robot
        self.servo_layout = servo_layout
        self.target_position_wheel_in_rest = np.zeros(3)
        self.target_offset = {
            "left_contact": np.zeros(3),
            "right_contact": np.zeros(3),
        }
        self.tasks = tasks
        self.transform_rest_to_world = {
            "left_contact": np.zeros((4, 4)),
            "right_contact": np.zeros((4, 4)),
        }
        self.visualizer = visualizer

    def get_next_height_from_joystick(
        self, observation: dict, dt: float
    ) -> float:
        """
        Update target base height from joystick inputs.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.

        Returns:
            New height target, in meters.
        """
        try:
            axis_value: float = observation["joystick"]["pad_axis"][1]
            velocity = self.max_crouch_velocity * axis_value
        except KeyError:
            velocity = 0.0
        height = self.target_position_wheel_in_rest[2]
        height += velocity * dt
        return height

    def get_next_height_from_playback(
        self, observation: dict, dt: float
    ) -> Optional[float]:
        try:
            square_pressed = observation["joystick"]["square_button"]
            if self.jump_playback is None and square_pressed:
                initial_height = self.target_position_wheel_in_rest[2]
                self.jump_playback = JumpPlayback(initial_height)
        except KeyError:
            pass
        height = None
        if self.jump_playback is not None:
            height = self.jump_playback.cycle(dt)
            if self.jump_playback.is_over:
                self.jump_playback = None
        return height

    def update_target_height(self, observation: dict, dt: float) -> None:
        """
        Update target base height from joystick inputs.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.
        """
        playback_height = self.get_next_height_from_playback(observation, dt)
        height = (
            playback_height
            if playback_height is not None
            else self.get_next_height_from_joystick(observation, dt)
        )
        self.target_position_wheel_in_rest[2] = clamp(
            height, 0.0, self.max_crouch_height
        )

    def update_ik_targets(self, observation: dict, dt: float) -> None:
        """
        Update IK frame targets from individual target positions.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.
        """
        transform_common_to_rest = pin.SE3(
            rotation=np.eye(3),
            translation=self.target_position_wheel_in_rest,
        )
        for target in ["left_contact", "right_contact"]:
            transform_target_to_common = pin.SE3(
                rotation=np.eye(3),
                translation=self.target_offset[target],
            )
            transform_target_to_world = (
                self.transform_rest_to_world[target]
                * transform_common_to_rest
                * transform_target_to_common
            )
            self.tasks[target].set_target(transform_target_to_world)

    def _process_first_observation(self, observation: dict) -> None:
        """
        Function called at the first iteration of the controller.

        Args:
            observation: Observation from the spine.
        """
        q = observe_configuration(
            observation, self.configuration, self.servo_layout
        )
        self.configuration = pink.Configuration(
            self.robot.model, self.robot.data, q
        )
        for target in ["left_contact", "right_contact"]:
            transform_target_to_world = (
                self.configuration.get_transform_frame_to_world(target)
            )
            self.tasks[target].set_target(transform_target_to_world)
            self.transform_rest_to_world[target] = transform_target_to_world
        self.__initialized = True

    def _observe_ground_positions(self, observation: dict) -> None:
        """Observe the transform from right to left ground frames."""
        transform_left_to_world = self.tasks[
            "left_contact"
        ].transform_target_to_world
        transform_right_to_world = self.tasks[
            "right_contact"
        ].transform_target_to_world
        transform_right_to_left = transform_left_to_world.actInv(
            transform_right_to_world
        )
        observation["height_controller"] = {
            "position_right_in_left": transform_right_to_left.translation,
        }

        if self.visualizer is not None:
            self.visualizer.display(self.configuration.q)
            viewer = self.visualizer.viewer
            viewer["left_contact_target"].set_transform(
                transform_left_to_world.np
            )
            viewer["right_contact_target"].set_transform(
                transform_right_to_world.np
            )

    def log(self) -> dict:
        """
        Log internal state to a dictionary.

        Returns:
            Log data as a dictionary.
        """
        return {
            "configuration": self.configuration.q,
            "target_height": self.target_position_wheel_in_rest[2],
            "velocity": self.last_velocity,
        }

    def cycle(self, observation: dict, dt: float) -> dict:
        """
        Compute action for a new cycle.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action and some internal state for logging.
        """
        if not self.__initialized:
            self._process_first_observation(observation)

        self.update_target_height(observation, dt)
        self.update_ik_targets(observation, dt)
        velocity = solve_ik(
            self.configuration,
            self.tasks.values(),
            dt,
            solver="proxqp",
        )
        self.configuration.integrate_inplace(velocity, dt)
        self.last_velocity = velocity

        self._observe_ground_positions(observation)
        servo_action = serialize_to_servo_action(
            self.configuration, velocity, self.servo_layout
        )
        action = {"servo": servo_action}
        return action
