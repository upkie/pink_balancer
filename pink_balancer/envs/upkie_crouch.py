#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023-2024 Inria

from typing import Dict, Optional, Tuple

import gymnasium as gym
import numpy as np
import pink
import upkie_description
from pink import solve_ik
from pink.visualization import start_meshcat_visualizer
from upkie.envs import UpkieBaseEnv
from upkie.exceptions import UpkieException
from upkie.utils.robot_state import RobotState

from .utils import abs_bounded_derivative_filter


def observe_configuration(
    observation, configuration, servo_layout
) -> np.ndarray:
    """Compute configuration vector from a new observation.

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
    """Serialize robot state for the spine.

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
    for joint_name in servo_layout.keys():
        joint_id = model.getJointId(joint_name)
        joint = model.joints[joint_id]
        target[joint_name] = {
            "position": configuration.q[joint.idx_q],
            "velocity": velocity[joint.idx_v],
            "maximum_torque": tau_max[joint.idx_v],
        }
    return target


class UpkieCrouch(UpkieBaseEnv):
    """Upkie environment with the ability to crouch.

    Attributes:
        action_space: Action space.
        left_wheeled: Set to True (default) if the robot is left wheeled, that
            is, a positive turn of the left wheel results in forward motion.
            Set to False for a right-wheeled variant.
        max_init_joint_velocity: Maximum joint velocity during the initial
            phase, in [rad] / [s].
        observation_space: Observation space.
        version: Version number.
        wheel_radius: Wheel radius in [m].
    """

    action_space: gym.spaces.Box
    left_wheeled: bool
    observation_space: gym.spaces.Box
    version = 1
    max_init_joint_velocity: float
    gain_scale: float
    turning_gain_scale: float
    wheel_radius: float

    def __init__(
        self,
        gain_scale: float,
        visualize: bool,
        max_init_joint_velocity: float,
        turning_gain_scale: float,
        fall_pitch: float = 1.0,
        frequency: float = 200.0,
        frequency_checks: bool = True,
        init_state: Optional[RobotState] = None,
        left_wheeled: bool = True,
        max_ground_velocity: float = 1.0,
        regulate_frequency: bool = True,
        shm_name: str = "/upkie",
        spine_config: Optional[dict] = None,
        wheel_radius: float = 0.06,
    ):
        """Initialize environment.

        Args:
            fall_pitch: Fall detection pitch angle, in radians.
            frequency: Regulated frequency of the control loop, in Hz.
            frequency_checks: If `regulate_frequency` is set and this
                parameter is true (default), a warning is issued every time the
                control loop runs slower than the desired `frequency`. Set this
                parameter to false to disable these warnings.
            init_state: Initial state of the robot, only used in simulation.
            left_wheeled: Set to True (default) if the robot is left wheeled,
                that is, a positive turn of the left wheel results in forward
                motion. Set to False for a right-wheeled variant.
            max_ground_velocity: Maximum commanded ground velocity in m/s.
                The default value of 1 m/s is conservative, don't hesitate to
                increase it once you feel confident in your agent.
                regulate_frequency Enables loop frequency regulation.
            max_init_joint_velocity: Maximum joint velocity during the initial
                phase, in [rad] / [s].
            shm_name: Name of shared-memory file.
            spine_config: Additional spine configuration overriding the
                default `upkie.config.SPINE_CONFIG`. The combined configuration
                dictionary is sent to the spine at every reset.
            wheel_radius: Wheel radius in [m].
        """
        super().__init__(
            fall_pitch=fall_pitch,
            frequency=frequency,
            frequency_checks=frequency_checks,
            init_state=init_state,
            regulate_frequency=regulate_frequency,
            shm_name=shm_name,
            spine_config=spine_config,
        )

        if self.dt is None:
            raise UpkieException("This environment needs a loop frequency")

        # gymnasium.Env: observation_space
        MAX_BASE_PITCH: float = np.pi
        MAX_GROUND_POSITION: float = float("inf")
        MAX_BASE_ANGULAR_VELOCITY: float = 1000.0  # rad/s
        observation_limit = np.array(
            [
                MAX_BASE_PITCH,
                MAX_GROUND_POSITION,
                MAX_BASE_ANGULAR_VELOCITY,
                max_ground_velocity,
            ],
            dtype=float,
        )
        self.observation_space = gym.spaces.Box(
            -observation_limit,
            +observation_limit,
            shape=observation_limit.shape,
            dtype=observation_limit.dtype,
        )

        # gymnasium.Env: action_space
        action_limit = np.array([max_ground_velocity], dtype=float)
        self.action_space = gym.spaces.Box(
            -action_limit,
            +action_limit,
            shape=action_limit.shape,
            dtype=action_limit.dtype,
        )

        robot = upkie_description.load_in_pinocchio(root_joint=None)
        neutral_configuration = pink.Configuration(
            robot.model, robot.data, robot.q0
        )
        visualizer = start_meshcat_visualizer(robot) if visualize else None

        leg_controller = LegController(robot)

        self.__initialized = False
        self.configuration = neutral_configuration
        self.left_wheeled = left_wheeled
        self.wheel_radius = wheel_radius
        self.max_init_joint_velocity = max_init_joint_velocity
        self.gain_scale = np.clip(gain_scale, 0.1, 2.0)
        self.leg_controller = leg_controller
        self.turning_gain_scale = turning_gain_scale
        self.robot = robot
        self.wheel_controller = WheelController()

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[np.ndarray, Dict]:
        r"""!
        Resets the environment and get an initial observation.

        \param seed Number used to initialize the environment’s internal random
            number generator.
        \param options Currently unused.
        \return
            - `observation`: Initial vectorized observation, i.e. an element
              of the environment's `observation_space`.
            - `info`: Dictionary with auxiliary diagnostic information. For
              Upkie this is the full observation dictionary sent by the spine.
        """
        observation, info = super().reset(seed=seed, options=options)
        spine_observation = info["spine_observation"]
        for joint in self.model.upper_leg_joints:
            position = spine_observation["servo"][joint.name]["position"]
            self.__leg_servo_action[joint.name]["position"] = position
        return observation, info

    def get_env_observation(self, spine_observation: dict) -> np.ndarray:
        r"""!
        Extract environment observation from spine observation dictionary.

        \param spine_observation Spine observation dictionary.
        \return Environment observation vector.
        """
        base_orientation = spine_observation["base_orientation"]
        pitch_base_in_world = base_orientation["pitch"]
        angular_velocity_base_in_base = base_orientation["angular_velocity"]
        ground_position = spine_observation["wheel_odometry"]["position"]
        ground_velocity = spine_observation["wheel_odometry"]["velocity"]

        obs = np.empty(4, dtype=float)
        obs[0] = pitch_base_in_world
        obs[1] = ground_position
        obs[2] = angular_velocity_base_in_base[1]
        obs[3] = ground_velocity
        return obs

    def get_spine_action(self, action: np.ndarray) -> dict:
        r"""!
        Convert environment action to a spine action dictionary.

        \param action Environment action.
        \return Spine action dictionary.
        """
        if not self.__initialized:
            servo_action = self.get_init_servo_action(observation, dt)
            spine_action = {"servo": servo_action}
            return spine_action

        # Step the IK
        tasks = self.leg_controller.cycle(observation, dt)
        velocity = solve_ik(
            self.ik_configuration,
            tasks,
            self.dt,
            solver="proxqp",
        )
        self.configuration.integrate_inplace(velocity, dt)

        wheel_action = self.wheel_controller.cycle(observation, dt)

        if self.visualizer is not None:
            self.visualizer.display(self.configuration.q)
            self.leg_controller.update_visualizer(visualizer)

        servo_action = {
            "left_hip": leg_action["servo"]["left_hip"],
            "left_knee": leg_action["servo"]["left_knee"],
            "left_wheel": wheel_action["servo"]["left_wheel"],
            "right_hip": leg_action["servo"]["right_hip"],
            "right_knee": leg_action["servo"]["right_knee"],
            "right_wheel": wheel_action["servo"]["right_wheel"],
        }
        turning_prob = self.wheel_controller.turning_probability
        kp_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        kd_scale = self.gain_scale + self.turning_gain_scale * turning_prob
        for joint_name in ["left_hip", "left_knee", "right_hip", "right_knee"]:
            servo_action[joint_name]["kp_scale"] = kp_scale
            servo_action[joint_name]["kd_scale"] = kd_scale

        spine_action = {"servo": servo_action}
        return spine_action

    def get_init_servo_action(self, observation: dict, dt: float) -> dict:
        """Initial phase where we return legs to the neutral configuration.

        Args:
            observation: Observation from the spine.
            dt: Duration in seconds until next cycle.

        Returns:
            Dictionary with the new action.
        """
        if self.q_init is None:
            self.q_init = observe_configuration(
                observation, self.configuration, self.servo_layout
            )
        # Applying this filter is ok as long as our root_joint is None
        self.q_init = abs_bounded_derivative_filter(
            prev_output=self.q_init,
            new_input=self.configuration.q,
            dt=dt,
            max_derivative=np.full(
                (self.robot.nv,), self.max_init_joint_velocity
            ),
        )
        # Difference is also OK, configuration space is a vector space
        q_diff = self.q_init - self.configuration.q
        if np.linalg.norm(q_diff, ord=1) < 1e-5:
            logging.info("Upkie initialized to the neutral configuration")
            self.__initialized = True

        return_configuration = pink.Configuration(
            self.robot.model, self.robot.data, self.q_init
        )
        return_velocity = np.zeros(self.robot.nv)
        return serialize_to_servo_action(
            return_configuration, return_velocity, self.servo_layout
        )
