#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023-2024 Inria


class UpkieCrouch(UpkieBaseEnv):
    """Upkie environment with the ability to crouch.

    Attributes:
        action_space: Action space.
        left_wheeled: Set to True (default) if the robot is left wheeled, that
            is, a positive turn of the left wheel results in forward motion.
            Set to False for a right-wheeled variant.
        observation_space: Observation space.
        version: Version number.
        wheel_radius: Wheel radius in [m].
    """

    action_space: gym.spaces.Box
    left_wheeled: bool
    observation_space: gym.spaces.Box
    version = 1
    gain_scale: float
    turning_gain_scale: float
    wheel_radius: float

    def __init__(
        self,
        gain_scale: float,
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

        \param fall_pitch Fall detection pitch angle, in radians.
        \param frequency Regulated frequency of the control loop, in Hz.
        \param frequency_checks If `regulate_frequency` is set and this
            parameter is true (default), a warning is issued every time the
            control loop runs slower than the desired `frequency`. Set this
            parameter to false to disable these warnings.
        \param init_state Initial state of the robot, only used in simulation.
        \param left_wheeled Set to True (default) if the robot is left wheeled,
            that is, a positive turn of the left wheel results in forward
            motion. Set to False for a right-wheeled variant.
        \param max_ground_velocity Maximum commanded ground velocity in m/s.
            The default value of 1 m/s is conservative, don't hesitate to
            increase it once you feel confident in your agent.
        \param regulate_frequency Enables loop frequency regulation.
        \param shm_name Name of shared-memory file.
        \param spine_config Additional spine configuration overriding the
            default `upkie.config.SPINE_CONFIG`. The combined configuration
            dictionary is sent to the spine at every reset.
        \param wheel_radius Wheel radius in [m].
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

        self.left_wheeled = left_wheeled
        self.wheel_radius = wheel_radius
        self.gain_scale = np.clip(gain_scale, 0.1, 2.0)
        self.height_controller = HeightController(visualize=visualize)
        self.turning_gain_scale = turning_gain_scale
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
        leg_action = self.height_controller.cycle(observation, dt)
        wheel_action = self.wheel_controller.cycle(observation, dt)
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

        spine_action = {
            "servo": servo_action,
            "height_controller": self.height_controller.log(),
            "wheel_controller": self.wheel_controller.log(),
        }
        return spine_action
