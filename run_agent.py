#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

import argparse
import socket
import traceback
from math import pi
from pathlib import Path

import gin
import yaml

from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation

from pink_balancer import WholeBodyController
from upkie.exceptions import FallDetected
from upkie.spine import SpineInterface
from upkie.utils.raspi import configure_agent_process, on_raspi
from upkie.utils.spdlog import logging


def get_vertical_force(
    step: int,
    start: int = 200,
    lift_steps: int = 200,
    delta: float = 0.1,
    hold_steps: int = 400,
) -> float:
    lift: float = 0.0  # 0 = no force, 1 => apply -mg
    if step < start:
        lift = 0.0
    elif step - start < lift_steps // 2:
        lift = 1.0 + delta
    elif step - start < lift_steps:
        lift = 1.0 - delta
    elif step - start - lift_steps < hold_steps:
        lift = 1.0
    else:
        lift = 0.0
    mass = 5.34  # approximative, in [kg]
    return lift * mass * 9.81  # in [N]


def parse_command_line_arguments() -> argparse.Namespace:
    """Parse command line arguments.

    Returns:
        Command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-c",
        "--config",
        metavar="config",
        help="Additional agent configuration to apply",
        type=str,
    )
    parser.add_argument(
        "--visualize",
        help="Publish robot visualization to MeshCat for debugging",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "--levitate",
        help="Levitate the robot by applying a vertical force",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "--forward",
        help="Make robot go forward",
        default=True,
        action="store_true",
    )
    return parser.parse_args()


def run(
    spine: SpineInterface,
    spine_config: dict,
    controller: WholeBodyController,
    frequency: float = 200.0,
    levitate: bool = False,
    forward: bool = False,
) -> None:
    """Read observations and send actions to the spine.

    Args:
        spine: Interface to the spine.
        spine_config: Spine configuration dictionary.
        controller: Whole-body controller.
        frequency: Control frequency in Hz.
    """
    dt = 1.0 / frequency
    rate = RateLimiter(frequency, "controller")

    if levitate:
        torso_force_in_world = [0.0, 0.0, 0.0]
        bullet_action = {
            "external_forces": {
                "torso": {
                    "force": torso_force_in_world,
                    "local": False,
                }
            }
        }

    spine.start(spine_config)

    step: int = 0
    observation = spine.get_observation()  # pre-reset observation

    observation["joystick"] = {""}

    
    while True:
        observation = spine.get_observation()

        if forward: 
            controller.height_controller.target_height = 0.05 # max_crouch_height: 0.08
            controller.wheel_controller.target_ground_velocity = 0.3

            # Simple P-controller to make sure we are always
            # facing the stairs (which corresponds to a 0-yaw value)
            current_orientation = observation["imu"]["orientation"]
            current_yaw = Rotation.from_quat(current_orientation).as_euler("xyz")[0] * 180 / pi
            print(f"current_yaw: {current_yaw.round(1)}")
            target_yaw = 0.0
            controller.wheel_controller.target_yaw_velocity = - 0.1 * (
                target_yaw - current_yaw
            )

        if step % 1000 and levitate:
            observation["joystick"] = {"cross_button": True} # trigger reset

        action = controller.cycle(observation, dt)

        if levitate:
            torso_force_in_world[2] = get_vertical_force(step % 1000)
            action["bullet"] = bullet_action
        
        spine.set_action(action)
        step += 1
        rate.sleep()


if __name__ == "__main__":
    args = parse_command_line_arguments()

    # Agent configuration
    hostname = socket.gethostname()
    config_dir = Path(__file__).parent / "config"
    gin.parse_config_file(config_dir / "base.gin")
    host_config = config_dir / f"{hostname}.gin"
    if host_config.exists():
        gin.parse_config_file(host_config)
    if args.config is not None:
        gin.parse_config_file(config_dir / f"{args.config}.gin")

    # On Raspberry Pi, configure the process to run on a separate CPU core
    if on_raspi():
        configure_agent_process()

    with open(config_dir / "spine.yaml", "r") as file:
        spine_config = yaml.safe_load(file)

    controller = WholeBodyController(visualize=args.visualize)
    wheel_radius = controller.wheel_controller.wheel_radius
    wheel_odometry_config = spine_config["wheel_odometry"]
    wheel_odometry_config["signed_radius"]["left_wheel"] = +wheel_radius
    wheel_odometry_config["signed_radius"]["right_wheel"] = -wheel_radius
    spine = SpineInterface(retries=10)

    keep_trying = True
    while keep_trying:
        keep_trying = False
        try:
            run(spine, spine_config, controller, levitate=args.levitate, forward=args.forward)
        except KeyboardInterrupt:
            logging.info("Caught a keyboard interrupt")
        except FallDetected:
            logging.error("Fall detected, resetting the spine!")
            spine.stop()
            controller = WholeBodyController(visualize=args.visualize)
            spine = SpineInterface(retries=10)
            keep_trying = True
        except Exception:
            logging.error("Controller raised an exception!")
            print("")
            traceback.print_exc()
            print("")

    logging.info("Stopping the spine...")
    try:
        spine.stop()
    except Exception:
        logging.error("Error while stopping the spine!")
        print("")
        traceback.print_exc()
        print("")
