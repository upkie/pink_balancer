#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

import argparse
import socket
import traceback
from pathlib import Path

import gin
import upkie.config
from loop_rate_limiters import RateLimiter
from upkie.spine import SpineInterface
from upkie.utils.raspi import configure_agent_process, on_raspi
from upkie.utils.spdlog import logging

from pink_balancer import WholeBodyController


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
    return parser.parse_args()


def run(
    spine: SpineInterface,
    spine_config: dict,
    controller: WholeBodyController,
    frequency: float = 200.0,
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

    spine.start(spine_config)
    observation = spine.get_observation()  # pre-reset observation
    while True:
        observation = spine.get_observation()
        action = controller.cycle(observation, dt)
        spine.set_action(action)
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

    spine = SpineInterface(retries=10)
    controller = WholeBodyController(visualize=args.visualize)
    spine_config = upkie.config.SPINE_CONFIG.copy()
    wheel_radius = controller.wheel_controller.wheel_radius
    wheel_odometry_config = spine_config["wheel_odometry"]
    wheel_odometry_config["signed_radius"]["left_wheel"] = +wheel_radius
    wheel_odometry_config["signed_radius"]["right_wheel"] = -wheel_radius
    try:
        run(spine, spine_config, controller)
    except KeyboardInterrupt:
        logging.info("Caught a keyboard interrupt")
    except Exception:
        logging.error("Controller raised an exception")
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
