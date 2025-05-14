# Pink balancer

[![upkie](https://img.shields.io/badge/upkie-8.0.0-bbaacc)](https://github.com/upkie/upkie/tree/v8.0.0)

An agent for [Upkie](https://github.com/upkie/upkie/) that combines wheeled balancing with inverse kinematics computed by [Pink](https://github.com/stephane-caron/pink). This is the controller that runs in the [first](https://www.youtube.com/shorts/8b36XcCgh7s) [two](https://www.youtube.com/watch?v=NO_TkHGS0wQ) videos of Upkie.

## Installation

### On your machine

```console
conda env create -f environment.yaml
conda activate ppo_balancer
```

### On your Upkie

The PPO balancer uses [pixi](https://pixi.sh/latest/#installation) and [pixi-pack](https://github.com/Quantco/pixi-pack/releases) to pack a standalone Python environment to run policies on your Upkie. First, create `environment.tar` and upload it by:

```console
make pack_pixi_env
make upload
```

Then, unpack the remote environment:

```console
$ ssh user@your-upkie
user@your-upkie:~$ cd ppo_balancer
user@your-upkie:ppo_balancer$ make unpack_pixi_env
```

## Usage

Start the simulation or pi3hat spine, then run:

```console
make run_agent
```

### Gamepad commands

Once the agent is running, you can direct your Upkie using the game controller 🎮

- **Left joystick:** go forward right backward
- **Right joystick:** turn left or right
- **Directional pad:** down to crouch, up to stand up
- **Right button:** (B on an Xbox controller, red circle on a PS4 controller) emergency stop 🚨 all motors will turn off
