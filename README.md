# Pink balancer

An agent for [Upkie](https://github.com/upkie/upkie/) that combines wheeled balancing with inverse kinematics computed by [Pink](https://github.com/stephane-caron/pink). This is the controller that runs in the [first](https://www.youtube.com/shorts/8b36XcCgh7s) [two](https://www.youtube.com/watch?v=NO_TkHGS0wQ) videos of Upkie.

## Run in simulation

- Run the Bullet spine: `./start_simulation.sh`
- Run the agent: `make run_bullet`

## Run on the real robot

- Build spines and agent: ``make build``
- Upload to your Upkie: ``make upload``
- SSH into your Upkie
- Run the pi3hat spine: ``make run_pi3hat_spine``
- Run the agent: ``make run_pink_balancer``
