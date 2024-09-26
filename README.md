# Pink balancer

An agent for [Upkie](https://github.com/upkie/upkie/) that combines wheeled balancing with inverse kinematics computed by [Pink](https://github.com/stephane-caron/pink). This is the controller that runs in the [first](https://www.youtube.com/shorts/8b36XcCgh7s) [two](https://www.youtube.com/watch?v=NO_TkHGS0wQ) videos of Upkie.

## Usage

- Install Python packages to a conda environment: `conda env create -f environment.yaml`
- Activate conda environment: `conda activate pink_balancer`
- Run the Bullet spine: `./start_simulation.sh` (from your Upkie repository)
- Run the agent with Bullet: `python pink_balancer/main.py -c bullet`
- Upload this repository to your Upkie: `make upload`
- Run the pi3hat spine: `pi3hat_spine` (on your robot)
- Run the agent: `python pink_balancer/main.py -c $(hostname)`

## Configuration tweaks

- Select the MPC or PI sagittal balancer in `WheelController.balancer_class`
