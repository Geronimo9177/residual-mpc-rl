# Residual MPC-RL

This repository contains the simulation and control framework for model-based
path tracking over localized terrain irregularities.

The associated paper is:

> **Recurrent Residual Reinforcement Learning for MPC-Based Path Tracking over
> Localized Terrain Irregularities**

The paper is currently **under ICRA review**.

## Demonstration video

[![Demonstration video](https://img.youtube.com/vi/eCw2W8Uuggc/hqdefault.jpg)](https://youtu.be/eCw2W8Uuggc)

## Branches

| Branch | Description |
|---|---|
| [`main`](../../tree/main) (this branch) | Kinematic MPC and camera-based rule-based MPC baselines for a Tesla Model 3 in Webots. |
| [`webots_PPO_residual`](../../tree/webots_PPO_residual) | Residual reinforcement-learning implementation for the feed-forward residual variant (FF-RRL) and Recurrent Residual RL (3ReL). |

## Repository contents

The workspace contains two ROS 2 packages.

### `vehicle_webots`

This package contains the vehicle simulation and Webots integration:

- Tesla Model 3 model with Ackermann steering geometry;
- Baylands world with roads, curves, and 21 speed-bump locations;
- odometry, ground-truth, IMU, GPS, segmentation-camera, wheel-state, and TF
  publication;
- reception of the final vehicle command on `/cmd_vel`;
- optional episode synchronization through `/sim/start`, `/sim/reset`, and
  `/sim/trigger`;
- `SpeedBumpSupervisor`, which can hide four speed bumps during training
  episodes.

### `vehicle_controller`

This package contains the controllers and planning nodes:

- `ackermann_mpc`: kinematic-bicycle MPC;
- `topp`: trajectory reconstruction and feasible speed-profile generation;
- `episode_manager`: trajectory selection and episode lifecycle management;
- `mpc_debug_visualizer`: optional MPC-solution visualization;
- `test_logger`: CSV logging for one evaluation run;
- `trajectory_publisher`: auxiliary trajectory publisher.

## Execution architecture

The main execution flow is:

1. `ackermann_sim.launch.py` starts Webots with
   `vehicle_webots/worlds/baylands.wbt`.
2. `vehicle_webots_driver` publishes the Tesla state and receives `/cmd_vel`.
3. `episode_manager` selects a trajectory CSV file.
4. `trajectory_publisher` publishes the trajectory points on `/trajectory`.
5. `topp` interpolates the trajectory, computes curvature, and generates a
   speed profile on `/trajectory_topp`.
6. `ackermann_mpc` solves the MPC problem and publishes the final command on
   `/cmd_vel`.
7. The driver applies the speed and steering command to the simulated
   vehicle.

The MPC uses the following kinematic model:

```text
x_dot     = v cos(theta)
y_dot     = v sin(theta)
theta_dot = v tan(phi) / wheelbase
```

At every iteration, the controller solves a 20-step horizon with
`dt = 0.05 s`. The cost penalizes longitudinal, lateral, heading, and speed
errors, as well as control effort and changes between consecutive commands.

## Requirements

### System software

The project was developed and tested with **ROS 2 Jazzy** on
**Ubuntu 24.04.5 LTS**, using **Webots R2025a** and `webots_ros2_driver`.

### Python dependencies

The nodes in this branch import:

- NumPy;
- SciPy;
- Matplotlib, for the debug visualizer;
- CasADi, used by the MPC's IPOPT solver.

This branch does not import PyTorch, but the complete repository uses it in
the `webots_PPO_residual` branch. In some ROS 2 environments, installing
these dependencies through system packages conflicts with the Python
interpreter used by ROS. The practical solution used in this project is to
install them into the same interpreter that runs the ROS nodes:

```bash
python3 -m pip install --user numpy scipy matplotlib casadi torch
```

> **Warning:** installing Python packages with `pip` into the system
> interpreter may replace versions used by ROS 2 packages. If installation is
> rejected because the environment is externally managed, use a Python
> environment compatible with ROS 2 or follow the package-management policy
> of your distribution. CasADi is required by `main`; PyTorch is only required
> by the residual-learning branch.

ROS dependencies can first be installed with `rosdep`:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Basic usage

After building the workspace and sourcing the installed workspace, execute:

```bash
ros2 launch vehicle_controller ackermann_sim.launch.py
```

The launch file opens Webots with `baylands.wbt` and starts the nominal MPC
controller.

### Optional launch arguments

| Argument | Options | Default | Description |
|---|---|---|---|
| `control_mode` | `nominal`, `rule_based` | `nominal` | Selects the nominal MPC or the camera-based braking baseline. |
| `training_mode` | `true`, `false` | `true` | Selects randomized training episodes or the fixed evaluation route. |
| `run_debug_visualizer` | `true`, `false` | `false` | Enables the MPC debug visualizer. |

For example:

```bash
ros2 launch vehicle_controller ackermann_sim.launch.py \
  control_mode:=rule_based \
  training_mode:=false \
  run_debug_visualizer:=true
```

## Evaluation logging

In evaluation mode, data is written by default to:

```text
~/mpc_test_logs_<control_mode>/
```

For example:

```text
~/mpc_test_logs_nominal/
~/mpc_test_logs_rule_based/
```

Each CSV contains, among other fields:

- ground-truth pose and velocity;
- IMU accelerations and angular velocities;
- the reference used by the MPC;
- MPC speed and steering commands;
- progress along the trajectory;
- solver time;
- the final command sent to the vehicle.

The output directory is constructed automatically in
`ackermann_sim.launch.py`. `test_logger` is designed for one evaluation run:
after receiving the episode result, it saves the file and requests shutdown
of the complete launch.

## Included trajectories

Trajectories are stored as CSV files containing `x,y` control-point columns:

```text
vehicle_controller/trajectories/train/
vehicle_controller/trajectories/test/
```

The auxiliary publisher can be used independently, for example:

```bash
ros2 run vehicle_controller trajectory_publisher \
  --ros-args -p trajectory_file:=train/baylands_nurbs_01.csv
```

This is not required during normal execution: `episode_manager` starts the
publisher with the selected trajectory.

## Configuration

Vehicle, MPC, and TOPP parameters are defined in
[`vehicle_controller/config/tesla.yaml`](vehicle_controller/config/tesla.yaml).

## Relevant ROS 2 topics

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Final command to the vehicle |
| `/ground_truth_odom` | `nav_msgs/Odometry` | Ground-truth vehicle state |
| `/trajectory` | `nav_msgs/Path` | Route control points |
| `/trajectory_topp` | `std_msgs/Float64MultiArray` | Route with yaw, speed, and progress |
| `/camera/segmentation` | `sensor_msgs/Image` | Speed-bump detection |
| `/mpc/debug` | `std_msgs/Float64MultiArray` | Diagnostic data |
| `/mpc/predicted_path` | `nav_msgs/Path` | MPC predicted horizon |
| `/mpc/success` | `std_msgs/Bool` | Goal-reached signal |

## License

The original source code in this repository is licensed under the
[Apache License 2.0](LICENSE).

Third-party assets remain subject to their original licenses. In particular,
the license of an asset is not changed when it is used or adapted in this
project.

## Acknowledgments

- The **Baylands** environment is based on the Baylands 3D model published by
  **OpenRobotics** in [Gazebo Fuel](https://fuel.gazebosim.org/1.0/OpenRobotics/models/baylands).
  The model is licensed under the
  [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/).
- The **Tesla Model 3** simulation assets are sourced from the official
  [Webots Tesla Model 3 vehicle model](https://github.com/cyberbotics/webots/blob/released/projects/vehicles/protos/tesla/TeslaModel3.proto).
  Several model components and textures declare
  [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), while the
  aggregate vehicle and other abstract vehicle components are distributed under
  the [Webots assets license](https://cyberbotics.com/webots_assets_license),
  with copyright by Cyberbotics Ltd.
- The Webots vehicle and environment infrastructure also uses public assets
  and PROTO definitions distributed with Webots.
