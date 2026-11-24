# Residual MPC-RL

This repository contains the simulation and learning framework for residual
reinforcement learning on top of MPC path tracking over localized terrain
irregularities.

The associated paper is:

> **Recurrent Residual Reinforcement Learning for MPC-Based Path Tracking over
> Localized Terrain Irregularities**

The paper is currently **under ICRA review**.

## Demonstration video

[![Demonstration video](https://img.youtube.com/vi/eCw2W8Uuggc/hqdefault.jpg)](https://youtu.be/eCw2W8Uuggc)

## Branches

| Branch | Description |
|---|---|
| [`main`](../../tree/main) | Kinematic MPC and camera-based rule-based MPC baselines for a Tesla Model 3 in Webots. |
| [`webots_PPO_residual`](../../tree/webots_PPO_residual) (this branch) | PPO residual-learning implementation for the feed-forward residual variant (FF-RRL) and Recurrent Residual RL (3ReL). |

## Overview

The system combines a nominal MPC path tracker with a learned residual
controller:

```text
final command = MPC command + PPO residual correction
```

The MPC remains responsible for nominal path tracking. PPO learns a bounded
longitudinal correction for unmodeled chassis dynamics caused by speed bumps.
The longitudinal action is enabled, while steering residual corrections are
disabled.

Two policy variants are supported:

- **FF-RRL:** a purely feed-forward Actor-Critic policy; set
  `use_recurrence=False` in `ppo/config.py`.
- **3ReL:** a recurrent Actor-Critic policy using an LSTM; this branch defaults
  to `use_recurrence=True`.

The recurrent policy addresses partial observability caused by the occlusion
gap: a speed bump can leave the forward camera's field of view before the
vehicle reaches it. The LSTM maintains a hidden and cell state across control
steps and resets at the end of an episode.

## Repository contents

The workspace contains two ROS 2 packages.

### `vehicle_webots`

- Tesla Model 3 vehicle model with Ackermann steering geometry.
- Baylands Webots world with roads, curves, curbs, potholes, and 21 candidate
  speed-bump locations.
- C++ Webots driver publishing ground-truth odometry, IMU, GPS, wheel state,
  and a two-channel segmentation camera image.
- ROS/Webots synchronization for MPC and PPO control steps.
- `SpeedBumpSupervisor`, which can hide four speed bumps during each training
  episode.

### `vehicle_controller`

- `ackermann_mpc`: nominal kinematic-bicycle MPC.
- `topp`: trajectory reconstruction and time-optimal speed-profile generation.
- `rl_master`: episode selection, reset, success/crash detection, and episode
  lifecycle management.
- `ppo_agent`: PPO training/evaluation, actor-critic model, replay buffer,
  checkpoint handling, and residual command generation.
- `ppo_debug_visualizer`: live plots and CSV logging of episode and PPO
  metrics.
- `test_logger`: IMU-driven evaluation logging.
- `trajectory_publisher`: publishes a selected route to `/trajectory`.

## Execution architecture

The control loop has a synchronous two-phase structure:

1. Webots publishes `/sim/trigger` every 50 ms and pauses in training mode.
2. `ackermann_mpc` computes the nominal command and publishes it on
   `/cmd_vel_mpc`.
3. `ppo_agent` receives the segmentation image, vehicle state, tracking errors,
   and MPC command.
4. The Actor-Critic policy produces a residual action.
5. `ppo_agent` publishes the base-plus-residual command on `/cmd_vel`.
6. The Webots driver applies the command and resumes simulation.

In evaluation mode, Webots runs normally and the PPO node publishes the final
deterministic command directly.

## Observation and action

The two-channel segmentation image is processed by a CNN:

- Channel 0: curbs.
- Channel 1: speed bumps.
- Resolution: `128 x 96`.

The vector observation is 12-dimensional:

```text
[vx, vy, vz, wx, wy, wz,
 e_lat, e_lon, e_yaw, e_v,
 mpc_v, mpc_steer]
```

The policy has a two-dimensional action:

```text
[delta_v, delta_steer]
```

By default, `delta_steer` is disabled with an action scale of zero. The
default longitudinal scale is 6 m/s. The final command is:

```text
v_final = mpc_v + residual_v
w_final = v_final / wheelbase * tan(mpc_steer)
```

## Policy architecture

The default model in `vehicle_controller/vehicle_controller/ppo/config.py`
uses:

- three convolutional feature-extraction stages;
- a 512-unit image embedding;
- two 128-unit vector-encoding layers with Mish activations;
- a 256-unit shared hidden layer;
- an LSTM with a 256-unit hidden state;
- Normal policy outputs squashed with `tanh`;
- a scalar value head.

Set `use_recurrence=False` to produce an FF-RRL policy. Set it to `True` for
3ReL. LSTM/GRU hidden states are reset on episode termination and are not
preserved across episodes.

## PPO and reward

The default PPO configuration uses:

| Parameter | Default |
|---|---:|
| Discount factor `gamma` | `0.99` |
| GAE parameter `lambda` | `0.95` |
| Collected steps per update | `1024` |
| PPO epochs | `4` |
| Mini-batches | `8` |
| Truncated recurrent sequence length | `32` |
| Value loss coefficient | `0.5` |
| Maximum gradient norm | `0.5` |
| Maximum episodes | `200` |

The reward includes:

- lateral, longitudinal, heading, and speed tracking penalties;
- residual-magnitude regularization;
- command slew penalties;
- roll and pitch rate penalties with dead bands;
- vertical-velocity penalty;
- reward for route progress;
- terminal rewards for success, rollover, being stuck, or falling.

Default terminal values are:

| Outcome | Reward |
|---|---:|
| Success | `+500` |
| Rollover | `-200` |
| Stuck | `-100` |
| Fall | `-200` |

The complete values are configurable in
[`vehicle_controller/vehicle_controller/ppo/config.py`](vehicle_controller/vehicle_controller/ppo/config.py).

## Requirements

### System software

The project was developed and tested with:

- **Ubuntu 24.04.5 LTS**
- **ROS 2 Jazzy**
- **Webots R2025a**
- `webots_ros2_driver`

### Additional dependencies

The residual-learning stack additionally requires:

- PyTorch;
- NumPy;
- OpenCV;
- SciPy;
- Matplotlib;
- TensorBoard;
- CasADi, used by the nominal MPC solver.
- `cv_bridge`, provided by the ROS 2 package `ros-jazzy-cv-bridge`.

For a practical setup in an environment where ROS and PyTorch coexist:

```bash
python3 -m pip install --user numpy scipy matplotlib casadi torch opencv-python tensorboard
sudo apt install ros-jazzy-cv-bridge
```

Installing Python packages into a system interpreter may conflict with ROS 2
packages. If the environment uses externally managed Python packages, use a
compatible Python environment or the package-management policy of the
distribution.

CasADi and TensorBoard are required by the MPC/PPO implementation; PyTorch,
OpenCV, and `cv_bridge` are required by the PPO node.

## Basic usage

After building and sourcing the workspace:

```bash
ros2 launch vehicle_controller training_RL.launch.py
```

This launches:

- Webots and the Tesla vehicle driver.
- TOPP and the nominal MPC.
- `rl_master`.
- `ppo_agent`.
- `ppo_debug_visualizer`.

### Optional launch arguments

| Argument | Options | Default | Description |
|---|---|---|---|
| `training_mode` | `true`, `false` | `true` | Run PPO training or load the final evaluation checkpoint. |
| `seed` | Integer | `0` | Sets PPO, Python, NumPy, CUDA, trajectory, and Webots seeds where supported. |
| `checkpoint_dir` | Directory path | `~/ppo_checkpoints` | Directory containing PPO checkpoints. |
| `output_dir` | Directory path | `~/ppo_test_logs` | Directory for evaluation CSV files. |

Example training run:

```bash
ros2 launch vehicle_controller training_RL.launch.py \
  training_mode:=true \
  seed:=0
```

Example evaluation run:

```bash
ros2 launch vehicle_controller training_RL.launch.py \
  training_mode:=false \
  seed:=0 \
  checkpoint_dir:=~/ppo_checkpoints \
  output_dir:=~/ppo_test_logs
```

In evaluation mode, `ppo_agent` loads the highest matching checkpoint and uses
deterministic policy actions. `test_logger` records one evaluation episode,
writes a CSV file, and shuts down the launch.

## Checkpoints and logs

Checkpoints are stored as:

```text
<checkpoint_dir>/ppo_<run_id>_ep<episode>.pt
```

The default run ID is:

```text
speedbump_seed_<seed>
```

For example:

```text
~/ppo_checkpoints/ppo_speedbump_seed_0_ep000200.pt
```

Training metrics are also written to:

- TensorBoard summaries under `./summaries/<run_id>/`;
- PPO debug visualizer CSV files under
  `<checkpoint_dir>/<run_id>/`;
- checkpoint files under `<checkpoint_dir>`.

Evaluation CSV files are named using the run ID, seed, route, and episode
result.

## Trajectories and episode randomization

Training routes are read from:

```text
vehicle_controller/trajectories/train/
```

Evaluation routes are read from:

```text
vehicle_controller/trajectories/test/
```

In training mode, `rl_master` randomly selects a training route and
`SpeedBumpSupervisor` randomly hides four of the 21 speed bumps before each
episode.

In evaluation mode, the selected test route is used and the world speed bumps
remain visible.

Episode termination is triggered by:

- reaching the goal;
- rollover;
- insufficient progress/stuck behavior;
- leaving the drivable area/falling.

## PPO debug visualizer

The `ppo_debug_visualizer` node subscribes to:

- `/ppo/episode_metrics`;
- `/ppo/train_metrics`;
- `/ppo/reward_terms`;
- `/rl/trajectory_id`.

It displays episode reward and length, policy/value/total losses, entropy,
approximate KL divergence, update time, and individual reward components. It
also writes their values to CSV files.

## Configuration

Vehicle, MPC, and TOPP parameters are defined in
[`vehicle_controller/config/tesla.yaml`](vehicle_controller/config/tesla.yaml).

PPO, recurrence, network, reward, and schedule parameters are defined in
[`vehicle_controller/vehicle_controller/ppo/config.py`](vehicle_controller/vehicle_controller/ppo/config.py).

To train FF-RRL instead of 3ReL, set:

```python
"use_recurrence": False
```

in `ppo/config.py`, then rebuild or restart the launch as appropriate.

## Relevant ROS 2 topics

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Final MPC-plus-PPO vehicle command |
| `/cmd_vel_mpc` | `geometry_msgs/TwistStamped` | Nominal MPC command |
| `/ground_truth_odom` | `nav_msgs/Odometry` | Ground-truth vehicle state |
| `/camera/segmentation` | `sensor_msgs/Image` | Two-channel curb/speed-bump segmentation |
| `/trajectory` | `nav_msgs/Path` | Route control points |
| `/trajectory_topp` | `std_msgs/Float64MultiArray` | Route with yaw, speed, and progress |
| `/mpc/debug` | `std_msgs/Float64MultiArray` | Tracking state and MPC diagnostics |
| `/mpc/predicted_path` | `nav_msgs/Path` | MPC prediction horizon |
| `/mpc/success` | `std_msgs/Bool` | Goal-reached signal |
| `/ppo/episode_metrics` | `std_msgs/Float64MultiArray` | Episode reward and length |
| `/ppo/train_metrics` | `std_msgs/Float64MultiArray` | PPO update statistics |
| `/ppo/reward_terms` | `std_msgs/Float64MultiArray` | Individual reward components |

## License

The original source code in this repository is licensed under the
[Apache License 2.0](../../blob/main/LICENSE).

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
- The Webots vehicle and environment infrastructure uses public assets and
  PROTO definitions distributed with Webots.
