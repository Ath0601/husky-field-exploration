# Husky Field Exploration

ROS 2 package for a simulated Clearpath Husky implementing a complete
sense–plan–act pipeline: reference-tracking control, autonomous navigation
(SLAM + Nav2), and information-gain frontier exploration of unknown
environments in Gazebo.

Built for a PhD position assignment in Field Robotics:

| Task | Requirement | Where it lives |
|------|-------------|----------------|
| 1 | Control system accepting position and heading references; pose-to-pose and circular-trajectory tracking | `fieldrobo/pose_controller.py`, `fieldrobo/trajectory_generator.py`, `launch/task1_sim.launch.py` (nodes run via `ros2 run`) |
| 2 | State-of-the-art navigation stack with technical description of each sub-component | `config/nav2params.yaml`, `config/rtabmap_params.yaml`, `launch/task2_nav.launch.py` |
| 3 | Deployment of controller + navigation stack for exploration of an unknown environment | `fieldrobo/frontier_explorer.py`, `launch/task3_explorer.launch.py`, `world/task3_explorer.sdf` |

## Video results

<!-- Add links to your demo videos here -->

- Task 1 — pose-to-pose and circular trajectory tracking: [link]()
- Task 3 — autonomous exploration: [link]()

## System overview

```mermaid
flowchart LR
    subgraph Gazebo
        SIM[Gazebo Fortress] --- HUSKY[Husky\nVLP-16 LiDAR + IMU]
    end
    HUSKY -->|ros_gz_bridge| BR{{/velodyne_points,\n/imu/data_raw, /clock}}
    BR --> RTAB[RTAB-Map\n2D occupancy grid SLAM]
    BR --> ODOM[/husky_velocity_controller/odom]
    RTAB --> MAP[/map]
    MAP --> EXP[Information-gain\nfrontier explorer]
    EXP -->|NavigateToPose| NAV2[Nav2\nNavFn + DWB]
    NAV2 -->|cmd_vel_unstamped| CTRL[husky_velocity_controller\ngz_ros2_control]
    TRAJ[trajectory_generator\ncircular references] -->|/reference_pose| PC[pose_controller\nfeed-forward + feedback]
    PC -->|cmd_vel_unstamped| CTRL
    ODOM --> PC
    CTRL --> HUSKY
```

**Platform.** A Clearpath Husky (differential drive) simulated in Gazebo
Fortress, spawned from `urdf/clearpathHusky.urdf.xacro` with a Velodyne
VLP-16 GPU LiDAR (10 Hz point cloud on `/velodyne_points`) and an IMU
(50 Hz). `ros_gz_bridge` (`launch/bridge.launch.py` +
`config/bridge_config.yaml`, `config/lidar_bridge_config.yaml`) carries the
clock, sensor, and pose topics between Gazebo and ROS 2, and
`gz_ros2_control` runs the `husky_velocity_controller` diff-drive interface
(`.../cmd_vel_unstamped` in, `.../odom` out).

## Task 1 — Control system

One controller (`fieldrobo/pose_controller.py`) handles **both** stationary
pose-to-pose references and moving trajectory references, with no custom
message types: for moving references, `v_ref` and `ω_ref` are estimated from
consecutive `PoseStamped` references on `/reference_pose` and classified by
a speed threshold.

- **Moving reference** — feed-forward + feedback unicycle controller with
  body-frame position errors:

  $$v = v_{ref}\cos(e_\theta) + k_x\,e_x \qquad \omega = \omega_{ref} + k_y\,v_{ref}\,e_y + k_\theta\,\sin(e_\theta)$$

- **Stationary reference** — polar pose-regulation law
  ($k_\rho, k_\alpha, k_\beta$) drives the robot to the goal, then aligns
  final yaw.

Commands are saturated (`max_linear_velocity`, `max_angular_velocity`) and
run at a 50 Hz control rate. All gains and tolerances are ROS parameters, so
they can be tuned from launch files without rebuilding.

`fieldrobo/trajectory_generator.py` publishes the circular reference
trajectory (center, radius, period configurable; e.g. radius 2 m, 30 s
period by default) as a time-parametrized stream of `PoseStamped` on
`/reference_pose`, letting the controller track it exactly as it would any
moving reference.

## Task 2 — Navigation stack

`launch/task2_nav.launch.py` starts:

- **SLAM — [RTAB-Map](https://github.com/introlab/rtabmap)**: subscribes to
  the LiDAR point cloud and wheel odometry, and incrementally builds a 2D
  occupancy grid (`config/rtabmap_params.yaml`: 5 cm cells, 20 m max range,
  ground/obstacle height filtering). Loop closure detection keeps the map
  globally consistent.
- **Global planning — Nav2 `navfn_planner`**: computes a global route on the
  costmap from the current pose to the goal.
- **Local control — Nav2 `dwb_core::DWBLocalPlanner`**: tracks the global
  plan while respecting the Husky's velocity limits, with a smoother server
  refining plans and recovery behaviors (spin, backup, wait) on failure.
- **Costmaps**: local + global costmaps with a voxel layer (from the
  point cloud) and inflation layer, all under `config/nav2params.yaml`.
- **Sim-ROS bridge** — `ros_gz_bridge` as described above.

## Task 3 — Autonomous exploration

`launch/task3_explorer.launch.py` runs the full stack in the unknown
environment of `world/task3_explorer.sdf` and starts the
`information_gain_explorer` node (`fieldrobo/frontier_explorer.py`), which
closes the autonomy loop:

1. **Frontier detection** — on every occupancy grid update, free cells
   adjacent to unknown cells are clustered into connected components
   (8-connectivity, minimum size filter).
2. **Candidate generation** — each frontier yields several candidate goal
   cells, offset into known free space and checked for local clearance.
3. **Reachability filtering** — Nav2's `ComputePathToPose` action verifies
   each candidate is plannable and rejects paths with insufficient clearance.
4. **Goal selection** — candidates are scored by an information-gain
   utility: expected newly-sensed area (sensor radius) weighted against
   travel distance; goals too close to previous/failed goals are skipped.
5. **Execution & recovery** — the chosen goal goes to Nav2 via
   `NavigateToPose`; a watchdog timeout retries blacklisted goals and moves
   on, and the cycle repeats until no frontiers remain.

## Requirements

- Ubuntu 22.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Gazebo Fortress (installed with `ros-humble-ros-gz`)
- `colcon` (from `ros-humble-colcon-common-extensions`)

### Dependencies

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-rtabmap-ros \
                 ros-humble-ros-gz \
                 ros-humble-gz-ros2-control \
                 ros-humble-controller-manager \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-joint-trajectory-controller
```

## Build

```bash
cd ~/phd_ws
colcon build --packages-select fieldrobo
source install/setup.bash
```

## Usage

### Task 1 — Control system evaluation

`task1_sim.launch.py` starts everything: Gazebo with the Husky, the
diff-drive `husky_velocity_controller`, the Gazebo bridge, and RViz:

```bash
ros2 launch fieldrobo task1_sim.launch.py
```

In a second terminal, start the pose controller:

```bash
ros2 run fieldrobo pose_control
```

For pose-to-pose navigation, publish a reference pose (in the `odom` frame):

```bash
ros2 topic pub -1 /reference_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 2.0}}}'
```

For circular-trajectory tracking — with the simulation and controller still
running — start the trajectory generator (`traj_generate`, 2 m radius,
30 s period, centered at the origin by default):

```bash
ros2 run fieldrobo traj_generate

# or with custom geometry:
ros2 run fieldrobo traj_generate --ros-args -p radius:=3.0 -p period:=45.0
```

`task1_sim.launch.py` also accepts `world` (path to a Gazebo SDF), `rviz_config`,
`spawn_x`, `spawn_y` and `spawn_z` to reposition the robot or change the
environment.

### Task 2 — Navigation stack

```bash
ros2 launch fieldrobo task2_nav.launch.py
```

### Task 3 — Autonomous exploration of an unknown environment

```bash
ros2 launch fieldrobo task3_explorer.launch.py
```

### Useful commands while running

```bash
# Watch the controller output
ros2 topic echo /husky_velocity_controller/cmd_vel_unstamped
```

## Repository structure

```
fieldrobo/
├── config/          # Nav2, RTAB-Map, controller and Gazebo-bridge parameters
├── fieldrobo/       # Nodes: pose controller, trajectory generator, frontier explorer
├── launch/          # Launch files for the simulation, navigation, exploration and bridge
├── meshes/          # Husky chassis and VLP-16 / LMS1xx sensor mounts
├── urdf/            # Husky robot description (xacro)
├── world/           # Gazebo worlds (incl. the Task-3 unknown environment)
└── config/rviz/     # RViz configurations for control and navigation
```

## Results, limitations and future work

<!-- Summarize: tracking accuracy for Task 1, exploration coverage and
completion time for Task 3, and the limitations you discovered. The full
discussion lives in the submitted assignment PDF. -->

## AI tool usage

<!-- As required by the assignment, state which AI tools were used and for
what (e.g. launch-file boilerplate, debugging bridge topic mismatches,
documentation drafting), and what you built and verified yourself. -->

## License

Apache-2.0 — see [LICENSE](LICENSE).
