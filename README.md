# ROS 2 Smartphone-Based Autonomous Robotics System

A complete ROS 2 Jazzy mobile-robotics stack — robot description, simulation, SLAM and
autonomous navigation — built and verified **without any physical robot hardware**.

The simulated robot maps an unknown room by itself and then drives to commanded goals,
planning around obstacles. A smartphone is used as a source of *real* sensor data to
complement the simulated ones.

![System architecture](docs/architecture.svg)

---

## Project Overview

A differential-drive robot is described in URDF, simulated in Gazebo Harmonic with a
360-ray LiDAR, an RGB camera and an IMU, and driven through the standard ROS 2 interfaces.
`slam_toolbox` builds an occupancy grid of an unseen room while a custom reactive node
explores it; Navigation2 then localises against that saved map and drives autonomously to
goal poses.

Every component talks over standard ROS 2 topics, so the simulated sensors are
interchangeable with real ones: moving to physical hardware means replacing the Gazebo
bridge, not the autonomy code.

**Status of each part is stated honestly below** — the simulation, SLAM and navigation
pipeline is complete and verified; the smartphone sensor bridge and the OpenCV perception
node are designed and scaffolded but not yet implemented.

## Motivation

Robotics is usually gated behind expensive hardware. This project tests how far a complete
autonomy stack can be taken with a laptop alone, while keeping the architecture honest
enough that the same code would run on a real base.

Two constraints shaped it:

- **No purchased hardware.** No LiDAR, no robot chassis, no depth camera. A smartphone
  already carries a camera, an accelerometer, a gyroscope and GPS, so it becomes the one
  real sensor in the system.
- **Host OS is Kali Linux**, which is not a supported ROS 2 platform. The entire
  environment therefore lives in a Docker container defined by `docker/Dockerfile`, and
  nothing is installed on the host.

## System Architecture

```
Gazebo Harmonic  ──gz-transport──>  ros_gz_bridge  ──DDS──>  ROS 2 topics
   simulated LiDAR / camera / IMU                              /scan /odom /imu /tf
   differential-drive plugin                                        │
                                                                    ├──> slam_toolbox ──> /map
                                                                    ├──> Navigation2  ──> /cmd_vel
                                                                    └──> RViz2 / rosbag2
```

The key property is that **no consumer knows where its data came from.** `slam_toolbox`
subscribes to `/scan` carrying `sensor_msgs/LaserScan`; whether a Gazebo GPU-LiDAR plugin
or an RPLidar driver published it is invisible and irrelevant. This is what makes the
simulation a genuine development environment rather than a demo.

![Node and topic graph](docs/node_graph.svg)

### Coordinate frames

```
map ──> odom ──> base_footprint ──> base_link ──> {left_wheel, right_wheel, lidar,
                                                   camera ──> camera_optical, imu, caster}
```

- `odom -> base_footprint` comes from wheel odometry: smooth and continuous, but it drifts.
- `map -> odom` is published by SLAM (while mapping) or AMCL (while navigating). It *is*
  the accumulated odometry error, corrected. It jumps when a loop closes, which is normal.
- Nothing publishes `map -> base_footprint` directly; two publishers on one frame would
  conflict, so the correction is inserted upstream instead.
- `camera_optical_link` exists separately from `camera_link` because vision libraries use
  z-forward/x-right/y-down while ROS bodies use x-forward/y-left/z-up. Conflating them
  rotates every projection by 90°.

## Technologies Used

| Area | Tools |
|---|---|
| Middleware | ROS 2 Jazzy, DDS (Fast DDS), `rclpy` |
| Simulation | Gazebo Harmonic 8.15, `ros_gz_bridge`, SDF worlds |
| Robot description | URDF + xacro, TF2, `robot_state_publisher` |
| Mapping | `slam_toolbox` (async, pose-graph SLAM with Ceres) |
| Navigation | Navigation2 — NavFn global planner, Regulated Pure Pursuit controller, AMCL, costmaps |
| Vision | OpenCV 4.6, `cv_bridge` *(scaffolded, not yet implemented)* |
| Visualisation | RViz2, `rqt_graph`, `tf2_echo`, `ros2 topic hz/echo` |
| Environment | Docker, Xvfb (headless screenshots) |
| Language | Python 3.12 |

## Repository Structure

```
.
├── docker/Dockerfile                  # the whole environment, reproducible
├── scripts/
│   ├── capture_rviz.sh                # screenshots via Xvfb, no display needed
│   ├── record_demo.sh                 # the demo video, same way
│   ├── bake_rviz_geometry.sh          # let RViz write its own panel layout
│   ├── check_xml.py                   # catch the '--' in XML comments trap
│   └── stop_all.sh                    # clean shutdown (see Limitations)
├── src/
│   ├── smartphone_robot_interfaces/   # custom action definitions (ament_cmake)
│   │   └── action/DriveDistance.action
│   └── smartphone_robot/              # main package (ament_python)
│       ├── smartphone_robot/          # nodes
│       │   ├── velocity_publisher.py     # /cmd_vel producer, service-gated
│       │   ├── velocity_monitor.py       # /cmd_vel watchdog + interpreter
│       │   ├── drive_distance_server.py  # action server with feedback + cancel
│       │   └── wander_node.py            # reactive obstacle avoidance from /scan
│       ├── launch/                    # display, gazebo, mapping, navigation
│       ├── config/                    # slam, nav2, bridge, node parameters
│       ├── urdf/                      # robot.urdf.xacro, gazebo.xacro, inertials
│       ├── worlds/indoor.world        # 8×8 m room with partition and obstacles
│       ├── maps/                      # map produced by SLAM
│       └── rviz/                      # three RViz configurations
├── docs/                              # architecture diagrams (SVG)
└── screenshots/
```

## Installation

Requires Docker. Nothing is installed on the host OS.

```bash
git clone <this-repo> ros2-smartphone-robot
cd ros2-smartphone-robot

docker build -t ros2_smartphone_robot:jazzy -f docker/Dockerfile docker/

docker run -dit --name ros2_dev \
  --network host --ipc host \
  --device /dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD":/home/dev/ros2_ws \
  ros2_smartphone_robot:jazzy

xhost +SI:localuser:$USER          # allow the container to draw on your X display
docker exec -it ros2_dev bash
colcon build --symlink-install
source install/setup.bash
```

Notes on the `docker run` flags, since they are not decoration:

- `--network host` — ROS 2 has no master; nodes discover each other by broadcasting on the
  network. Docker's default bridge network breaks that, and would also stop a phone on the
  same Wi-Fi from reaching the bridge node.
- `--ipc host` — lets DDS use shared memory instead of loopback TCP, which matters once
  camera frames are flowing.
- `--device /dev/dri` — hardware OpenGL for Gazebo; without it rendering falls back to
  software and the simulation crawls.
- The image renames its `ubuntu` user to `dev` while keeping **uid 1000**, so files created
  in the container are owned by the host user and editable without `sudo`.

## How to Run

**1 — Inspect the robot model (no simulator)**

```bash
ros2 launch smartphone_robot display.launch.py
```
RViz plus slider controls for the wheel joints. The fastest way to check a URDF change.

**2 — Simulation only**

```bash
ros2 launch smartphone_robot gazebo.launch.py            # with Gazebo GUI
ros2 launch smartphone_robot gazebo.launch.py gz_args:='-r -v 3 -s'   # headless
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

**3 — Build a map (SLAM)**

```bash
ros2 launch smartphone_robot mapping.launch.py
# the robot explores on its own; when the map looks complete:
ros2 run nav2_map_server map_saver_cli -f src/smartphone_robot/maps/indoor \
  --ros-args -p use_sim_time:=true
```

**4 — Autonomous navigation**

```bash
ros2 launch smartphone_robot navigation.launch.py
```
Click **Nav2 Goal** in RViz, or send one from a terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0}, \
    orientation: {w: 1.0}}}}" --feedback
```

**Always stop a run with `scripts/stop_all.sh`** — see Limitations.

### Custom nodes

```bash
ros2 run smartphone_robot velocity_publisher --ros-args -p linear_x:=0.3
ros2 service call /velocity_publisher/enable_motion std_srvs/srv/SetBool "{data: false}"
ros2 param set /velocity_publisher linear_x 0.5     # validated at runtime
ros2 action send_goal /drive_distance \
  smartphone_robot_interfaces/action/DriveDistance "{distance: 1.0, speed: 0.25}" --feedback
```

## Results

Measured in the 8 × 8 m `indoor.world`, all verified from live topic output:

| Result | Value |
|---|---|
| Simulated LiDAR | 360 rays, 10 Hz, 0.12–12 m, Gaussian noise σ = 0.01 m |
| Simulated camera | 640 × 480 `rgb8`, 15 Hz |
| Simulated IMU | 50 Hz |
| Odometry | 30 Hz, full `odom → base_footprint` TF |
| **SLAM map** | 8.2 × 8.8 m at 0.05 m/cell; every wall, the L-partition, both pillars, three crates, the planter and even the four table legs reconstructed |
| **Localisation accuracy** | **76–83 % of laser returns land exactly on a mapped wall cell, 0 % fall outside the map, mean error 0.9–2.7 cm** (measured, see below) |
| **Navigation** | consecutive goals all `SUCCEEDED` with **0 recovery behaviours** and `Failed to make progress: 0`; final position error 0.16–0.20 m against a 0.20 m tolerance |
| Velocity control | commanded 10 Hz measured at 9.999 Hz; 20 Hz at 20.00 Hz |

### How localisation accuracy was measured

Rather than judging alignment by eye, each laser return is transformed into the map
frame through TF and the occupancy grid is sampled at that point:

| | Before tuning | After tuning |
|---|---|---|
| Returns landing exactly on a wall cell | 24.4 % | **82.8 %** |
| Within one cell (≤ 10 cm) | 28.1 % | 16.9 % |
| More than 10 cm off | 32.8 % | **0.3 %** |
| Falling outside the map | 14.7 % | **0 %** |
| Mean error | 7.9 cm | **0.9 cm** |

The fix was raising AMCL's `max_beams` from 60 to 180. With only 60 of the 360 returns
used, the filter had too little angular information to pin down heading, and a 2° heading
error displaces a return 7 m away by 24 cm — which is exactly what "the laser does not
line up with the walls" looks like.

![Simulated world](screenshots/07_gazebo_world.png)
*The Gazebo world: an 8 × 8 m room with an L-shaped partition, pillars, crates, a table
and a planter. Rendered by a fixed camera sensor inside the simulation — Gazebo's GUI
cannot run on a virtual framebuffer, but its sensors render on the GPU regardless.*

![Robot](screenshots/07_robot_gazebo.png)
*The robot in simulation. The orange band on the LiDAR marks the scan plane at 0.17 m:
anything below that line is invisible to a 2D laser, which is why the map shows the
table's four legs but not its top.*

![Robot camera](screenshots/07_robot_camera_view.png)
*A frame from the robot's own camera, taken straight off `/camera/image_raw` — the full
path from Gazebo through the bridge into ROS 2.*

![SLAM in progress](screenshots/08_slam_rviz_live.png)
*Live SLAM: laser returns (red) landing on the walls of the map being built.*

![Saved map](screenshots/08_slam_map.png)
*The saved occupancy grid. Compare with `worlds/indoor.world`: outer walls, the L-shaped
partition, two cylindrical pillars and two crates are all recovered.*

![Autonomous navigation](screenshots/09_nav2_navigation.png)
*Navigation2: global plan (green), inflated costmap around obstacles, AMCL localisation.*

![Robot model and TF](screenshots/06_rviz_robot_model_tf.png)
*URDF model with its TF frames.*

## Demo Video

[`demo/navigation_demo.mp4`](demo/navigation_demo.mp4) — 75 s, 1600 × 1000, recorded
directly from RViz while the robot executed two navigation goals back to back. Both
finished `SUCCEEDED`. The global plan (green) and the inflated costmap are visible
throughout.

It was produced with `scripts/record_demo.sh`, which renders RViz onto an Xvfb virtual
display and captures it with ffmpeg — so it needs no physical screen and anyone with this
repository can regenerate it:

```bash
ros2 launch smartphone_robot navigation.launch.py rviz:=false      # terminal 1
scripts/record_demo.sh \
  install/smartphone_robot/share/smartphone_robot/rviz/navigation.rviz \
  demo/navigation_demo.mp4 75                                      # terminal 2
```

> To make the video play inline on GitHub rather than appear as a download link, drag the
> file into a GitHub issue or pull-request comment and paste the resulting CDN URL here.
> Video files served from the repository itself do not embed.

## Engineering Notes

Problems that were found by testing rather than by reading tutorials, and how they were fixed:

- **A node that dies mid-motion leaves the robot driving.** `/cmd_vel` is a latched intent,
  not a pulse. `rclpy`'s default SIGINT handler tears the ROS context down *before*
  `spin()` returns, so a zero-velocity publish in `finally` fails with
  `publisher's context is invalid`. Fixed by taking the signal with
  `SignalHandlerOptions.NO` and handling shutdown explicitly.
- **Log throttling hid the most important message.** The `/cmd_vel` monitor throttled every
  line at 1 Hz, which silently swallowed the `STOP` transition. State *changes* are now
  logged immediately and only repetitions are throttled.
- **`slam_toolbox` on Jazzy is a lifecycle node.** Launched on its own it starts, subscribes
  to nothing, publishes nothing, and looks healthy. It needs a `nav2_lifecycle_manager`
  with `autostart`.
- **Launch configurations leak across `IncludeLaunchDescription` in both directions** —
  first starting a second RViz, then suppressing the intended one. Fixed with
  `GroupAction(scoped=True, forwarding=True)`.
- **Nav2's `map:=` argument does not reach `localization_launch.py` here**, so `map_server`
  came up with an empty `yaml_filename`, AMCL logged *"Waiting for map...."* forever, and
  the whole navigation bringup aborted because `global_costmap` could never find the `map`
  frame. Fixed by injecting the path with `RewrittenYaml`.
- **"The laser does not line up with the walls" had two separate causes.** One was real
  (AMCL using 60 of 360 beams, above). The other was not: RViz draws the map at z = 0 but
  laser returns at the LiDAR's height of 0.17 m, so with a tilted camera the returns are
  projected outward and appear offset — badly at the edges of the view, not at all near
  the centre, which is why it looked intermittent. The RViz configs now default to a
  near-top-down view so parallax cannot be mistaken for a localisation error.
- **DWB stalled in tight spaces.** Near inflated obstacles every sampled trajectory
  scored badly, so the controller commanded almost nothing and the progress checker
  reported `Failed to make progress`. Switching to Regulated Pure Pursuit — fewer
  parameters, designed for differential drives — and reducing the inflation radius from
  0.55 m to 0.45 m took recoveries from 2 per goal to 0.
- **nav2_lifecycle_manager reported SLAM as dead while it was working.** slam_toolbox
  never establishes the heartbeat bond the manager expects, so the manager logged
  `unable to be reached ... Aborting bringup` at any timeout. Configure and activate had
  already succeeded, so SLAM mapped perfectly throughout. Bond checking is now disabled
  for that node.
- **An RViz config does not fully describe an RViz layout.** Panel sizes live in
  `Window Geometry: QMainWindow State`, a Qt-serialised blob tied to the window size.
  A hand-written config without it looks completely different on another screen, because
  RViz falls back to distributing panels evenly. `scripts/bake_rviz_geometry.sh` runs
  RViz on a 1920x1080 virtual display, sends it Ctrl+S, and splices the resulting block
  back into the hand-written file so the comments survive. That also needs a window
  manager on the virtual display — without one, windows cannot be focused and keystrokes
  never arrive.
- **RViz applies one material per link; Gazebo applies one per visual.** A link with
  several `<visual>` elements renders in RViz entirely in the *first* one's colour.
  Confirmed by reordering: moving the blue deck ahead of the chassis turned the whole
  body blue. So the robot shows its full detail in Gazebo and a single flat colour in
  RViz, and the visual order in the URDF is chosen with that in mind.
- **XML comments cannot contain `--`.** This broke every xacro file, the world file and
  a robot description four separate times, each time surfacing as an unhelpful
  "not well-formed (invalid token)". `scripts/check_xml.py --fix` now catches it.
- **An unused Nav2 server stopped navigation from starting, silently.** `nav2_bringup`
  brings up `docking_server`, `route_server` and `waypoint_follower` as well as the
  planning stack. This project uses none of them, but they still have to be given valid
  parameters or the lifecycle manager aborts the whole bringup — and worse, under load
  `docking_server` once timed out sending its lifecycle response, so the manager waited
  forever, the navigation servers never activated, and every goal came back
  `Action server is inactive. Rejecting the goal.` with no error logged anywhere.
  `navigation.launch.py` now lists the seven nodes this project actually uses, which
  removes the failure mode and makes the /cmd_vel chain visible instead of buried in an
  included launch file.
- **Stale processes corrupt simulated time.** Two surviving `ros_gz_bridge` instances both
  republish `/clock` and `/scan`, producing `Transform data too old` and aborted goals that
  look exactly like a tuning problem. `pkill -x` misses them because `/proc/<pid>/comm` is
  truncated to 15 characters (`parameter_bridg`). Hence `scripts/stop_all.sh`.

## Limitations

Stated plainly, because they matter for interpreting the results:

- **The robot is simulated.** Gazebo's sensor noise is Gaussian and well-behaved; real LiDAR
  is defeated by glass, mirrors and dark surfaces, real wheels slip on dust, and real
  timestamps jitter. Simulation flatters these algorithms.
- **The smartphone bridge and the OpenCV node are not implemented yet.** They are designed
  (see the dashed boxes in the architecture diagram) and their dependencies are installed
  and verified, but no phone data flows through the system today. Nothing in this README
  claims otherwise.
- **`DriveDistance` measures distance open-loop**, as `speed × elapsed_time`. It assumes no
  wheel slip and no acceleration ramp. It should subscribe to `/odom` instead; the action
  *interface* would not change, only the implementation.
- **Tuned for one room.** Nav2's costmap and controller parameters were set for an 8 × 8 m indoor
  world and this robot's geometry. They are not general-purpose defaults.
- **Host resources are the binding constraint.** The development machine has 7.4 GB RAM and
  an integrated GPU, so Gazebo usually runs headless and transform tolerances are raised
  above their textbook values to survive scheduling lag.

## Future Improvements

1. **Smartphone sensor bridge** — a Python node turning a phone's HTTP/JSON sensor stream
   into `sensor_msgs/Imu` and `sensor_msgs/Image`, with correct `header.stamp` and
   `frame_id`. This is the next piece of work.
2. **OpenCV perception node** — subscribe to the camera, detect objects, publish detections,
   and let them influence robot behaviour.
3. **Closed-loop `DriveDistance`** using `/odom` instead of dead reckoning.
4. **Sensor fusion** — `robot_localization` combining wheel odometry with IMU, which is
   where the phone's real IMU becomes genuinely useful.
5. **`rosbag2` datasets** so perception can be tested repeatably without the simulator.
6. **Port to hardware** — the point of the architecture. Replacing `ros_gz_bridge` with real
   drivers should leave SLAM, Nav2 and the custom nodes untouched.

## License

Apache-2.0
