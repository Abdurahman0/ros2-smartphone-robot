# ROS 2 Smartphone-Based Autonomous Robotics System

A complete ROS 2 Jazzy mobile-robotics stack — robot description, simulation, SLAM and
autonomous navigation — built and verified **without any physical robot hardware**.

The simulated robot maps an unknown room by itself and then drives to commanded goals,
planning around obstacles. A smartphone is used as a source of *real* sensor data to
complement the simulated ones.

![System architecture](docs/architecture.svg)

**Documentation:** [Project Report](docs/REPORT.md) (formal write-up) ·
[Portfolio & interview prep](docs/portfolio.md) ·
[Node/topic graph](docs/node_graph.svg)

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
pipeline is complete and verified; the smartphone IMU/GPS bridge, the phone-camera bridge
and the OpenCV / YOLO perception nodes are all implemented and verified end to end against
real hardware.

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
| Phone sensors | Phyphox HTTP API, `urllib`, `sensor_msgs/Imu` + `NavSatFix` |
| Phone camera | IP Camera Lite MJPEG/RTSP, OpenCV `VideoCapture`, `cv_bridge` |
| Vision | OpenCV 4.6 HSV colour servoing + YOLOv4-tiny (`cv2.dnn`, 80 COCO classes) |
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
│   ├── experiment_beam_count.sh       # controlled AMCL sweep against ground truth
│   ├── add_room.py                    # drop a 3D mesh -> Gazebo world + mapping command
│   ├── download_house.sh              # fetch a full house world from Gazebo Fuel
│   ├── download_yolo.sh               # fetch the object-detection model
│   ├── save_map.sh                    # export the live SLAM map to .pgm + .yaml
│   └── stop_all.sh                    # clean shutdown (see Limitations)
├── src/
│   ├── smartphone_robot_interfaces/   # custom action definitions (ament_cmake)
│   │   └── action/DriveDistance.action
│   └── smartphone_robot/              # main package (ament_python)
│       ├── smartphone_robot/          # nodes
│       │   ├── node_utils.py             # shared node lifecycle (run_node helper)
│       │   ├── drive_distance_server.py  # action server with feedback + cancel
│       │   ├── wander_node.py            # reactive avoidance; auto-stops when mapped
│       │   ├── keyboard_teleop.py        # arrow-key driving
│       │   ├── phone_sensor_bridge.py    # phone IMU/GPS  -> sensor_msgs
│       │   ├── phone_camera_bridge.py    # phone camera   -> sensor_msgs/Image
│       │   ├── object_follower.py        # colour-blob visual servoing
│       │   ├── object_detector.py        # YOLOv4-tiny object recognition
│       │   ├── camera_viewer.py          # sizable image window
│       │   └── localization_eval.py      # AMCL error vs ground truth -> CSV
│       ├── launch/                    # 8 launch files (see How to Run)
│       ├── config/                    # slam, nav2, bridge parameters
│       ├── urdf/                      # robot.urdf.xacro, gazebo.xacro, inertials
│       ├── worlds/                    # indoor.world (8×8 m) + house.world (Fuel)
│       ├── meshes/                    # imported room meshes (see add_room.py)
│       ├── maps/                      # map produced by SLAM
│       └── rviz/                      # RViz configs (display, nav, follow, teleop)
├── models/yolo/                       # YOLOv4-tiny (weights via scripts/download_yolo.sh)
├── results/                           # localisation-error CSVs from the experiments
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

Every command is run inside the container (`docker exec -it ros2_dev bash`). **Only one
launch at a time** — each owns Gazebo and `/cmd_vel`. Between runs:

```bash
bash scripts/stop_all.sh
```

### Simulation demos (no phone needed)

**Inspect the robot model** — RViz with slider-driven joints, no simulator.
```bash
ros2 launch smartphone_robot display.launch.py
```

**Drive it by hand** — arrow keys, a clean 3D RViz, and the robot camera.
```bash
ros2 launch smartphone_robot teleop.launch.py
```
Click the small xterm window, then drive with the arrow keys (or WASD); `+`/`-` change
speed, space stops.

**Build a map (SLAM)** — the robot explores by itself, **stops automatically when the map
is complete**, then the arrow keys take over. RViz and the robot camera open too.
```bash
ros2 launch smartphone_robot mapping.launch.py
# after "MAPPING COMPLETE", save it from another terminal:
bash scripts/save_map.sh indoor   # writes maps/indoor.pgm + maps/indoor.yaml
```

**Map your own room from a 3D mesh** — drop in a `.dae` / `.obj` / `.stl` and the
script builds a Gazebo world and prints the exact mapping command to run.
```bash
python3 scripts/add_room.py path/to/room.dae myroom
colcon build --symlink-install     # pick up the new world + meshes
# then run the command it printed, e.g.:
ros2 launch smartphone_robot mapping.launch.py world:=myroom.world x:=0 y:=0 rviz:=true
```
If the robot spawns inside a wall or sees nothing, re-run with `--spawn "X Y"` pointing
at open floor. OBJ meshes without vertex normals are repaired automatically (DART mesh
collision needs them).

**Map a downloaded 3D house** — fetch a full house from Gazebo Fuel and map it.
```bash
bash scripts/download_house.sh
ros2 launch smartphone_robot mapping.launch.py world:=house.world rviz:=true
```

**Autonomous navigation** — localise on the saved map and drive to clicked goals.
```bash
ros2 launch smartphone_robot navigation.launch.py
```
Use **Nav2 Goal** in RViz (press, drag to set heading, release), or:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0}, \
    orientation: {w: 1.0}}}}" --feedback
```
If the robot mislocalises, use **2D Pose Estimate** to tell AMCL where it really is.

**Vision follow (simulated)** — the robot follows a red post with its OWN camera.
```bash
ros2 launch smartphone_robot follow_sim.launch.py rviz:=true
```

### Smartphone demos (phone on the same Wi-Fi / hotspot)

First check the phone is reachable: `ping 172.20.10.1`. The `%40` below is the `@` in the
password, URL-encoded.

**Object detection (YOLO)** — the phone camera recognises real objects.
```bash
ros2 launch smartphone_robot phone_detect.launch.py \
  stream_url:='http://user:%40pass@172.20.10.1:8081/video'
```

**Phone steers the robot** — a colour the phone sees turns the simulated robot.
```bash
ros2 launch smartphone_robot phone_follow.launch.py \
  stream_url:='http://user:%40pass@172.20.10.1:8081/video'
```

**Phone IMU / GPS into ROS** — real motion data as `sensor_msgs`.
```bash
ros2 run smartphone_robot phone_sensor_bridge --ros-args -p host:=172.20.10.1 -p port:=80
```

### Measurement

```bash
ros2 run smartphone_robot localization_eval --ros-args -p use_sim_time:=true \
  -p output_csv:=results/run.csv          # logs AMCL error vs ground truth
bash scripts/experiment_beam_count.sh 30 180 360   # controlled AMCL sweep
```

### Custom nodes

```bash
ros2 action send_goal /drive_distance \
  smartphone_robot_interfaces/action/DriveDistance "{distance: 1.0, speed: 0.25}" --feedback
```

## Code Reference

Every runnable piece, what it does, and the exact command. Launch files orchestrate
several nodes; the nodes can also be run alone with `ros2 run smartphone_robot <name>`.

### Launch files (`ros2 launch smartphone_robot <file>`)

| File | What it does | Command |
|---|---|---|
| `display.launch.py` | Robot URDF in RViz with slider-driven joints. No simulator. | `ros2 launch smartphone_robot display.launch.py` |
| `teleop.launch.py` | Gazebo + arrow-key driving + robot camera window. | `ros2 launch smartphone_robot teleop.launch.py` |
| `gazebo.launch.py` | Gazebo + robot only, no autonomy. Takes `world:=`, `x:=`, `y:=`. Included by the others. | `ros2 launch smartphone_robot gazebo.launch.py world:=indoor.world` |
| `mapping.launch.py` | SLAM: robot auto-explores, stops when the map is complete, then arrow keys take over. RViz + camera. Takes `world:=`, `x:=`, `y:=`, `rviz:=`, `teleop:=`, `camera:=`. | `ros2 launch smartphone_robot mapping.launch.py` |
| `navigation.launch.py` | Localise on a saved map (AMCL) and drive to Nav2 goals. | `ros2 launch smartphone_robot navigation.launch.py` |
| `follow_sim.launch.py` | Visual servoing in sim: robot follows a red post with its own camera. | `ros2 launch smartphone_robot follow_sim.launch.py rviz:=true` |
| `phone_follow.launch.py` | A colour the phone camera sees steers the simulated robot. | `ros2 launch smartphone_robot phone_follow.launch.py stream_url:='http://user:%40pass@172.20.10.1:8081/video'` |
| `phone_detect.launch.py` | YOLO object recognition on the live phone camera. | `ros2 launch smartphone_robot phone_detect.launch.py stream_url:='http://user:%40pass@172.20.10.1:8081/video'` |

### Nodes (`ros2 run smartphone_robot <name>`)

| Node | What it does |
|---|---|
| `wander_node` | Reactive obstacle-avoidance explorer; auto-stops when `/map` growth plateaus and hands `/cmd_vel` to teleop. |
| `keyboard_teleop` | Arrow-key / WASD driving to `/cmd_vel`; silent until the first key. |
| `drive_distance_server` | `DriveDistance` action server with feedback and cancel (open-loop `speed × time`). |
| `object_follower` | OpenCV HSV colour-blob visual servoing; centroid error drives `angular.z` / `linear.x`. Works on either the robot or the phone camera (topic remap only). |
| `object_detector` | YOLOv4-tiny via `cv2.dnn` on a camera topic; 80 COCO classes. |
| `phone_sensor_bridge` | Phone IMU + GPS over Phyphox HTTP into `sensor_msgs/Imu` + `NavSatFix`. Batch timestamp polling reconstructs the full 50 Hz. |
| `phone_camera_bridge` | Phone camera over MJPEG/RTSP into `sensor_msgs/Image`; downscales before serialisation. |
| `camera_viewer` | Standalone sizable image window for any image topic. |
| `localization_eval` | Logs AMCL error vs ground truth to CSV at 5 Hz. |
| `node_utils` | Not runnable — shared `run_node` lifecycle helper imported by the others. |

### Scripts (`bash scripts/<file>` or `python3 scripts/<file>`)

| Script | What it does | Command |
|---|---|---|
| `add_room.py` | Drop a `.dae`/`.obj`/`.stl` mesh, get a Gazebo world and a ready mapping command. Repairs OBJ normals. | `python3 scripts/add_room.py room.dae myroom` |
| `save_map.sh` | Export the live SLAM map to `maps/<name>.pgm` + `.yaml`. | `bash scripts/save_map.sh indoor` |
| `download_house.sh` | Fetch the OpenRobotics House world from Gazebo Fuel. | `bash scripts/download_house.sh` |
| `download_yolo.sh` | Fetch the YOLOv4-tiny weights (git-ignored). | `bash scripts/download_yolo.sh` |
| `experiment_beam_count.sh` | Controlled AMCL beam-count sweep against ground truth. | `bash scripts/experiment_beam_count.sh 30 180 360` |
| `stop_all.sh` | Kill every stray Gazebo / bridge process between runs. | `bash scripts/stop_all.sh` |
| `capture_rviz.sh` | Screenshot RViz on a headless Xvfb display. | `bash scripts/capture_rviz.sh` |
| `record_demo.sh` | Record an RViz demo video on Xvfb with ffmpeg. | `scripts/record_demo.sh <rviz> <out.mp4> <seconds>` |
| `bake_rviz_geometry.sh` | Splice RViz's own panel-layout blob back into a hand-written config. | `bash scripts/bake_rviz_geometry.sh` |
| `check_xml.py` | Catch the `--`-in-XML-comment trap across xacro/world files. | `python3 scripts/check_xml.py --fix` |

## Results

Measured in the 8 × 8 m `indoor.world`, all verified from live topic output:

| Result | Value |
|---|---|
| Simulated LiDAR | 360 rays, 10 Hz, 0.12–12 m, Gaussian noise σ = 0.01 m |
| Simulated camera | 1280 × 720 `rgb8`, 15 Hz |
| Simulated IMU | 50 Hz |
| Odometry | 30 Hz, full `odom → base_footprint` TF |
| **SLAM map** | 8.2 × 8.8 m at 0.05 m/cell; every wall, the L-partition, both pillars, three crates, the planter and even the four table legs reconstructed |
| **Localisation accuracy** | **76–83 % of laser returns land exactly on a mapped wall cell, 0 % fall outside the map, mean error 0.9–2.7 cm** (measured, see below) |
| **Navigation** | consecutive goals all `SUCCEEDED` with **0 recovery behaviours** and `Failed to make progress: 0`; final position error 0.16–0.20 m against a 0.20 m tolerance |

## The Smartphone as a Real Sensor

The rest of this project runs on simulated sensors. This part does not: an iPhone
publishes its real accelerometer, gyroscope and GPS into ROS 2, through a driver written
for this project.

```
phone (Phyphox)  --WiFi / HTTP-->  phone_sensor_bridge  --DDS-->  /phone/imu, /phone/gps
```

`phone_sensor_bridge` is a sensor driver in the ordinary sense: it speaks a link the rest
of the system knows nothing about and republishes what it reads as `sensor_msgs/Imu` and
`sensor_msgs/NavSatFix`. Downstream, nothing can tell it apart from a simulated or an
industrial IMU.

### The problem that made it interesting

A single HTTP round trip to the phone took a **median of 242 ms** — measured, not assumed.
Requesting one reading per request would have given about **4 Hz**, useless for an IMU.

Phyphox can also be asked for *every* sample newer than a given timestamp. One such
request, taking 150 ms, returned **108 samples spanning 2.1 s**. So the transport rate and
the sample rate are separable: poll slowly, receive batches, publish each sample with its
own timestamp.

Verified end to end by subscribing to the resulting topic:

| | Measured |
|---|---|
| Samples received | 624 in 12 s |
| **Rate reconstructed from timestamps** | **51.0 Hz** — the phone's full configured rate |
| Median acceleration magnitude, phone at rest | 9.76 m/s² (gravity is 9.81) |
| Range | 9.66 – 9.93 m/s² (sensor noise) |
| Angular velocity, phone at rest | max 0.04 rad/s |

The magnitude landing on gravity is the check that matters: it confirms the phone-to-ROS
axis remapping is right, not merely that numbers are arriving.

### The camera

A second phone stream, through `phone_camera_bridge`: IP Camera Lite serves the phone
camera over MJPEG (and RTSP), OpenCV's `VideoCapture` opens it, and each frame is
republished as `sensor_msgs/Image` on `/phone/camera/image_raw` — the same message
Gazebo's camera plugin produces. A background thread grabs frames while a timer publishes
the newest one, so a slow network drops frames rather than lagging further behind.

The bottleneck here was **message size, not the camera.** The phone sends 1920×1080; a raw
frame at that size is 6.2 MB, and 15 of them per second is 93 MB/s, which DDS could not
sustain — throughput at the subscriber collapsed to 1.9 Hz. Downscaling to 640 wide on the
publisher side, before serialisation, cut each frame to 0.7 MB and let the full ~10 Hz the
phone delivers through. The simulated camera is 640×480 anyway, so nothing is lost.

The stream is password protected. The URL, with credentials, is a launch parameter — never
in the source, never in git — and credentials are masked in the logs.

### Things that only showed up against real hardware

- **iOS serves Phyphox on port 80, not the documented 8080.** Port 8080 was refused.
  Checked rather than assumed.
- **JSON has one number type.** Phyphox sends a reading that happens to be exactly zero
  as `0`, which Python parses as an `int`. Assigning an int into a `geometry_msgs/Vector3`
  does not raise a Python exception — it fails inside the C extension with
  `Assertion 'PyFloat_Check(field)' failed` and aborts the process. It therefore survives
  every test where the phone is moving, and crashes the moment it is set down flat.
- **Phyphox keeps the whole recording.** Connecting to a session already running returned
  21096 backlogged samples at once. The bridge now moves its cursor to the newest sample
  on connect without publishing, so consumers are not handed a burst of stale readings
  stamped as if they had just arrived.
- **Restarting the experiment resets its clock to zero** and announces this only by
  changing a session id. Without watching for that, the bridge would keep asking for
  samples newer than a timestamp that will never come again, and go silent with no error.

### Honest limitations

- Absolute latency is **not** corrected. Between 50 and 400 ms passes between a
  measurement and this node seeing it, and it varies. Relative timing within a batch is
  correct; absolute timing is not. Good enough to observe motion, not good enough to fuse
  into odometry — that needs real time synchronisation.
- The axis mapping assumes the phone lies flat, screen up, top edge forward. Hold it
  differently and the mapping is wrong. On a robot this is what the URDF's `imu_link`
  exists to express.
- Orientation is not published (`orientation_covariance[0] = -1`). Phyphox streams raw
  sensors; inventing an orientation from them would be a fusion step, not a measurement.
- **The phone observes the room you are in; the robot drives in the simulated one.** They
  are different places. This is real sensor data entering a robotics stack, not a phone
  mapping your house.

## Closing the Loop: the Phone Steers the Robot

Until this point the phone's data was published but nothing consumed it. `object_follower`
closes that gap — the piece that makes the project more than a simulation demo.

```
phone camera -> phone_camera_bridge -> /phone/camera/image_raw
                                              |
                                        object_follower  (OpenCV HSV blob detection)
                                              |  /cmd_vel
                                              v
                                     Gazebo differential drive
```

Hold a brightly coloured object in front of the phone and the simulated robot turns to
face it and drives toward it; take it away and it stops. This is **visual servoing** —
closing a control loop through a camera — the same principle a real robot uses to approach
a dock or track a person, with a colour blob standing in for a learned detector.

Detection is an HSV colour threshold: the largest blob within a hue band, its centroid
normalised to [-1, 1] across the frame. That error drives `angular.z` (turn to centre the
object) and `linear.x` (approach until the blob fills enough of the frame). Verified end to
end: with the target at frame x = +0.60 the node commanded `angular.z = -1.0`, and the
robot's true heading, read from ground truth, rotated accordingly.

It is honest about being classical CV, not learning: it fails in poor light, against a
same-coloured background, or under motion blur. Swapping in a neural detector would change
only this node — the servoing, the topics and the robot are untouched.

The same node also runs entirely in simulation, following a coloured target with the
robot's OWN camera instead of the phone's. Only the input topic changes — the detection
and control code is identical:

```bash
ros2 launch smartphone_robot follow_sim.launch.py
```

The robot starts in one corner, sees the red post across the room, drives to it and stops
when it fills the frame — verified: from spawn at x=-3.0 it reached x=-0.45, with the post
centred (target x=-0.00) at 10% of the frame. That the identical `object_follower` works on
either camera, changed by a launch remapping alone, is the decoupling principle made
concrete: perception does not know or care where its images come from.

![Following, in RViz](screenshots/10_follow_rviz.png)
*One RViz window: the robot's camera with the detection overlay (top left), the robot, its
travelled path in blue, and the live laser scan in red. Fixed frame is `odom` — this demo
runs no SLAM or AMCL, so there is no `map` frame, and a config expecting one shows only an
empty grid.*

![Robot camera detection](screenshots/10_robot_camera_detection.png)
*What the robot sees through its own camera: the red target centred, ringed by the
detector. `target x=-0.00 area=10.1%` — centred and close, so the robot stops.*

![Following, overhead](screenshots/10_follow_overview.png)
*The same moment from above: the robot (blue) has driven across the room to the red post.*

Run it with the phone instead (reactive mode; do not run Nav2 at the same time, both drive /cmd_vel):

```bash
ros2 launch smartphone_robot phone_follow.launch.py \
    stream_url:='http://user:pass@172.20.10.1:8081/video'
```

## Measuring, Not Demonstrating

A demonstration answers "does it work?" — and a yes tells you very little, least of all
whether a change made things better or worse. The most useful thing simulation offers is
something hardware cannot: **ground truth.** Gazebo knows exactly where the robot is.
On a real robot, learning that needs a motion-capture rig costing more than the robot.

`localization_eval` exploits that. It subscribes to what AMCL *believes*
(`/amcl_pose`) and to where the robot *is* (`/ground_truth/odom`, published by an
`OdometryPublisher` attached to the model), transforms the truth into the map frame, and
logs the difference at 5 Hz to CSV.

Ground truth is deliberately kept out of `/tf` and off `/odom`. A robot that quietly
navigated on its true pose would look flawless and prove nothing.

### Experiment: how many laser beams does AMCL need?

AMCL subsamples the LiDAR's 360 returns down to `max_beams` before matching them against
the map. `scripts/experiment_beam_count.sh` runs the **same four navigation goals** for
each setting, with the map, world, spawn pose and controller all held fixed, and records
the error against ground truth.

| `max_beams` | samples | position mean | median | 95th pct | heading mean |
|---|---|---|---|---|---|
| 30 | 596 | 9.7 cm | 9.1 cm | 18.9 cm | 4.70° |
| 180 | 711 | 9.2 cm | 7.9 cm | 19.7 cm | 5.14° |
| 360 | 712 | **8.3 cm** | **7.7 cm** | **17.0 cm** | **4.44°** |

All twelve goals succeeded, in every configuration.

**The result is mostly a negative one, and that is the point.** Using all 360 returns
instead of 30 improves mean position error by about 15 %, from 9.7 cm to 8.3 cm — real,
monotonic, and far smaller than expected. Between 30 and 180 the difference is within
run-to-run spread. In a room this rich in features, beam count is simply not the limiting
factor; at roughly two map cells, the error is dominated by the 5 cm map resolution and
the quality of the SLAM map itself.

> **A correction.** An earlier version of this README claimed that raising `max_beams`
> from 60 to 180 cut mean error from 7.9 cm to 0.9 cm. That claim came from comparing two
> runs that differed in *many* ways at once — different map, different world, different
> SLAM keyframe density, different AMCL motion-noise parameters — and attributing the
> whole improvement to one of them. The controlled experiment above does not support it.
> The claim is withdrawn; the measurement tooling that disproved it is the more useful
> result.

### A second check: does the laser line up with the map?

An independent check on the same question. Each return is transformed into the map frame
and the occupancy grid is sampled at that point:

| | Value |
|---|---|
| Returns landing exactly on a mapped wall cell | 76–83 % |
| Within one cell (≤ 10 cm) | 17–41 % |
| Falling outside the map | **0 %** |
| Mean error | 0.9 cm stationary, 2.7 cm while driving |

This agrees with the ground-truth figures: the residual is about one map cell, which is
the floor set by a 5 cm grid rather than a defect in the estimator. AMCL's own claimed
standard deviation (~10 cm) also tracks its true error closely, meaning the filter is
well calibrated — it is neither overconfident nor needlessly vague.

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
- **"The laser does not line up with the walls" had two separate causes, and the first
  diagnosis was wrong.** The obvious suspect was AMCL's beam count; a controlled sweep
  later showed beam count barely matters here (see Measuring, Not Demonstrating). The
  second cause was not a localisation problem at all: RViz draws the map at z = 0 but
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
- **The two phone streams cannot run at once.** iOS only lets the foreground app hold a
  network server, so Phyphox (IMU) and IP Camera Lite (camera) run one at a time, not
  together. On a real robot each sensor is its own device; here they share one, so they
  are demonstrated separately, or recorded to separate rosbags and replayed together.
- **`DriveDistance` measures distance open-loop**, as `speed × elapsed_time`. It assumes no
  wheel slip and no acceleration ramp. It should subscribe to `/odom` instead; the action
  *interface* would not change, only the implementation.
- **Tuned for one room.** Nav2's costmap and controller parameters were set for an 8 × 8 m indoor
  world and this robot's geometry. They are not general-purpose defaults.
- **Host resources are the binding constraint.** The development machine has 7.4 GB RAM and
  an integrated GPU, so Gazebo usually runs headless and transform tolerances are raised
  above their textbook values to survive scheduling lag.

## Future Improvements

1. **Closed-loop `DriveDistance`** using `/odom` instead of dead reckoning.
2. **Sensor fusion** — `robot_localization` combining wheel odometry with IMU, which is
   where the phone's real IMU becomes genuinely useful.
3. **`rosbag2` datasets** so perception can be tested repeatably without the simulator.
4. **A detector that steers the robot** — `object_detector` publishes YOLO detections
   today, but nothing yet turns a detection into a `/cmd_vel`. The colour servoing in
   `object_follower` already proves the loop; swapping the blob for a YOLO class is the step.
5. **Port to hardware** — the point of the architecture. Replacing `ros_gz_bridge` with real
   drivers should leave SLAM, Nav2 and the custom nodes untouched.

## License

Apache-2.0
