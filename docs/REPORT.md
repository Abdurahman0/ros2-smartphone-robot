# ROS 2 Smartphone-Based Autonomous Robotics System
### Project Report

**Author:** Abdurahmon · **Contact:** cognilabsco@gmail.com
**Platform:** ROS 2 Jazzy · Gazebo Harmonic · Python 3.12 · Docker on Kali Linux

---

## 1. Abstract

This project implements a complete autonomous mobile-robotics software stack — robot
description, physics simulation, SLAM mapping, and goal-directed navigation — and
integrates a smartphone as a source of *real* sensor data, all without any purpose-built
robot hardware. The autonomy runs in Gazebo; a smartphone contributes a real camera, IMU
and GPS over Wi-Fi, and a neural network recognises real objects in the phone's view.
The work emphasises two things a tutorial normally omits: **quantitative evaluation
against ground truth**, and an architecture designed so the same code transfers from
simulation to physical hardware.

## 2. Motivation and Objectives

Robotics education is usually gated behind expensive hardware. The objective was to build
and *understand* a full autonomy stack on a laptop alone, under three self-imposed
constraints:

1. **No purchased hardware** — no LiDAR, no chassis, no depth camera. A smartphone
   provides the one real sensor.
2. **Do not modify the host OS** — Kali Linux is not a supported ROS 2 platform, so the
   entire environment lives in a Docker container and nothing is installed on the host.
3. **Design for sim-to-real transfer** — every component communicates over standard ROS 2
   topics so that a simulated sensor can be replaced by a real driver without rewriting
   the consumers.

## 3. System Architecture

The system is a graph of independent ROS 2 nodes communicating over typed topics. No node
holds a direct reference to another; each publishes or subscribes to named channels.

```
Gazebo Harmonic ── gz-transport ── ros_gz_bridge ── DDS ── ROS 2 topics
  diff-drive plugin                                        /scan /odom /imu /tf
  simulated LiDAR / camera / IMU                                 │
                                                    ┌────────────┼───────────────┐
                                                 slam_toolbox  Navigation2   perception
                                                   -> /map      -> /cmd_vel   (OpenCV / YOLO)
```

**Coordinate frames** follow the ROS navigation convention
`map → odom → base_footprint → base_link → {wheels, lidar, camera, imu}`. `odom → base_footprint`
is smooth wheel odometry that drifts; `map → odom` is the drift correction published by
SLAM (while mapping) or AMCL (while navigating).

The decisive property is **interchangeability**: `slam_toolbox` subscribes to `/scan`
(`sensor_msgs/LaserScan`) and cannot tell whether a Gazebo GPU-LiDAR plugin or a physical
RPLidar produced it. This is what makes the simulation a genuine development environment
rather than a demonstration.

## 4. Implementation

### 4.1 Robot description
A differential-drive base is described in URDF/xacro (`robot.urdf.xacro`) with wheels, a
caster, a 2D LiDAR, a camera and an IMU, each with proper inertia so Gazebo physics is
stable. Camera frames are split into body (`camera_link`) and optical
(`camera_optical_link`) conventions to keep vision maths correct.

### 4.2 Simulation
`gazebo.xacro` adds the differential-drive controller and simulated sensors (360-ray
LiDAR at 10 Hz, 1280×720 camera at 15 Hz, IMU at 50 Hz). `ros_gz_bridge` (configured in
`gz_bridge.yaml`) translates between gz-transport and DDS. A ground-truth odometry plugin
publishes the robot's true pose for evaluation only — deliberately kept off `/tf` and
`/odom` so the navigation stack cannot consume it.

### 4.3 Mapping
`slam_toolbox` (async mode) performs pose-graph SLAM. A custom reactive node
(`wander_node`) explores the room from laser data alone, and **detects when the map has
stopped growing and stops the robot automatically**, handing control to the operator.

### 4.4 Navigation
Navigation2 provides AMCL localisation, a NavFn global planner, and a Regulated Pure
Pursuit local controller (chosen over DWB, which stalled in tight spaces on this robot).
Costmap inflation is tuned to the robot's 0.22 m radius.

### 4.5 Perception
Two complementary approaches, honest about their limits:
- **Classical** (`object_follower`): HSV colour-blob detection driving visual servoing.
- **Neural** (`object_detector`): YOLOv4-tiny (80 COCO classes) via OpenCV's DNN module,
  recognising real objects — person, laptop, chair, bottle — on the phone camera.

### 4.6 Smartphone integration
`phone_sensor_bridge` and `phone_camera_bridge` turn a smartphone into ROS 2 sensor
drivers: Phyphox streams IMU/GPS as HTTP JSON, IP Camera Lite streams MJPEG, and the
bridges republish both as standard `sensor_msgs`.

### 4.7 Software engineering
A single `node_utils.run_node` helper centralises the node lifecycle (signal handling,
spin loop, clean shutdown). The environment is a reproducible `Dockerfile`; helper scripts
handle screenshots, clean shutdown, an XML validator, and the evaluation experiment.

## 5. Evaluation and Results

Measured in the 8 × 8 m simulated room, verified from live topic data:

| Metric | Result |
|---|---|
| SLAM map | full room reconstructed at 5 cm resolution (~25,600 known cells) |
| Localisation accuracy (AMCL vs ground truth) | mean 8–9 cm, ~1–2 map cells |
| Navigation | consecutive goals reached, 0 recovery behaviours, ~0.17 m final error |
| Simulated sensors | LiDAR 10 Hz, camera 15 Hz, IMU 50 Hz, odometry 30 Hz |
| Phone IMU into ROS 2 | reconstructed 51 Hz over Wi-Fi; magnitude at rest = 9.76 m/s² (gravity) |
| Phone camera into ROS 2 | ~10 Hz after down-scaling to control DDS load |
| Object detection | real objects labelled live (person, keyboard, tvmonitor, …) |

### Controlled experiment: AMCL beam count
Using the ground-truth evaluator, the number of laser beams AMCL uses (`max_beams`) was
swept over {30, 180, 360} with everything else fixed:

| max_beams | mean position error | mean heading error |
|---|---|---|
| 30 | 9.7 cm | 4.70° |
| 180 | 9.2 cm | 5.14° |
| 360 | 8.3 cm | 4.44° |

**Result (largely negative, and stated as such):** beam count has only a ~15 % effect
here; at roughly two map cells the error is dominated by the 5 cm grid resolution, not by
the estimator. An earlier informal claim that this parameter cut error dramatically was
**withdrawn** after this controlled test — the measurement tooling disproving a hypothesis
is itself a result.

## 6. Limitations

Stated plainly, because they bound the interpretation of the results:

- **The robot is simulated.** Gazebo's sensor noise is Gaussian and clean; real LiDAR is
  defeated by glass and dark surfaces, real wheels slip, real timestamps jitter.
- **The phone observes the operator's real room; the robot drives in the simulated one.**
  They are different spaces. This is real sensor data entering a robotics stack, not a
  phone mapping a real room.
- **A single 2D camera cannot build a metric map** — that requires monocular visual SLAM,
  which is sparse, scale-ambiguous and out of scope. Mapping therefore uses LiDAR, which
  measures distance directly. This is a deliberate sensor-selection decision.
- **Phone sensor latency is uncorrected** (50–400 ms, variable), so the phone IMU is
  suitable for observation but not yet for fusion into odometry.
- **iOS allows only the foreground app a network server**, so the phone's IMU and camera
  apps run one at a time, not together.
- **Object detection is limited to YOLOv4-tiny's 80 trained classes** and degrades in poor
  lighting or at odd angles, as any such model does.

## 7. Future Work

1. Closed-loop sensor fusion (`robot_localization`) combining wheel odometry with the
   phone IMU.
2. Recording real sessions with `rosbag2` for repeatable offline testing.
3. Porting to physical hardware — the architecture's intended endpoint, expected to
   require replacing the Gazebo bridge rather than the autonomy code.

## 8. Conclusion

The project delivers a working, measured autonomy stack in simulation, integrates a real
smartphone as a sensor, and — most importantly — treats the result as something to
evaluate and be honest about rather than merely demonstrate. The topic-based architecture
keeps sensing, mapping, planning and perception decoupled, so the path from simulation to
a physical robot is a matter of swapping components, not rewriting the system.

---

*Environment reproducible from `docker/Dockerfile`. Full operating instructions in
`README.md`. Interview-preparation notes and CV wording in `docs/portfolio.md`.*
