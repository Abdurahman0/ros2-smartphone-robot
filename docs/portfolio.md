# Portfolio and CV wording

Every claim below is limited to what is actually implemented and verified in this
repository. Nothing here describes the smartphone bridge or the OpenCV node, because
those are not built yet. Overstating them is the fastest way to lose a technical
interview.

---

## One-line CV entry

> Built a ROS 2 (Jazzy) autonomous mobile-robotics system in simulation: URDF robot
> description, Gazebo sensor simulation, SLAM mapping and Navigation2 goal-directed
> navigation, with custom Python nodes, services and actions.

## Three-line project entry

> **ROS 2 Autonomous Mobile Robot (simulation)** — Python, ROS 2 Jazzy, Gazebo Harmonic, Nav2, Docker
> Modelled a differential-drive robot in URDF/xacro with simulated LiDAR, camera and IMU;
> used slam_toolbox to map an unknown 8 × 8 m environment (97.6 % coverage) and Navigation2
> with AMCL to reach commanded goals autonomously (0.17 m final error, no recovery
> behaviours). Wrote the custom nodes — a reactive explorer that auto-stops when the map is
> complete, a DriveDistance action server with feedback and cancel, and OpenCV/YOLO vision —
> and packaged the whole toolchain in Docker.

## Longer paragraph (personal statement or cover letter)

> I wanted to learn mobile robotics without access to hardware, so I built a complete ROS 2
> autonomy stack in simulation and treated the simulator as a stand-in for a real robot
> rather than as the end goal. I described a differential-drive base in URDF with a TF tree,
> simulated a 2D LiDAR, camera and IMU in Gazebo Harmonic, and bridged them into ROS 2 so
> that every downstream component consumes standard message types. On top of that I ran
> slam_toolbox to map an unseen room — driven by a reactive obstacle-avoidance node I wrote
> from raw laser data — and Navigation2 to localise against the saved map and drive to
> goals. Most of what I learned came from debugging: a lifecycle node that silently did
> nothing until a manager activated it, launch arguments leaking across included launch
> files, and duplicated bridge processes corrupting simulated time in a way that looked
> exactly like a controller tuning problem. Because the architecture keeps sensing,
> mapping and planning decoupled through topics, moving this to a physical robot should
> mean replacing the simulator bridge rather than the autonomy code — which is the part
> I most want to test next.

---

## Questions to be ready for

Short answers; the README's Engineering Notes section has the detail.

**What is ROS 2?** A middleware, not an operating system: libraries and conventions that let
many small single-purpose programs discover each other over a network and exchange typed
messages, plus tooling to inspect a running system. It uses DDS, so there is no master.

**What is a node? A topic?** A node is one process doing one job. A topic is a named, typed,
one-way broadcast channel. Publishers do not know their subscribers, and each subscriber
gets its own copy — a topic is not a queue.

**Why publish/subscribe?** Decoupling in four dimensions — time (nodes start in any order),
identity (a publisher survives a subscriber dying), location (nodes can be on different
machines) and implementation. The last one is why this project works: `/scan` carries
`sensor_msgs/LaserScan` whether Gazebo or an RPLidar driver produced it, so SLAM is
unchanged either way.

**What is `/cmd_vel`?** The standard velocity-command topic, `geometry_msgs/Twist`. A
differential-drive robot uses only `linear.x` (m/s) and `angular.z` (rad/s) because it
cannot move sideways or fly. Nav2 writes it; the motor driver reads it.

**How does LiDAR data reach navigation?** The driver (or Gazebo plugin) publishes
`LaserScan` on `/scan`: an array of ranges plus `angle_min` and `angle_increment` that say
which direction each one points. Nav2's costmap layers ray-trace those into occupied and
cleared cells; TF2 converts them from `lidar_link` into the map frame.

**What is TF2?** The transform system. It maintains a time-stamped tree of coordinate
frames so any measurement can be expressed in any frame. It is time-aware, which matters:
"where was the laser when this scan was taken", not "where is it now".

**How does SLAM work?** Wheel odometry drifts. slam_toolbox aligns each new scan against
previously collected scans (scan matching) to work out where the robot must be, builds an
occupancy grid, and publishes the correction as `map -> odom`. When it recognises a
previously visited place, loop closure re-optimises the whole pose graph and past error is
corrected retroactively.

**What does Navigation2 do?** Splits navigation in two: a global planner (NavFn) finds a
route across the static map, and a local controller (DWB) picks a velocity every 50 ms by
simulating and scoring candidate trajectories against the local costmap. Plus AMCL for
localisation, recovery behaviours, and a behaviour tree sequencing it all.

**What changes on a real robot?** The simulator bridge is replaced by real drivers, and the
`<gazebo>` blocks in the URDF become irrelevant. SLAM, Nav2 and my nodes stay as they are.
What does change is that the URDF's measurements must match the physical robot exactly,
sensor noise gets much messier, and the parameters — especially AMCL's motion noise and the
costmap inflation radius — need retuning against reality.
