#!/usr/bin/env python3
"""Turn a smartphone into a real ROS 2 sensor.

WHAT THIS IS
============
This node is a SENSOR DRIVER. It talks to hardware over a link the rest of the system
knows nothing about, and republishes what it reads as standard ROS 2 messages. That is
exactly what a LiDAR driver or a camera driver does; the only unusual part is that the
hardware here is a phone and the link is HTTP over WiFi.

Downstream, nothing can tell the difference. `sensor_msgs/Imu` on /phone/imu looks the
same whether it came from an industrial IMU, from Gazebo, or from an iPhone on a desk.

    phone (Phyphox app)  --WiFi/HTTP-->  this node  --DDS-->  /phone/imu, /phone/gps

HOW THE PHONE SERVES DATA
=========================
Phyphox (free, open source, from RWTH Aachen) exposes a running experiment over HTTP.
Two query forms matter:

    /get?accX                      the single newest value
    /get?accX=<t>|acc_time         every value whose acc_time is greater than <t>

THE SECOND FORM IS THE WHOLE TRICK, and the reason this node is usable at all.

Measured on this setup, one HTTP round trip to the phone takes a median of 242 ms. Asking
for one value per request would therefore yield about 4 Hz -- useless for an IMU. But a
single 150 ms request using the second form returned 108 samples spanning 2.1 s, i.e. the
full 49.8 Hz the sensor was actually recording.

So the transport rate and the sample rate are decoupled: poll slowly, receive batches,
and publish every sample with its own timestamp. Worth remembering generally -- the
bottleneck was per-request latency, not bandwidth.

AXES
====
Phyphox reports in the PHONE's frame: x across the screen to the right, y along the
screen towards the top, z out of the screen. ROS uses REP-103 for robot bodies: x
forward, y left, z up.

With the phone lying flat, screen up, its top edge pointing the way the robot would
drive, the mapping is:

    ROS x (forward) =  phone y
    ROS y (left)    = -phone x
    ROS z (up)      =  phone z

Sanity check that this is right: a phone lying still reads about +9.81 on phone z, and a
stationary ROS IMU should report +9.81 on its z. It does. Hold the phone differently and
this mapping is wrong -- which is the honest version of "sensor mounting matters", and on
a real robot is what the URDF's imu_link exists to express.

TIMESTAMPS, AND AN HONEST LIMITATION
====================================
Phyphox timestamps samples in seconds since the experiment started, not wall-clock time.
This node anchors the newest sample of each batch to the moment the reply arrived and
spaces the rest backwards by their true intervals, so the *relative* timing within a
batch is correct to the phone's own clock.

What is NOT corrected is absolute latency: somewhere between 50 and 400 ms passes between
a measurement happening and this node seeing it, and that delay is variable. For watching
a phone move, that is fine. For fusing this IMU into odometry it would not be -- a real
driver needs proper time synchronisation, and this one does not have it.
"""

import json
import math
import signal
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


class PhoneSensorBridge(Node):
    """Polls Phyphox over HTTP and republishes as sensor_msgs."""

    def __init__(self):
        super().__init__('phone_sensor_bridge')

        self.declare_parameter('host', '172.20.10.1')
        # Phyphox serves on 8080 on Android but on 80 on iOS. Checked, not assumed:
        # port 8080 was refused and port 80 answered.
        self.declare_parameter('port', 80)
        self.declare_parameter('poll_period', 0.4)     # [s] between HTTP requests
        self.declare_parameter('timeout', 3.0)         # [s] HTTP timeout
        self.declare_parameter('imu_frame_id', 'phone_imu_link')
        self.declare_parameter('gps_frame_id', 'phone_gps_link')
        self.declare_parameter('publish_gps', True)
        # Rough noise figures for a phone MEMS IMU. Not calibrated -- stated as a guess
        # rather than left at zero, because zero covariance means "perfect", and a filter
        # downstream would believe it.
        self.declare_parameter('accel_stddev', 0.10)   # [m/s^2]
        self.declare_parameter('gyro_stddev', 0.02)    # [rad/s]

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.timeout = self.get_parameter('timeout').value
        self.imu_frame = self.get_parameter('imu_frame_id').value
        self.gps_frame = self.get_parameter('gps_frame_id').value
        self.want_gps = self.get_parameter('publish_gps').value
        a = self.get_parameter('accel_stddev').value
        g = self.get_parameter('gyro_stddev').value
        self.accel_var, self.gyro_var = a * a, g * g

        self.base_url = f'http://{self.host}:{self.port}'

        # Sensor data uses BEST_EFFORT: a dropped sample should be skipped rather than
        # retransmitted, because by the time it arrives it describes the past.
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=50)
        self.imu_pub = self.create_publisher(Imu, 'phone/imu', sensor_qos)
        self.gps_pub = self.create_publisher(NavSatFix, 'phone/gps', sensor_qos)

        self.last_acc_t = 0.0
        self.last_gps_t = 0.0
        self.session = None
        self.primed = False
        self.imu_count = 0
        self.gps_count = 0
        self.fail_count = 0
        self.warned_offline = False

        self.create_timer(self.get_parameter('poll_period').value, self.poll)
        self.get_logger().info(f'phone_sensor_bridge polling {self.base_url}')

    # ----------------------------------------------------------------- transport

    def fetch(self, query: str):
        try:
            with urllib.request.urlopen(f'{self.base_url}/get?{query}',
                                        timeout=self.timeout) as response:
                data = json.load(response)
            if self.warned_offline:
                self.get_logger().info('Phone reachable again.')
                self.warned_offline = False
            self.fail_count = 0
            return data
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            self.fail_count += 1
            if not self.warned_offline and self.fail_count >= 3:
                self.get_logger().warn(
                    f'No reply from {self.base_url}: {exc}. '
                    f'Is the phone on the same network, the Phyphox experiment RUNNING, '
                    f'and remote access enabled?'
                )
                self.warned_offline = True
            return None

    # -------------------------------------------------------------------- polling

    def poll(self):
        """One HTTP request for everything new since the last one."""
        fields = [
            f'acc_time={self.last_acc_t:.4f}|acc_time',
            f'accX={self.last_acc_t:.4f}|acc_time',
            f'accY={self.last_acc_t:.4f}|acc_time',
            f'accZ={self.last_acc_t:.4f}|acc_time',
            f'gyroX={self.last_acc_t:.4f}|acc_time',
            f'gyroY={self.last_acc_t:.4f}|acc_time',
            f'gyroZ={self.last_acc_t:.4f}|acc_time',
        ]
        if self.want_gps:
            fields += [
                f'gps_time={self.last_gps_t:.4f}|gps_time',
                f'gpsLat={self.last_gps_t:.4f}|gps_time',
                f'gpsLon={self.last_gps_t:.4f}|gps_time',
                f'gpsZ={self.last_gps_t:.4f}|gps_time',
                f'gpsAccuracy={self.last_gps_t:.4f}|gps_time',
            ]

        data = self.fetch('&'.join(fields))
        if data is None:
            return

        # Phyphox restarts its clock at zero whenever the experiment is restarted, and
        # announces this only by changing the session id. Without noticing, this node
        # would keep asking for samples newer than the old (large) timestamp, receive
        # nothing forever, and report no error at all -- a silent stall that looks like
        # the phone has gone offline.
        status = data.get('status', {})
        session = status.get('session')
        if session is not None and session != self.session:
            if self.session is not None:
                self.get_logger().warn(
                    f'Phyphox session changed ({self.session} -> {session}); '
                    f'the experiment was restarted, so resetting the timestamp cursor.'
                )
            self.session = session
            self.last_acc_t = 0.0
            self.last_gps_t = 0.0
            self.primed = False

        if not status.get('measuring', False):
            self.get_logger().warn(
                'Phyphox is not measuring. Press play in the app.',
                throttle_duration_sec=10.0,
            )
            return

        buffers = data.get('buffer', {})

        # First successful poll: move the cursor to the newest sample WITHOUT publishing.
        #
        # Phyphox keeps everything the experiment has recorded, so connecting to a session
        # that has been running for a few minutes returns tens of thousands of samples at
        # once -- 18022 of them, in the first test. Every one is stale, and dumping them
        # into /phone/imu in a single burst would hand any consumer a flood of readings
        # timestamped as if they had just happened.
        if not self.primed:
            newest_acc = self.series(buffers, 'acc_time')
            newest_gps = self.series(buffers, 'gps_time')
            if newest_acc:
                self.last_acc_t = newest_acc[-1]
            if newest_gps:
                self.last_gps_t = newest_gps[-1]
            self.primed = True
            self.get_logger().info(
                f'Connected. Skipping {len(newest_acc)} samples already in the phone\'s '
                f'buffer and starting from now.'
            )
            return

        self.publish_imu(buffers)
        if self.want_gps:
            self.publish_gps(buffers)

    @staticmethod
    def series(buffers, name):
        """Extract one buffer as a list of floats.

        The float() is not cosmetic. JSON has one number type, so Phyphox sends a value
        that happens to be a whole number as `0`, and Python parses that as an int.
        Assigning an int into a geometry_msgs/Vector3 field does not raise a Python
        TypeError -- it fails inside the C extension with

            geometry_msgs__msg__vector3__convert_from_py:
            Assertion `PyFloat_Check(field)` failed.

        and aborts the whole process. It only happens once a reading is exactly zero, so
        it survives every test where the phone is moving and dies the moment it is set
        down flat.
        """
        entry = buffers.get(name) or {}
        values = []
        for v in (entry.get('buffer') or []):
            if v is None:
                continue
            try:
                values.append(float(v))
            except (TypeError, ValueError):
                continue
        return values

    def publish_imu(self, buffers):
        times = self.series(buffers, 'acc_time')
        ax = self.series(buffers, 'accX')
        ay = self.series(buffers, 'accY')
        az = self.series(buffers, 'accZ')
        gx = self.series(buffers, 'gyroX')
        gy = self.series(buffers, 'gyroY')
        gz = self.series(buffers, 'gyroZ')

        # The accelerometer and gyroscope are separate hardware and their buffers can
        # differ in length by a sample. Truncating to the shortest keeps them aligned;
        # it costs at most one sample and avoids pairing readings from different moments.
        n = min(len(times), len(ax), len(ay), len(az), len(gx), len(gy), len(gz))
        if n == 0:
            return

        now = self.get_clock().now()
        newest = times[n - 1]

        for i in range(n):
            msg = Imu()
            # Place each sample relative to the newest one, which is anchored to the
            # moment this reply arrived. Absolute latency is not corrected -- see the
            # module docstring.
            age_ns = int((newest - times[i]) * 1e9)
            msg.header.stamp = (now - rclpy.duration.Duration(nanoseconds=age_ns)).to_msg()
            msg.header.frame_id = self.imu_frame

            # Phone axes -> ROS REP-103 body axes.
            msg.linear_acceleration.x = ay[i]
            msg.linear_acceleration.y = -ax[i]
            msg.linear_acceleration.z = az[i]
            msg.angular_velocity.x = gy[i]
            msg.angular_velocity.y = -gx[i]
            msg.angular_velocity.z = gz[i]

            # -1 in the first element is the ROS convention for "this message carries no
            # orientation". A phone could estimate one, but Phyphox is publishing raw
            # sensors here and inventing an orientation would be dishonest.
            msg.orientation_covariance[0] = -1.0
            for k in (0, 4, 8):
                msg.linear_acceleration_covariance[k] = self.accel_var
                msg.angular_velocity_covariance[k] = self.gyro_var

            self.imu_pub.publish(msg)

        self.last_acc_t = newest
        self.imu_count += n
        self.get_logger().info(
            f'IMU: +{n} samples (total {self.imu_count}), '
            f'a=({ay[n-1]:+.2f}, {-ax[n-1]:+.2f}, {az[n-1]:+.2f}) m/s^2',
            throttle_duration_sec=2.0,
        )

    def publish_gps(self, buffers):
        times = self.series(buffers, 'gps_time')
        lat = self.series(buffers, 'gpsLat')
        lon = self.series(buffers, 'gpsLon')
        alt = self.series(buffers, 'gpsZ')
        acc = self.series(buffers, 'gpsAccuracy')

        n = min(len(times), len(lat), len(lon))
        if n == 0:
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.gps_frame
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat[n - 1]
        msg.longitude = lon[n - 1]
        msg.altitude = alt[n - 1] if len(alt) >= n else 0.0

        # Phyphox reports horizontal accuracy in metres; ROS wants a variance.
        if len(acc) >= n and acc[n - 1] > 0.0:
            var = acc[n - 1] ** 2
            msg.position_covariance[0] = var
            msg.position_covariance[4] = var
            msg.position_covariance[8] = (acc[n - 1] * 3.0) ** 2   # vertical is worse
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.gps_pub.publish(msg)
        self.last_gps_t = times[n - 1]
        self.gps_count += 1


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PhoneSensorBridge()

    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.get_logger().info(
                f'Shutting down after {node.imu_count} IMU and {node.gps_count} GPS messages.'
            )
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
