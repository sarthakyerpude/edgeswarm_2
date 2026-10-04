"""webots_ros2_driver plugin reproducing the EdgeSwarm per-robot sim contract.

Attached per robot by resource/amr_webots.urdf. Subscribes geometry_msgs/Twist
on <ns>/cmd_vel (raw Twist — Jazzy's diff_drive_controller is TwistStamped-only,
which is why this plugin exists), drives the two wheel motors, integrates
wheel-encoder odometry (NOT ground truth — matches Gazebo DiffDrive semantics
and the no-god-mode constraint), and publishes:
  <ns>/odom  nav_msgs/Odometry, 30 Hz, odom -> base_footprint
  <ns>/tf    tf2_msgs/TFMessage, the same odom -> base_footprint transform
A cmd_vel watchdog zeroes the target after CMD_TIMEOUT of silence, and a
stopped robot has its wheels locked in position control (released on the
next non-zero command) so it stays exactly where it stopped.
The namespace is the Webots robot name (robot_1..robot_3), so one URDF serves
the whole fleet. Scan publishing is handled by webots_ros2's Ros2Lidar device
plugin configured in the same URDF.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

WHEEL_RADIUS = 0.05
WHEEL_SEPARATION = 0.38
# 16 ms control steps: 0.031 s -> publish every 2nd step (~31 Hz).
ODOM_PERIOD = 0.031
# Acceleration limits matching the Gazebo DiffDrive plugin, applied as a slew
# rate on the commanded body velocities. Without them the robots move far more
# abruptly than in Gazebo and P2P permit races end in contact.
MAX_LIN_ACC = 2.0
MAX_ANG_ACC = 4.0
# No cmd_vel for this long (sim seconds) -> target velocity zero. The
# velocity_gate republishes at 20 Hz, so silence means the gate is gone.
CMD_TIMEOUT = 0.5
# Below this |v| / |w| a command counts as "stop" for the wheel hold.
STOP_EPS = 1e-3
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'


class AmrDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.__left_motor = self.__robot.getDevice('wheel_left_motor')
        self.__right_motor = self.__robot.getDevice('wheel_right_motor')
        for motor in (self.__left_motor, self.__right_motor):
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        self.__max_wheel_speed = self.__left_motor.getMaxVelocity()

        self.__left_sensor = self.__robot.getDevice('wheel_left_sensor')
        self.__right_sensor = self.__robot.getDevice('wheel_right_sensor')
        self.__left_sensor.enable(self.__timestep)
        self.__right_sensor.enable(self.__timestep)
        self.__last_left = None
        self.__last_right = None

        # Heading from the gyro, distance from the encoders: wheel slip under
        # contact (a shove, or spinning against a peer) otherwise goes straight
        # into yaw and AMCL follows it — the main divergence mechanism seen in
        # the supervisor logs. Optional so an older world without it still runs.
        self.__gyro = self.__robot.getDevice('gyro')
        if self.__gyro is not None:
            self.__gyro.enable(self.__timestep)

        self.__x = 0.0
        self.__y = 0.0
        self.__yaw = 0.0
        self.__v = 0.0
        self.__w = 0.0
        self.__target_v = 0.0
        self.__target_w = 0.0
        self.__cmd_v = 0.0
        self.__cmd_w = 0.0
        self.__last_cmd_time = None
        self.__held = False
        self.__last_pub_time = -ODOM_PERIOD

        if not rclpy.ok():
            rclpy.init(args=None)
        # Namespace = Webots robot name, so the shared URDF yields
        # /robot_N/{cmd_vel,odom,tf} per instance.
        self.__node = rclpy.create_node('amr_driver',
                                        namespace=self.__robot.getName())
        self.__node.create_subscription(Twist, 'cmd_vel', self.__on_cmd_vel, 1)
        self.__odom_pub = self.__node.create_publisher(Odometry, 'odom', 10)
        self.__tf_pub = self.__node.create_publisher(TFMessage, 'tf', 10)
        # One robot owns the global sim clock (the webots_ros2 Ros2Supervisor
        # is not used: its connect/die cycling under the Windows+WSL TCP setup
        # repeatedly stalled the simulation for every other controller).
        self.__clock_pub = None
        if self.__robot.getName() == 'robot_1':
            self.__clock_pub = self.__node.create_publisher(Clock, '/clock', 10)

    def __on_cmd_vel(self, msg):
        self.__target_v = msg.linear.x
        self.__target_w = msg.angular.z
        self.__last_cmd_time = self.__robot.getTime()

    def __hold_wheels(self):
        """Lock both wheels at their current encoder angle (position control).

        A velocity-controlled wheel at 0 rad/s still lets the body creep when
        pushed and spins freely against a wall; position control resists both.
        Odometry is unaffected: it integrates the encoders, which keep
        counting across the mode switch.
        """
        left = self.__left_sensor.getValue()
        right = self.__right_sensor.getValue()
        if math.isnan(left) or math.isnan(right):
            return
        # In position mode setVelocity is the max speed used to reach the
        # target; it must be non-zero for the PID to hold against a push.
        for motor, angle in ((self.__left_motor, left),
                             (self.__right_motor, right)):
            motor.setPosition(angle)
            motor.setVelocity(self.__max_wheel_speed)
        self.__cmd_v = 0.0
        self.__cmd_w = 0.0
        self.__held = True

    def __release_wheels(self):
        for motor in (self.__left_motor, self.__right_motor):
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        self.__held = False

    def __apply_drive(self, now, dt):
        if (self.__last_cmd_time is not None
                and now - self.__last_cmd_time > CMD_TIMEOUT):
            self.__target_v = 0.0
            self.__target_w = 0.0
        stop_target = (abs(self.__target_v) < STOP_EPS
                       and abs(self.__target_w) < STOP_EPS)
        if self.__held:
            if stop_target:
                return
            self.__release_wheels()
        elif (stop_target and abs(self.__cmd_v) < STOP_EPS
                and abs(self.__cmd_w) < STOP_EPS):
            self.__hold_wheels()
            if self.__held:
                return
        dv = max(-MAX_LIN_ACC * dt,
                 min(MAX_LIN_ACC * dt, self.__target_v - self.__cmd_v))
        dw = max(-MAX_ANG_ACC * dt,
                 min(MAX_ANG_ACC * dt, self.__target_w - self.__cmd_w))
        self.__cmd_v += dv
        self.__cmd_w += dw
        left = (self.__cmd_v - self.__cmd_w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        right = (self.__cmd_v + self.__cmd_w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        limit = self.__max_wheel_speed
        self.__left_motor.setVelocity(max(-limit, min(limit, left)))
        self.__right_motor.setVelocity(max(-limit, min(limit, right)))

    def __integrate(self, dt):
        left = self.__left_sensor.getValue()
        right = self.__right_sensor.getValue()
        if math.isnan(left) or math.isnan(right):
            return
        if self.__last_left is None:
            self.__last_left = left
            self.__last_right = right
            return
        dl = (left - self.__last_left) * WHEEL_RADIUS
        dr = (right - self.__last_right) * WHEEL_RADIUS
        self.__last_left = left
        self.__last_right = right
        d = (dl + dr) / 2.0
        dyaw = (dr - dl) / WHEEL_SEPARATION
        if self.__gyro is not None and dt > 0.0:
            try:
                rate = self.__gyro.getValues()[2]
            except (ValueError, SystemError, IndexError, TypeError):
                rate = float('nan')
            if not math.isnan(rate):
                dyaw = rate * dt
        self.__x += d * math.cos(self.__yaw + dyaw / 2.0)
        self.__y += d * math.sin(self.__yaw + dyaw / 2.0)
        self.__yaw = math.atan2(math.sin(self.__yaw + dyaw),
                                math.cos(self.__yaw + dyaw))
        if dt > 0.0:
            self.__v = d / dt
            self.__w = dyaw / dt

    def __publish(self, now):
        sec = int(now)
        nanosec = int((now - sec) * 1e9)
        qz = math.sin(self.__yaw / 2.0)
        qw = math.cos(self.__yaw / 2.0)

        odom = Odometry()
        odom.header.stamp.sec = sec
        odom.header.stamp.nanosec = nanosec
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = self.__x
        odom.pose.pose.position.y = self.__y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.__v
        odom.twist.twist.angular.z = self.__w
        self.__odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp.sec = sec
        tf.header.stamp.nanosec = nanosec
        tf.header.frame_id = ODOM_FRAME
        tf.child_frame_id = BASE_FRAME
        tf.transform.translation.x = self.__x
        tf.transform.translation.y = self.__y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.__tf_pub.publish(TFMessage(transforms=[tf]))

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        now = self.__robot.getTime()
        if self.__clock_pub is not None:
            clock = Clock()
            clock.clock.sec = int(now)
            clock.clock.nanosec = int((now - int(now)) * 1e9)
            self.__clock_pub.publish(clock)
        self.__apply_drive(now, self.__timestep / 1000.0)
        self.__integrate(self.__timestep / 1000.0)
        if now - self.__last_pub_time >= ODOM_PERIOD:
            self.__last_pub_time = now
            self.__publish(now)
