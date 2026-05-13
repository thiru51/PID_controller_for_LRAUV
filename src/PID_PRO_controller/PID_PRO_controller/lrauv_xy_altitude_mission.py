#!/usr/bin/env python3
"""
LRAUV XY Altitude Mission Node
--------------------------------
Derived structurally from your 3D LOS controller.

Features:
✔ Projection-based XY LOS (proper geometry)
✔ Cascaded depth → pitch → fin control
✔ Altimeter-based altitude hold
✔ FLS beam clustering via regression
✔ Adaptive lookahead based on slope
✔ Emergency override
✔ Sensor validation + interactive start
✔ Debug topic publishing for plotting
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


import numpy as np
import yaml
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory



pkg_path = get_package_share_directory('PID_PRO_controller')

file_path = os.path.join(
    pkg_path,
    'config',
    'waypoints.yaml'
)


# -------------------------
# GAINS (same philosophy as 3D controller)
# -------------------------

MIN_LOOKAHEAD = 6.0
MAX_LOOKAHEAD = 25.0
SLOPE_LOOKAHEAD_GAIN = 15.0

KP_YAW = 9.0
KD_YAW = 0.5
MAX_VERT_FIN_RAD = 0.52

KP_DEPTH = 0.12
KD_DEPTH = 0.06
MAX_PITCH_RAD = 0.45

KP_PITCH = 6.0
KD_PITCH = 0.8
MAX_HORIZ_FIN_RAD = 0.52

KP_SURGE = 500.0
DEFAULT_CRUISE_SPEED = -1.0

ALTITUDE_FILTER_ALPHA = 0.7


# -------------------------
# Utilities (exact structure preserved)
# -------------------------

def clamp(x, lo, hi):
    if x != x:
        return lo
    return max(lo, min(hi, x))

def angle_wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def euler_from_quaternion(q):
    x, y, z, w = q

    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = clamp(t2, -1.0, 1.0)
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


# -------------------------
# Controller Node
# -------------------------

class XYAltitudeMission(Node):

    def __init__(self):
        super().__init__('xy_altitude_mission')

        # Publishers
        self.vert_pub = self.create_publisher(Float64, '/cmd/vertical_fins', 10)
        self.horiz_pub = self.create_publisher(Float64, '/cmd/horizontal_fins', 10)

        # self.horiz_port_pub = self.create_publisher(
        #     Float64,
        #     '/cmd/horizontal_fin_port',
        #     10
        # )
        # self.horiz_star_pub = self.create_publisher(
        #     Float64,
        #     '/cmd/horizontal_fin_starboard',
        #     10
        # )
        # self.thrust_pub = self.create_publisher(Float64, '/cmd/thrust', 10)

        # Debug publishers
        self.debug_alt_error = self.create_publisher(Float64, '/debug/altitude_error', 10)
        self.debug_zd = self.create_publisher(Float64, '/debug/z_d', 10)
        self.debug_slope = self.create_publisher(Float64, '/debug/slope', 10)
        self.debug_lookahead = self.create_publisher(Float64, '/debug/lookahead', 10)
        self.debug_theta_d = self.create_publisher(Float64, '/debug/theta_d', 10)
        self.debug_pitch = self.create_publisher(Float64, '/debug/pitch', 10)
        self.debug_fin = self.create_publisher(Float64, '/debug/horiz_fin', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(LaserScan, '/altimeter/scan', self.alt_cb, 10)
        self.create_subscription(LaserScan, 'forward_looking_sonar/scan', self.fls_cb, 10)

        # Internal state
        self.h_now = None
        self.fls_msg = None
        self.z_d_filtered = None
        self.mission_started = False

        self.load_waypoints()

        threading.Thread(target=self.wait_for_sensors, daemon=True).start()

        self.get_logger().info("XY Altitude Mission Node Ready")

    # -------------------------
    # Waypoint Loader
    # -------------------------

    def load_waypoints(self):

        # file_path = "/home/thiru/ros2_ws/src/PID_PRO_controller/PID_PRO_controller/waypoints.yaml"

        if not os.path.exists(file_path):
            self.get_logger().error("Waypoints file not found.")
            return

        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)

        self.desired_altitude = float(data['desired_altitude'])
        self.emergency_altitude = float(data['emergency_altitude'])
        self.waypoints = np.array(data['waypoints'], dtype=float)

        self.current_idx = 0

        print("\n--- ALTITUDE MISSION CONFIG ---")
        print(f"Desired Altitude: {self.desired_altitude} m")
        print(f"Emergency Altitude: {self.emergency_altitude} m")
        print(f"Waypoints loaded: {len(self.waypoints)}\n")

    # -------------------------
    # Sensor Callbacks
    # -------------------------

    def alt_cb(self, msg):
        if msg.ranges:
            self.h_now = msg.ranges[0]

    def fls_cb(self, msg):
        self.fls_msg = msg

    # -------------------------
    # Sensor Validation
    # -------------------------

    def wait_for_sensors(self):

        print("Waiting for altimeter and FLS...")

        while rclpy.ok():
            if self.h_now is not None and self.fls_msg is not None:
                valid = sum(not math.isinf(r) for r in self.fls_msg.ranges)
                print(f"✔ Altimeter OK")
                print(f"✔ FLS OK ({valid} valid beams)")
                break
            time.sleep(1)

        print("Mission started.\n")
        self.mission_started = True

    # -------------------------
    # Proper XY LOS (projection-based)
    # -------------------------

    def compute_los_target_xy(self, wp_start, wp_end, curr_pos):

        x1, y1 = wp_start
        x2, y2 = wp_end
        x3, y3 = curr_pos[0], curr_pos[1]

        dx = x2 - x1
        dy = y2 - y1

        line_len_squared = dx*dx + dy*dy

        if line_len_squared < 1e-6:
            proj = np.array([x1, y1])
            l = 0.0
        else:
            l = ((dx)*(x3 - x1) + (dy)*(y3 - y1)) / line_len_squared
            proj = np.array([x1 + l*dx, y1 + l*dy])

        d2 = float(np.linalg.norm(proj - np.array([x2, y2])))
        dist_to_goal = float(np.linalg.norm(
            np.array([x3, y3]) - np.array([x2, y2])
        ))

        delta = max(min(MAX_LOOKAHEAD, dist_to_goal), MIN_LOOKAHEAD)

        if d2 < 1e-6:
            target = np.array([x2, y2])
        else:
            target = ((d2 - delta) * proj +
                      delta * np.array([x2, y2])) / d2

        return target, dist_to_goal, l

    # -------------------------
    # FLS Regression (Beam Clustering)
    # -------------------------

    def estimate_slope(self):

        if self.fls_msg is None:
            return 0.0

        msg = self.fls_msg

        angles = np.linspace(msg.angle_min,
                             msg.angle_max,
                             len(msg.ranges))

        xs = []
        zs = []

        for r, theta in zip(msg.ranges, angles):
            if not math.isinf(r):
                x = r * math.cos(theta)
                z = r * math.sin(theta)
                xs.append(x)
                zs.append(z)

        if len(xs) < 20:
            return 0.0

        xs = np.array(xs)
        zs = np.array(zs)

        A = np.vstack([xs, np.ones(len(xs))]).T
        slope, _ = np.linalg.lstsq(A, zs, rcond=None)[0]

        return slope

    # -------------------------
    # Main Control Loop
    # -------------------------

    def odom_cb(self, msg):

        if not self.mission_started:
            return

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        twist = msg.twist.twist

        curr_pos = np.array([pos.x, pos.y, pos.z])

        roll, pitch, yaw = euler_from_quaternion(
            [ori.x, ori.y, ori.z, ori.w])

        pitch_rate = twist.angular.y
        yaw_rate = twist.angular.z
        u_curr = twist.linear.x
        w_curr = twist.linear.z

        # Waypoint switching
        wp_start = self.waypoints[self.current_idx]
        wp_end = self.waypoints[self.current_idx + 1]

        dist = np.linalg.norm(curr_pos[:2] - wp_end)

        if dist < 10.0 and self.current_idx < len(self.waypoints) - 2:
            self.current_idx += 1

        # LOS
        target_xy, dist_to_goal, lproj = \
            self.compute_los_target_xy(wp_start, wp_end, curr_pos)

        desired_heading = math.atan2(
            target_xy[1] - curr_pos[1],
            target_xy[0] - curr_pos[0])

        heading_err = angle_wrap(desired_heading - yaw)

        vert_fin = -KP_YAW * heading_err - KD_YAW * yaw_rate
        vert_fin = clamp(vert_fin,
                         -MAX_VERT_FIN_RAD,
                         MAX_VERT_FIN_RAD)

        # Altitude + slope
        slope = self.estimate_slope()

        z_seabed_now = curr_pos[2] + self.h_now
        z_altitude = z_seabed_now - self.desired_altitude

        adaptive_lookahead = MIN_LOOKAHEAD + \
            SLOPE_LOOKAHEAD_GAIN * abs(slope)

        adaptive_lookahead = clamp(adaptive_lookahead,
                                   MIN_LOOKAHEAD,
                                   MAX_LOOKAHEAD)

        z_future_safe = z_altitude - slope * adaptive_lookahead

        z_d = min(z_altitude, z_future_safe)

        if self.h_now < self.emergency_altitude:
            print("⚠ Emergency climb triggered!")
            z_d = curr_pos[2] - 2.0

        if self.z_d_filtered is None:
            self.z_d_filtered = z_d

        z_d = ALTITUDE_FILTER_ALPHA * z_d + \
              (1 - ALTITUDE_FILTER_ALPHA) * self.z_d_filtered

        self.z_d_filtered = z_d

        # Depth outer loop
        e_z = z_d - curr_pos[2]
        theta_d = KP_DEPTH * e_z + KD_DEPTH * (-w_curr)
        theta_d = clamp(theta_d,
                        -MAX_PITCH_RAD,
                        MAX_PITCH_RAD)

        # Pitch inner loop
        pitch_err = theta_d - pitch

        horiz_fin = -KP_PITCH * pitch_err \
                    - KD_PITCH * pitch_rate

        horiz_fin = clamp(horiz_fin,
                          -MAX_HORIZ_FIN_RAD,
                          MAX_HORIZ_FIN_RAD)

        # Surge
        e_u = DEFAULT_CRUISE_SPEED - u_curr
        thrust = KP_SURGE * e_u
        thrust = clamp(thrust, -200.0, 200.0)

        # Publish
        self.publish(self.vert_pub, vert_fin)
        self.publish(self.horiz_pub, horiz_fin)

        # self.publish(self.horiz_port_pub, horiz_fin)
        # self.publish(self.horiz_star_pub, horiz_fin)
        self.publish(self.thrust_pub, thrust)

        # Debug publishing
        self.publish_debug(self.debug_alt_error, e_z)
        self.publish_debug(self.debug_zd, z_d)
        self.publish_debug(self.debug_slope, slope)
        self.publish_debug(self.debug_lookahead, adaptive_lookahead)
        self.publish_debug(self.debug_theta_d, theta_d)
        self.publish_debug(self.debug_pitch, pitch)
        self.publish_debug(self.debug_fin, horiz_fin)

        print(f"Slope: {slope:.3f} | "
              f"Alt Error: {e_z:.2f} m | "
              f"Lookahead: {adaptive_lookahead:.2f}")

    def publish(self, pub, value):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def publish_debug(self, pub, value):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = XYAltitudeMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()