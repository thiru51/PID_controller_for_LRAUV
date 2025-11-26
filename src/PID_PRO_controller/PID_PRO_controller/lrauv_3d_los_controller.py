#!/usr/bin/env python3
"""
ros2 node: lrauv_3d_los_controller.py

- 3D LOS path follower with cascaded depth->pitch PD, yaw PD, and surge P.
- Publishes:
    /cmd/vertical_fins   (Float64)  -- yaw control (rudder-like)
    /cmd/horizontal_fins (Float64)  -- pitch control (elevator-like)
    /cmd/thrust          (Float64)  -- surge thrust command
- Subscribes:
    /odom (nav_msgs/Odometry)
- Loads waypoints YAML (format described by user).
- Assumptions:
    * z increases downward (positive down).
    * Right-hand coord: +x forward, +y right, +z down.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import yaml
import numpy as np
import math
import os
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

# -------------------------
# DEFAULT CONFIG / GAINS
# -------------------------
DEFAULT_WAYPOINTS_FILE = "/home/thiru_na22b078/ros2_ws/src/PID_PRO_controller/PID_PRO_controller/waypoints.yaml"

# LOS lookahead limits (meters)
MIN_LOOKAHEAD = 6.2
MAX_LOOKAHEAD = 20.0

# Waypoint acceptance radius (m)
ACCEPTANCE_RADIUS = 15.0

# Yaw (heading) PD (vertical fins)
KP_YAW = 9.0
KD_YAW = 0.5          # tuned reasonably larger than original very-small kd
MAX_VERT_FIN_RAD = 0.52  # rad (~30 deg)

# Depth outer PD -> desired pitch (theta_d)
KP_DEPTH = 0.12      # (rad / m) -- map depth error to pitch angle
KD_DEPTH = 0.06      # (rad / (m/s)) using w (vel z)

# Pitch inner PD (horizontal fins)
KP_PITCH = 6.0       # fin_deflection / rad
KD_PITCH = 0.8       # fin_deflection / (rad/s)
MAX_HORIZ_FIN_RAD = 0.52

# Surge (simple P) -- optional: can use constant thrust instead
KP_SURGE = 40.0      # (thrust per m/s) - tune carefully
DEFAULT_CRUISE_SPEED = -1.0  # m/s (forward)

# Pitch angle limit for safety
MAX_PITCH_RAD = 0.45  # ~25 degrees

# Fin command rate limit (rad/s) optional
MAX_FIN_RATE = 1.0  # rad/s (not enforced here, left as simple param)

# -------------------------
# Helper utilities
# -------------------------
def clamp(x, lo, hi):
    if x != x:  # NaN check
        return lo
    return max(lo, min(hi, x))

def euler_from_quaternion(quat):
    x, y, z, w = quat
    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def angle_wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

# -------------------------
# Controller Node
# -------------------------
class LOS3DController(Node):
    def __init__(self):
        super().__init__('los_3d_controller')

        qos = QoSProfile(depth=10)
        # publishers
        self.vert_fin_pub = self.create_publisher(Float64, '/cmd/vertical_fins', qos)
        self.horiz_fin_pub = self.create_publisher(Float64, '/cmd/horizontal_fins', qos)
        self.thrust_pub = self.create_publisher(Float64, '/cmd/thrust', qos)

        # subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 20)

        # parameters (allow overrides from launch file)
        self.declare_parameter('waypoints_file', DEFAULT_WAYPOINTS_FILE)
        self.declare_parameter('min_lookahead', MIN_LOOKAHEAD)
        self.declare_parameter('max_lookahead', MAX_LOOKAHEAD)
        self.declare_parameter('acceptance_radius', ACCEPTANCE_RADIUS)

        # Gains as parameters for easy tuning later
        self.declare_parameter('kp_yaw', KP_YAW)
        self.declare_parameter('kd_yaw', KD_YAW)
        self.declare_parameter('kp_depth', KP_DEPTH)
        self.declare_parameter('kd_depth', KD_DEPTH)
        self.declare_parameter('kp_pitch', KP_PITCH)
        self.declare_parameter('kd_pitch', KD_PITCH)
        self.declare_parameter('kp_surge', KP_SURGE)

        # read params
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.min_lookahead = float(self.get_parameter('min_lookahead').value)
        self.max_lookahead = float(self.get_parameter('max_lookahead').value)
        self.acceptance_radius = float(self.get_parameter('acceptance_radius').value)

        # Gains
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.kp_depth = float(self.get_parameter('kp_depth').value)
        self.kd_depth = float(self.get_parameter('kd_depth').value)
        self.kp_pitch = float(self.get_parameter('kp_pitch').value)
        self.kd_pitch = float(self.get_parameter('kd_pitch').value)
        self.kp_surge = float(self.get_parameter('kp_surge').value)

        # state
        self.waypoints = None   # Nx3 array [[x,y,z],...]
        self.current_idx = 0
        self.mission_complete = False

        # last commanded setpoints (for rate-limiting if needed)
        self.last_theta_d = 0.0

        # cruise speed (can be used or left as fixed thrust)
        self.cruise_speed = DEFAULT_CRUISE_SPEED

        # load waypoints
        self.load_waypoints(self.waypoints_file)

        self.get_logger().info('LOS3DController initialized.')

    # -------------------------
    # Waypoints
    # -------------------------
    def load_waypoints(self, fp):
        if not fp or not os.path.exists(fp):
            self.get_logger().error(f"Waypoints file not found: {fp}")
            self.waypoints = None
            self.mission_complete = True
            return
        try:
            with open(fp, 'r') as f:
                data = yaml.safe_load(f)
            raw_points = data.get('waypoints', None)
            if raw_points is None:
                self.get_logger().error("No 'waypoints' key in YAML!")
                self.waypoints = None
                self.mission_complete = True
                return
            arr = np.array(raw_points, dtype=float)
            if arr.ndim != 2 or arr.shape[1] < 3:
                self.get_logger().error("Waypoints must be list of [x,y,z] entries.")
                self.waypoints = None
                self.mission_complete = True
                return
            self.waypoints = arr[:, :3]
            if self.waypoints.shape[0] < 2:
                self.get_logger().error("Need at least 2 waypoints.")
                self.mission_complete = True
                return
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {fp}.")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            self.waypoints = None
            self.mission_complete = True

    # -------------------------
    # LOS 3D helper: project and compute lookahead target
    # returns (x5,y5,z5), dist_to_goal, projection_l
    # -------------------------
    def compute_los_target_3d(self, wp_start, wp_end, curr_pos):
        # wp_start, wp_end, curr_pos are numpy arrays [x,y,z]
        x1, y1, z1 = wp_start
        x2, y2, z2 = wp_end
        x3, y3, z3 = curr_pos

        # line vector and squared length
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        line_len_squared = dx*dx + dy*dy + dz*dz

        if line_len_squared < 1e-6:
            # degenerate segment -> target is endpoint
            proj = np.array([x1, y1, z1])
            l = 0.0
        else:
            # projection parameter l (can be <0 or >1 if beyond segment)
            l = ((dx)*(x3 - x1) + (dy)*(y3 - y1) + (dz)*(z3 - z1)) / line_len_squared
            proj = np.array([x1 + l*dx, y1 + l*dy, z1 + l*dz])

        # distance from projection to goal (3D)
        d2 = float(np.linalg.norm(proj - np.array([x2, y2, z2])))

        # adaptive lookahead based on distance to current goal
        dist_to_goal = float(np.linalg.norm(curr_pos - np.array([x2, y2, z2])))
        delta = max(min(self.max_lookahead, dist_to_goal), self.min_lookahead)

        if d2 < 1e-6:
            target = np.array([x2, y2, z2])
        else:
            # same interpolation formula as in your XY code, extended to 3D
            target = ((d2 - delta) * proj + delta * np.array([x2, y2, z2])) / d2

        return target, dist_to_goal, l

    # -------------------------
    # Controller computations
    # -------------------------
    def compute_heading_command(self, desired_heading, yaw_curr, yaw_rate):
        raw_err = angle_wrap(desired_heading - yaw_curr)
        fin_cmd = -self.kp_yaw * raw_err - self.kd_yaw * yaw_rate
        fin_cmd = clamp(fin_cmd, -MAX_VERT_FIN_RAD, MAX_VERT_FIN_RAD)
        return fin_cmd, raw_err

    def depth_outer_to_pitch(self, z_d, z_curr, w):
        # e_z = z_d - z_curr (z positive down)
        e_z = z_d - z_curr
        # derivative approx: -w term (since e_z_dot ~= -w when z_d constant)
        theta_d = self.kp_depth * e_z + self.kd_depth * (-w)
        # clamp desired pitch
        theta_d = clamp(theta_d, -MAX_PITCH_RAD, MAX_PITCH_RAD)
        return theta_d, e_z

    def compute_pitch_command(self, theta_d, theta_curr, pitch_rate):
        # inner PD (angle tracking)
        err = theta_d - theta_curr
        fin_h = -self.kp_pitch * err - self.kd_pitch * pitch_rate
        fin_h = clamp(fin_h, -MAX_HORIZ_FIN_RAD, MAX_HORIZ_FIN_RAD)
        return fin_h, err

    def compute_surge_thrust(self, u_d, u_curr):
        # simple P controller for surge speed. If you want constant thrust, set u_d=None
        if u_d is None:
            return 0.0
        e_u = u_d - u_curr
        thrust = self.kp_surge * e_u
        # clamp thrust to reasonable bounds (user may customize)
        thrust = clamp(thrust, -200.0, 200.0)  # adjust to actuator capability
        return thrust

    # -------------------------
    # ODOM CALLBACK: main loop
    # -------------------------
    def odom_cb(self, msg: Odometry):
        # safety
        if self.mission_complete or self.waypoints is None:
            self.stop_vehicle()
            return

        # extract pose / twist
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        twist = msg.twist.twist

        # current state
        curr_pos = np.array([pos.x, pos.y, pos.z], dtype=float)
        roll, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        # angular rates: twist.angular.x = roll rate, .y = pitch rate, .z = yaw rate
        pitch_rate = float(twist.angular.y)
        yaw_rate = float(twist.angular.z)
        # linear velocities: twist.linear.x = forward speed
        u_curr = float(twist.linear.x)
        w_curr = float(twist.linear.z)  # vertical speed (positive downward by assumption)

        # Current segment
        idx = self.current_idx
        wp_start = self.waypoints[idx]
        wp_end = self.waypoints[idx + 1]

        # distance to current goal (endpoint)
        dist_to_goal = float(np.linalg.norm(curr_pos - wp_end))

        # waypoint switching
        if dist_to_goal < self.acceptance_radius:
            if idx < len(self.waypoints) - 2:
                self.current_idx += 1
                idx = self.current_idx
                wp_start = self.waypoints[idx]
                wp_end = self.waypoints[idx + 1]
                self.get_logger().info(f"Waypoint reached -> switching to segment {idx}: {wp_start} -> {wp_end}")
            else:
                # final waypoint reached; keep tracking final point (or change behavior)
                self.get_logger().info("FINAL waypoint reached. Holding at last segment/point.")
                # Optionally set mission_complete = True
                # self.mission_complete = True

        # LOS 3D target
        target_3d, dist_to_goal, lproj = self.compute_los_target_3d(wp_start, wp_end, curr_pos)
        x5, y5, z5 = target_3d

        # Desired heading from LOS target in XY plane
        desired_heading = math.atan2(y5 - curr_pos[1], x5 - curr_pos[0])
        # compute yaw fin command
        vert_fin_cmd, heading_err = self.compute_heading_command(desired_heading, yaw, yaw_rate)

        # Depth outer -> desired pitch (cascaded approach)
        # z positive down -> z_d given by z5
        z_d = z5
        theta_d, depth_err = self.depth_outer_to_pitch(z_d, curr_pos[2], w_curr)

        # pitch inner -> horizontal fin command
        horiz_fin_cmd, pitch_err = self.compute_pitch_command(theta_d, pitch, pitch_rate)

        # Surge (simple): try to keep cruise speed or publish constant thrust
        # Here we compute thrust from cruise speed setpoint. If you prefer constant thrust,
        # set u_d = None and then use FORWARD_THRUST constant below.
        u_d = self.cruise_speed
        thrust_cmd = self.compute_surge_thrust(u_d, u_curr)

        # If you prefer fixed thrust regardless of speed, uncomment:
        # thrust_cmd = -50.0

        # Publish commands
        self.publish_command(self.vert_fin_pub, vert_fin_cmd)
        self.publish_command(self.horiz_fin_pub, horiz_fin_cmd)
        self.publish_command(self.thrust_pub, thrust_cmd)

        # Debug / logging (controlled verbosity)
        self.get_logger().debug(
            f"Seg:{self.current_idx} | DistGoal:{dist_to_goal:.2f} | "
            f"ψ_d:{desired_heading:.2f} ψ:{yaw:.2f} err:{heading_err:.2f} fin_v:{vert_fin_cmd:.3f} | "
            f"z_d:{z_d:.2f} z:{curr_pos[2]:.2f} depth_err:{depth_err:.2f} θ_d:{theta_d:.3f} θ:{pitch:.3f} fin_h:{horiz_fin_cmd:.3f} | "
            f"u:{u_curr:.2f} T:{thrust_cmd:.1f}"
        )

    # -------------------------
    # Publishing helper
    # -------------------------
    def publish_command(self, pub, value: float):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    # -------------------------
    # Safety stop
    # -------------------------
    def stop_vehicle(self):
        try:
            msg = Float64()
            msg.data = 0.0
            self.thrust_pub.publish(msg)
            self.vert_fin_pub.publish(msg)
            self.horiz_fin_pub.publish(msg)
        except Exception:
            pass

# -------------------------
# Main
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LOS3DController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
