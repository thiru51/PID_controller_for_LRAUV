


# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Float64, Float32MultiArray, Float32
# import yaml
# import numpy as np
# import os
# import math
# from ament_index_python.packages import get_package_share_directory


# def euler_from_quaternion(quat):
#     # quat is [x, y, z, w]
#     x, y, z, w = quat
#     # roll (x-axis rotation)
#     t0 = +2.0 * (w * x + y * z)
#     t1 = +1.0 - 2.0 * (x * x + y * y)
#     roll = math.atan2(t0, t1)
#     # pitch (y-axis rotation)
#     t2 = +2.0 * (w * y - z * x)
#     if t2 > +1.0:
#         t2 = +1.0
#     if t2 < -1.0:
#         t2 = -1.0
#     pitch = math.asin(t2)
#     # yaw (z-axis rotation)
#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     yaw = math.atan2(t3, t4)
#     return roll, pitch, yaw

# WAYPOINTS_FILE = "/home/thiru/ros2_ws/src/pid_controller_1/pid_controller_1/waypoints.yaml"
# # LOOKAHEAD_DIST = 5  # meters
# KP =  30  # Proportional gain
# KD = 0.5  # Derivative gain
# FORWARD_THRUST = -30.0  # Example value (Newtons or percent, tune for your sim)

# class LOSPathFollower(Node):
#     def __init__(self):
#         super().__init__('los_path_follower')
#         self.subscription = self.create_subscription(
#             Odometry, '/odom', self.odom_callback, 10)
#         self.thrust_pub = self.create_publisher(Float64, '/cmd/thrust', 10)
#         self.fins_pub = self.create_publisher(Float64, '/cmd/vertical_fins', 10)
#         self.get_waypoints()
#         self.prev_heading_error = 0.0
#         self.prev_time = None

#     def get_waypoints(self):
#         with open(WAYPOINTS_FILE, 'r') as f:
#             data = yaml.safe_load(f)
#         waypoints = data['waypoints']
#         # Take only XY, ignore Z: use first 2 waypoints
#         self.xk, self.yk = waypoints[0][:2]
#         self.xkp1, self.ykp1 = waypoints[1][:2]
#         # Path slope (alpha):
#         dx = self.xkp1 - self.xk
#         dy = self.ykp1 - self.yk
#         self.alpha = math.atan2(dy, dx)

#     def odom_callback(self, msg):
#         pos = msg.pose.pose.position
#         ori = msg.pose.pose.orientation
#         # Quaternion to euler (yaw is heading)
#         q = [ori.x, ori.y, ori.z, ori.w]
#         _, _, yaw = euler_from_quaternion(q)
#         # Cross-track error (e)
#         x, y = pos.x, pos.y
#         yaw_rate = msg.twist.twist.angular.z  # radians/sec
#         e = - (x - self.xk) * math.sin(self.alpha) + (y - self.yk) * math.cos(self.alpha)
#         # LOOKAHEAD_DIST = max(min(5.0, e), 3.0)
#         LOOKAHEAD_DIST = 8.0                                                                                                                                                                                                                                                                                                                                                                                   
#         desired_heading = self.alpha + math.atan2(-e, LOOKAHEAD_DIST)
#         # Heading error (wrap to [-pi, pi])
#         # heading_error = (desired_heading - yaw + math.pi) % (2 * math.pi) - math.pi
#         heading_error = desired_heading - yaw
#         while heading_error > math.pi:
#             heading_error -= 2 * math.pi
#         while heading_error < -math.pi:
#             heading_error += 2 * math.pi



#         # PD control
#         now = self.get_clock().now().nanoseconds / 1e9
#         dt = now - self.prev_time if self.prev_time is not None else 0.1
#         d_error = (heading_error - self.prev_heading_error) / dt if dt > 0 else 0.0
#         fin_cmd = -KP * heading_error - KD * yaw_rate

#         #clamp fin_cmd to reasonable limits, e.g., -0.4 to 0.4 radians
#         MAX_FIN_RAD = 1.45
#         fin_cmd = max(min(fin_cmd, MAX_FIN_RAD), -MAX_FIN_RAD)


#         # --- Publish commands ---
#         # 1. Forward thrust (constant for now)
#         thrust_msg = Float64()
#         thrust_msg.data = FORWARD_THRUST
#         self.thrust_pub.publish(thrust_msg)


#         # 2. Vertical fin command (for yaw control)
#         # If your sim expects angle in degrees/radians or normalized [-1,1], scale appropriately
#         fin_msg = Float64()
#         fin_msg.data = float(fin_cmd)
#         self.fins_pub.publish(fin_msg)


#         # Debug
#         self.get_logger().info(f"e={e:.2f}, yaw={yaw:.2f}, des_head={desired_heading:.2f}, fin={fin_cmd:.2f}")
#         self.prev_heading_error = heading_error
#         self.prev_time = now


# def main(args=None):
#     rclpy.init(args=args)
#     follower = LOSPathFollower()
#     try:
#         rclpy.spin(follower)
#     except KeyboardInterrupt:
#         pass
#     follower.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



















# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Float64
# import yaml
# import numpy as np
# import math

# # ==========================================
# # CONFIGURATION
# # ==========================================
# WAYPOINTS_FILE = "/home/thiru/ros2_ws/src/pid_controller_1/pid_controller_1/waypoints.yaml"

# # LOS Parameters
# MIN_LOOKAHEAD = 5
# MAX_LOOKAHEAD = 10

# # PID Gains (Tune these)
# KP = 2.0    # Lowered slightly; 30 is usually very high for radians. Check your magnitude!
# KD = 0.5    # Derivative (Damping)
# FORWARD_THRUST = -50.0 
# MAX_FIN_RAD = 0.785  # 45 degrees

# class LOSPathFollower(Node):
#     def __init__(self):
#         super().__init__('los_path_follower')
        
#         # Subscribers & Publishers
#         self.subscription = self.create_subscription(
#             Odometry, '/odom', self.odom_callback, 10)
#         self.thrust_pub = self.create_publisher(Float64, '/cmd/thrust', 10)
#         self.fins_pub = self.create_publisher(Float64, '/cmd/vertical_fins', 10)
        
#         # State Variables
#         self.prev_time = None
        
#         # Load Waypoints
#         self.get_waypoints()

#     def get_waypoints(self):
#         try:
#             with open(WAYPOINTS_FILE, 'r') as f:
#                 data = yaml.safe_load(f)
#             waypoints = data['waypoints']
            
#             # Define Start (x1, y1) and End (x2, y2) of the line
#             self.wp_start = np.array(waypoints[0][:2]) # x1, y1
#             self.wp_end   = np.array(waypoints[1][:2]) # x2, y2
            
#             self.get_logger().info(f"Path loaded: {self.wp_start} -> {self.wp_end}")
            
#         except Exception as e:
#             self.get_logger().error(f"Failed to load waypoints: {str(e)}")
#             # Fallback to dummy points to prevent crash
#             self.wp_start = np.array([0.0, 0.0])
#             self.wp_end = np.array([10.0, 10.0])

#     def euler_from_quaternion(self, quat):
#         x, y, z, w = quat
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + y * y)
#         roll = math.atan2(t0, t1)
        
#         t2 = +2.0 * (w * y - z * x)
#         t2 = +1.0 if t2 > +1.0 else t2
#         t2 = -1.0 if t2 < -1.0 else t2
#         pitch = math.asin(t2)
        
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         yaw = math.atan2(t3, t4)
#         return roll, pitch, yaw

#     def odom_callback(self, msg):
#         # 1. Extract Robot State
#         pos = msg.pose.pose.position
#         ori = msg.pose.pose.orientation
#         twist = msg.twist.twist
        
#         curr_pos = np.array([pos.x, pos.y])
        
#         q = [ori.x, ori.y, ori.z, ori.w]
#         _, _, yaw_curr = self.euler_from_quaternion(q)
#         yaw_rate = twist.angular.z

#         # ==========================================================
#         # NEW LOS LOGIC START
#         # ==========================================================
        
#         x1, y1 = self.wp_start
#         x2, y2 = self.wp_end
#         x3, y3 = curr_pos

#         # A. Calculate Squared Length of Line
#         line_len_squared = (x2 - x1)**2 + (y2 - y1)**2

#         # B. Calculate Projection Point (x4, y4)
#         if line_len_squared < 0.001:
#             x4, y4 = x1, y1
#         else:
#             # Dot product projection to find ratio 'l'
#             l = ((x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1)) / line_len_squared
#             x4, y4 = l * (x2 - x1) + x1, l * (y2 - y1) + y1

#         # C. Distance from Projection(x4) to Goal(x2)
#         d2 = math.sqrt((x4 - x2)**2 + (y4 - y2)**2)

#         # D. Adaptive Lookahead Calculation
#         # Distance from robot to goal
#         dist_to_goal = math.sqrt((x3 - x2)**2 + (y3 - y2)**2)
#         delta = max(min(MAX_LOOKAHEAD, dist_to_goal), MIN_LOOKAHEAD)

#         # E. Calculate Target Point (x5, y5)
#         if d2 < 0.001:
#             x5, y5 = x2, y2
#         else:
#             # Section formula to place point 'delta' meters down the line from x4
#             x5 = ((d2 - delta) * x4 + delta * x2) / d2
#             y5 = ((d2 - delta) * y4 + delta * y2) / d2

#         # ==========================================================
#         # CONTROL LOGIC
#         # ==========================================================

#         # 1. Calculate Desired Heading (Bearing to x5, y5)
#         desired_heading = math.atan2(y5 - y3, x5 - x3)

#         # 2. Calculate Heading Error with SSA (Smallest Signed Angle)
#         raw_error = desired_heading - yaw_curr
#         # This ensures error is always between -pi and +pi
#         heading_error = math.atan2(math.sin(raw_error), math.cos(raw_error))

#         # 3. PD Control
#         # Note: We use -KD * yaw_rate. This is "Derivative on Measurement".
#         # It provides damping against the robot's spinning motion.
#         fin_cmd = -KP * heading_error - KD * yaw_rate

#         # 4. Saturation (Clamping)
#         fin_cmd = max(min(fin_cmd, MAX_FIN_RAD), -MAX_FIN_RAD)

#         # ==========================================================
#         # PUBLISH
#         # ==========================================================

#         # Thrust
#         thrust_msg = Float64()
#         thrust_msg.data = FORWARD_THRUST
#         self.thrust_pub.publish(thrust_msg)

#         # Fins
#         fin_msg = Float64()
#         fin_msg.data = float(fin_cmd)
#         self.fins_pub.publish(fin_msg)

#         # Debugging
#         # Prints: Error, Target Point, Desired Yaw vs Current Yaw
#         self.get_logger().info(
#             f"Err: {heading_error:.2f} | Tgt: ({x5:.1f}, {y5:.1f}) | Head: {desired_heading:.2f} vs {yaw_curr:.2f} | Cmd: {fin_cmd:.2f}"
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     follower = LOSPathFollower()
#     try:
#         rclpy.spin(follower)
#     except KeyboardInterrupt:
#         pass
#     follower.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()























import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import yaml
import numpy as np
import math

# ==========================================
# CONFIGURATION
# ==========================================
WAYPOINTS_FILE = "/home/thiru/ros2_ws/src/pid_controller_1/pid_controller_1/waypoints.yaml"

# LOS Parameters
MIN_LOOKAHEAD = 5
MAX_LOOKAHEAD = 20 # Slightly increased for smoother transitions between segments

# Switching Logic
ACCEPTANCE_RADIUS = 10.0  # Radius around waypoint to consider it "reached"

# PID Gains
KP = 15 
KD = 0.5    
FORWARD_THRUST = -50.0 
MAX_FIN_RAD = 0.9785  # 45 degrees

class LOSPathFollower(Node):
    def __init__(self):
        super().__init__('los_path_follower')
        
        # Subscribers & Publishers
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.thrust_pub = self.create_publisher(Float64, '/cmd/thrust', 10)
        self.fins_pub = self.create_publisher(Float64, '/cmd/vertical_fins', 10)
        
        # Waypoint State Management
        self.waypoints = []       # List of all waypoints
        self.current_idx = 0      # Index of the current START waypoint
        self.mission_complete = False
        
        # Load Waypoints
        self.load_waypoints()

    def load_waypoints(self):
        """ Loads all waypoints from YAML into a numpy array """
        try:
            with open(WAYPOINTS_FILE, 'r') as f:
                data = yaml.safe_load(f)
            
            # Convert list of lists to numpy array, taking only X and Y
            # data['waypoints'] looks like [[50,50,0], [90,50,0], [30,20,0]]
            raw_points = data['waypoints']
            self.waypoints = np.array([p[:2] for p in raw_points])
            
            if len(self.waypoints) < 2:
                self.get_logger().error("Need at least 2 waypoints to form a path!")
                self.mission_complete = True
            else:
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints successfully.")
                
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {str(e)}")
            self.mission_complete = True

    def euler_from_quaternion(self, quat):
        x, y, z, w = quat
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def odom_callback(self, msg):
        if self.mission_complete or len(self.waypoints) < 2:
            # Stop the craft if mission is done or no waypoints
            self.stop_vehicle()
            return

        # 1. Extract Robot State
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        twist = msg.twist.twist
        
        curr_pos = np.array([pos.x, pos.y])
        q = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw_curr = self.euler_from_quaternion(q)
        yaw_rate = twist.angular.z

        # ==========================================================
        # WAYPOINT SWITCHING LOGIC
        # ==========================================================
        
        # Define current line segment: Start (idx) -> End (idx+1)
        wp_start = self.waypoints[self.current_idx]
        wp_end   = self.waypoints[self.current_idx + 1]
        
        # Calculate distance to the current GOAL (wp_end)
        dist_to_current_goal = np.linalg.norm(curr_pos - wp_end)
        
        # Check if we reached the waypoint
        if dist_to_current_goal < ACCEPTANCE_RADIUS:
            # Check if there are more segments after this
            # We need at least 2 points remaining to form a new line
            if self.current_idx < len(self.waypoints) - 2:
                self.current_idx += 1
                self.get_logger().info(f">>> Waypoint Reached! Switching to Segment {self.current_idx}: {self.waypoints[self.current_idx]} -> {self.waypoints[self.current_idx+1]}")
                # Update pointers immediately for the rest of this loop
                wp_start = self.waypoints[self.current_idx]
                wp_end   = self.waypoints[self.current_idx + 1]
            else:
                self.get_logger().info(">>> FINAL Waypoint Reached. Maintaining position.")
                # Optional: You could set self.mission_complete = True here to stop
                # For now, we let it track the final point indefinitely

        # ==========================================================
        # LOS LOGIC (Using wp_start and wp_end)
        # ==========================================================
        
        x1, y1 = wp_start
        x2, y2 = wp_end
        x3, y3 = curr_pos

        # A. Squared Length of Line Segment
        line_len_squared = (x2 - x1)**2 + (y2 - y1)**2

        # B. Projection Point (x4, y4)
        if line_len_squared < 0.001:
            x4, y4 = x1, y1
        else:
            # Vector Projection ratio 'l'
            l = ((x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1)) / line_len_squared
            x4, y4 = l * (x2 - x1) + x1, l * (y2 - y1) + y1

        # C. Distance from Projection to Goal
        d2 = math.sqrt((x4 - x2)**2 + (y4 - y2)**2)

        # D. Adaptive Lookahead
        # Note: We use dist_to_current_goal calculated earlier
        delta = max(min(MAX_LOOKAHEAD, dist_to_current_goal), MIN_LOOKAHEAD)
        # delta = 8.0  # Fixed lookahead for smoother control

        # E. Calculate Target Point (x5, y5)
        if d2 < 0.001:
            x5, y5 = x2, y2
        else:
            x5 = ((d2 - delta) * x4 + delta * x2) / d2
            y5 = ((d2 - delta) * y4 + delta * y2) / d2

        # ==========================================================
        # PID CONTROL
        # ==========================================================

        # 1. Desired Heading
        desired_heading = math.atan2(y5 - y3, x5 - x3)

        # 2. Heading Error (SSA)
        raw_error = desired_heading - yaw_curr
        heading_error = math.atan2(math.sin(raw_error), math.cos(raw_error))

        # 3. Compute Command
        fin_cmd = -KP * heading_error - KD * yaw_rate
        fin_cmd = max(min(fin_cmd, MAX_FIN_RAD), -MAX_FIN_RAD)

        # ==========================================================
        # PUBLISH
        # ==========================================================
        
        # Publish Thrust
        thrust_msg = Float64()
        thrust_msg.data = FORWARD_THRUST
        self.thrust_pub.publish(thrust_msg)

        # Publish Fins
        fin_msg = Float64()
        fin_msg.data = float(fin_cmd)
        self.fins_pub.publish(fin_msg)

        # Debug info to show which waypoint we are tracking
        self.get_logger().info(
            f"Seg: {self.current_idx} | DistGoal: {dist_to_current_goal:.1f}m | Err: {heading_error:.2f} | Cmd: {fin_cmd:.2f}"
        )

    def stop_vehicle(self):
        # Safety function to stop the vehicle
        stop_msg = Float64()
        stop_msg.data = 0.0
        self.thrust_pub.publish(stop_msg)
        self.fins_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    follower = LOSPathFollower()
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()