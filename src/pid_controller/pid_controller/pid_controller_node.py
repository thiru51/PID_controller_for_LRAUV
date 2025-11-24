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
WAYPOINTS_FILE = "/home/thiru/ros2_ws/src/pid_controller/pid_controller/waypoints.yaml"

# LOS Parameters
MIN_LOOKAHEAD = 3.0
MAX_LOOKAHEAD = 6.0  # Slightly increased for smoother transitions between segments

# Switching Logic
ACCEPTANCE_RADIUS = 5.0  # Radius around waypoint to consider it "reached"

# PID Gains
KP = 2.0   
KD = 0.5    
FORWARD_THRUST = -20.0 
MAX_FIN_RAD = 0.785  # 45 degrees

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
        
        # Declare and read parameter for waypoints file (can be overridden by launch)
        self.declare_parameter('waypoints_file', WAYPOINTS_FILE)
        self.waypoints_file = self.get_parameter('waypoints_file').value

        # Load Waypoints
        self.load_waypoints()

    def load_waypoints(self):
        """ Loads all waypoints from YAML into a numpy array """
        try:
            # Prefer parameter-specified file, fallback to module constant
            fp = getattr(self, 'waypoints_file', WAYPOINTS_FILE) or WAYPOINTS_FILE
            with open(fp, 'r') as f:
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