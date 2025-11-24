#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib
# Use a non-interactive backend so saving works on headless systems
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.listener_callback, 10
        )

        # Store trajectory
        self.x_data = []
        self.y_data = []

        # Load waypoints
        self.load_waypoints()
    
    def load_waypoints(self):
        # Your waypoint reading logic
        pkg_path = get_package_share_directory('pid_controller')
        # installed packages place runtime config under share/<pkg>/config
        waypoints_path = os.path.join(pkg_path, 'config', 'waypoints.yaml')
        
        if not os.path.exists(waypoints_path):
             # Fallback
             pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
             waypoints_path = os.path.join(pkg_path, 'waypoints.yaml')
        
        if os.path.exists(waypoints_path):
            self.get_logger().info(f'Plotter loading waypoints from: {waypoints_path}')
            with open(waypoints_path, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data['waypoints']
                self.get_logger().info(f"Plotter loaded {len(self.waypoints)} waypoints")
        else:
            self.waypoints = []
            self.get_logger().warning("Plotter could not find waypoints.yaml!")

    def listener_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_data.append(x)
        self.y_data.append(y)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()

    try:
        rclpy.spin(node)  # collect until Ctrl+C
    except KeyboardInterrupt:
        pass
    
    node.get_logger().info("Shutting down, generating plot...")

    # Plot after shutdown
    plt.figure(figsize=(10, 8))
    # Actual trajectory
    plt.plot(node.x_data, node.y_data, 'b-', linewidth=2, label="Trajectory")
    
    # --- MODIFIED PLOTTING LOGIC ---
    if node.waypoints:
        # Plot Start Point (index 0)
        wx_start, wy_start, _ = node.waypoints[0]
        plt.plot(wx_start, wy_start, 'go', markersize=10, label="Start") # Green circle
        plt.text(wx_start + 0.5, wy_start, "Start (WP 0)", fontsize=12, ha='left')
        
        # Plot other waypoints (1 to n-1)
        # This will plot WP 1, WP 2, WP 3, WP 4
        for i, (wx, wy, wz) in enumerate(node.waypoints[1:]): # Loop from index 1 onwards
            plt.plot(wx, wy, 'ro', markersize=8)  # red circle
            # Label as WP 1, WP 2, etc. (i starts at 0, so i+1 is correct)
            plt.text(wx + 0.5, wy, f"WP {i+1}", fontsize=12, ha='left')
    # --- END MODIFIED LOGIC ---
            
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("AUV Trajectory vs Waypoints")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
  
    # Your save logic
    try:
        # Prefer saving inside the package share folder results_and_plots
        try:
            pkg_share = get_package_share_directory('pid_controller')
            pkg_save_dir = os.path.join(pkg_share, 'results_and_plots')
        except Exception:
            pkg_save_dir = None

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename_pkg = None
        if pkg_save_dir:
            try:
                os.makedirs(pkg_save_dir, exist_ok=True)
                filename_pkg = os.path.join(pkg_save_dir, f"trajectory_{timestamp}.png")
                plt.savefig(filename_pkg)
                node.get_logger().info(f"Plot saved to package share: {filename_pkg}")
            except Exception as e:
                node.get_logger().warning(f"Failed saving to package share: {e}")

        # Always also save to a user-writable fallback directory for visibility
        fallback_dir = os.path.expanduser("~/ros2_plots")
        os.makedirs(fallback_dir, exist_ok=True)
        filename_fb = os.path.join(fallback_dir, f"trajectory_{timestamp}.png")
        plt.savefig(filename_fb)
        node.get_logger().info(f"Plot saved to fallback directory: {filename_fb}")
    except Exception as e:
        node.get_logger().error(f"Failed to save plot: {e}")
        
    # plt.show() # Uncomment this if you want to see the plot immediately

    node.destroy_node()
    rclpy.shutdown()
