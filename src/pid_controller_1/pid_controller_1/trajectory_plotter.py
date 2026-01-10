# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# import matplotlib.pyplot as plt
# import yaml
# import os
# from datetime import datetime

# class TrajectoryPlotter(Node):
#     def __init__(self):
#         super().__init__('trajectory_plotter')

#         # Subscribe to odometry
#         self.subscription = self.create_subscription(
#             Odometry, '/odom', self.odom_callback, 10
#         )

#         # Store trajectory
#         self.x_data = []
#         self.y_data = []

#         # Load waypoints from YAML
#         waypoints_file = os.path.join(
#             os.path.dirname(__file__),
#             "..", "waypoints.yaml"
#         )
#         with open(waypoints_file, 'r') as f:
#             data = yaml.safe_load(f)
#             self.waypoints = data['waypoints']

#     def odom_callback(self, msg):
#         self.x_data.append(msg.pose.pose.position.x)
#         self.y_data.append(msg.pose.pose.position.y)

#     def plot_and_save(self):
#         plt.figure(figsize=(8, 6))

#         # Plot actual trajectory
#         plt.plot(self.x_data, self.y_data, label="Actual Trajectory", color="blue")

#         # Plot waypoints
#         waypoints_x = [wp[0] for wp in self.waypoints]
#         waypoints_y = [wp[1] for wp in self.waypoints]
#         plt.scatter(waypoints_x, waypoints_y, color="red", marker="x", label="Waypoints")

#         plt.xlabel("X [m]")
#         plt.ylabel("Y [m]")
#         plt.title("AUV Trajectory vs Waypoints")
#         plt.legend()
#         plt.grid(True)

#         # 🔽 Added saving code 🔽
        

#         plt.show()

# def main(args=None):
#     rclpy.init(args=args)
#     node = TrajectoryPlotter()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down, generating trajectory plot...")
#         node.plot_and_save()
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()







 #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import yaml
import os
from datetime import datetime

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.x_data = []
        self.y_data = []

        # Load waypoints.yaml (assumes it’s in share/pid_controller/)
        package_share = os.path.join(
            os.path.expanduser('~'),
            'ros2_ws',
            'install',
            'pid_controller',
            'share',
            'pid_controller'
        )
        waypoints_path = os.path.join(package_share, 'waypoints.yaml')

        if os.path.exists(waypoints_path):
            with open(waypoints_path, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data['waypoints']
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        else:
            self.waypoints = []
            self.get_logger().warn("No waypoints.yaml found!")

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

    # Plot after shutdown
    plt.figure()
    # Actual trajectory
    plt.plot(node.x_data, node.y_data, 'b-', linewidth=2, label="Trajectory")
    # Waypoints (if available)
    if node.waypoints:
        for i, (wx, wy, wz) in enumerate(node.waypoints):
            plt.plot(wx, wy, 'ro')  # red circle
            plt.text(wx, wy, f"{i+1}", fontsize=9, ha='right')
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("AUV Trajectory vs Waypoints")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
  


    save_dir = "/home/thiru/ros2_ws/src/pid_controller/results_&_plots"
    os.makedirs(save_dir, exist_ok=True)

    filename = f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    save_path = os.path.join(save_dir, filename)

    plt.savefig(save_path, dpi=300)
    node.get_logger().info(f"Trajectory plot saved to: {save_path}")
        # 🔼 Added saving code 🔼

    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    