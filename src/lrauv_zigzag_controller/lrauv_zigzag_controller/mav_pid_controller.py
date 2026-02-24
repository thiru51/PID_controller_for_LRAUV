import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import yaml
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

from lrauv_zigzag_controller.module_kinematics import quat_to_eul
from lrauv_zigzag_controller import module_control as con

class MAVPIDController(Node):
    def __init__(self):
        super().__init__('mav_pid_controller')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.rudder_pub = self.create_publisher(
            Float64,
            '/cmd/horizontal_fins',
            10)
        
        self.thrust_pub = self.create_publisher(Float64, '/cmd/thrust', 10)

        # Read waypoints using ROS 2 package path
        self.waypoints, self.waypoints_type = self.read_waypoints()
        self.waypoint_idx = 1

        self.start_time = self.get_clock().now()
        self.ye_int = 0.0
        self.te_int = 0.0

        self.control_mode = con.pid_control

        self.get_logger().info('MAV PID Controller initialized')

    def read_waypoints(self):
        pkg_path = get_package_share_directory('lrauv_zigzag_controller')
        waypoints_path = os.path.join(pkg_path, 'lrauv_zigzag_controller', 'waypoints.yml')
        with open(waypoints_path, 'r') as file:
            wps_file = yaml.safe_load(file)
            waypoints = np.array(wps_file['waypoints'])
            waypoints_type = wps_file['waypoints_type']
        return waypoints, waypoints_type

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds / 1e9

        quat = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

        eul = quat_to_eul(quat, order='ZYX')

        state = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            eul[0],
            eul[1],
            eul[2],
            0.0
        ])

        rudder_cmd, ye, self.waypoint_idx = self.control_mode(t, state, self.waypoints, self.waypoint_idx, self.ye_int)

        dt = t - self.te_int
        if dt > 0:
            self.ye_int += ye * dt
            self.te_int = t

        thrust = -5.0

        cmd_msg = Float64()
        cmd_msg.data = float(rudder_cmd)
        thrust_msg = Float64()
        thrust_msg.data = thrust

        self.rudder_pub.publish(cmd_msg)
        self.thrust_pub.publish(thrust_msg)

        self.get_logger().info(
            f'\nTime: {t:.2f}s\n'
            f'Rudder Command: {rudder_cmd*180/np.pi:.2f} deg\n'
        )

def main(args=None):
    rclpy.init(args=args)
    controller = MAVPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()