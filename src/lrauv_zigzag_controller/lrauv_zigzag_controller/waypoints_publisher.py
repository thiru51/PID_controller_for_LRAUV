#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from ament_index_python.packages import get_package_share_directory

class WaypointsPublisher(Node):
    def __init__(self):
        super().__init__('waypoints_publisher')
        self.publisher = self.create_publisher(PoseArray, '/mav_waypoints', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)  # Publish every second
        
        # Try to find waypoints.yml
        try:
            package_dir = get_package_share_directory('lrauv_zigzag_controller')
            waypoints_path = os.path.join(package_dir, 'waypoints.yml')
            
            if not os.path.exists(waypoints_path):
                # Try in the package source directory
                package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                waypoints_path = os.path.join(package_dir, 'waypoints.yml')
            
            self.get_logger().info(f'Loading waypoints from: {waypoints_path}')
            with open(waypoints_path, 'r') as file:
                self.waypoints_data = yaml.safe_load(file)
                self.get_logger().info(f'Loaded waypoints: {self.waypoints_data}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
            self.waypoints_data = None
    
    def publish_waypoints(self):
        if not self.waypoints_data:
            return
            
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"
        
        for waypoint in self.waypoints_data.get('waypoints', []):
            pose = Pose()
            pose.position.x = float(waypoint[0])
            pose.position.y = float(waypoint[1])
            pose.position.z = float(waypoint[2])
            pose_array.poses.append(pose)
        
        self.publisher.publish(pose_array)
        
def main(args=None):
    rclpy.init(args=args)
    node = WaypointsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
