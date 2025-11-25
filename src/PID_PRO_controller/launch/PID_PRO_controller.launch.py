#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Path to your world file
    world_path = os.path.join(
        os.path.expanduser("~"),
        "ros2_ws", "src", "PID_PRO_controller", "worlds", "buoyant_lrauv.sdf"
    )

    # Path to bridge YAML
    pkg_share = get_package_share_directory('PID_PRO_controller')
    bridge_yaml = os.path.join(pkg_share, 'bridge', 'bridge_config.yaml')
    # The waypoints file in this workspace is under the python package directory.
    # Use the explicit workspace path so the node can find it during development.
    waypoints_path = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'src', 'PID_PRO_controller', 'PID_PRO_controller', 'waypoints.yaml'
    )

    # ros_gz_bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lrauv_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen'
    )



    return LaunchDescription([
        # Launch Gazebo with the world
        ExecuteProcess(
            cmd=["gz", "sim", world_path],
            output="screen"
        ),

        # Start the bridge
        ros_gz_bridge,

        Node(
            package='PID_PRO_controller',
            executable='lrauv_3d_los_controller',
            name='lrauv_3d_los_controller',
            output='screen',
            parameters=[{'waypoints_file': waypoints_path}]
        ),

        Node(
            package='PID_PRO_controller',
            executable='trajectory_plotter',
            name='trajectory_plotter',
            output='screen',
            parameters=[{'waypoints_file': waypoints_path}]
        )

    ])
