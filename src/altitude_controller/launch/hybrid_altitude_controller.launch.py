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
        "ros2_ws", "src", "altitude_controller", "worlds", "sydney_regatta_lrauv.sdf"
    )

    # Path to bridge YAML
    pkg_share = get_package_share_directory('altitude_controller')
    bridge_yaml = os.path.join(pkg_share, 'bridge', 'bridge_config.yaml')
    # The waypoints file in this workspace is under the python package directory.
    # Use the explicit workspace path so the node can find it during development.
    waypoints_path = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'src', 'altitude_controller', 'altitude_controller', 'waypoints.yaml'
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
            package='altitude_controller',
            executable='lrauv_xy_altitude_mission',
            name='lrauv_xy_altitude_mission',
            output='screen',
            parameters=[{'waypoints_file': waypoints_path}]
        ),

        Node(
            package='altitude_controller',
            executable='lrauv_altitude_debugger',
            name='altitude_debugger',
            output='screen',
            parameters=[{'waypoints_file': waypoints_path}]
            ),     

        Node(
            package='altitude_controller',
            executable='trajectory_paraview',
            name='trajectory_logger'
        ),

        Node(
            package='altitude_controller',
            executable='trajectory_plotter',
            name='trajectory_plotter',
            output='screen',
            parameters=[{'waypoints_file': waypoints_path}]
        ),
        # Node(
        #     package='altitude_controller',
        #     executable='sonar_image_node',
        #     name='sonar_image_node',
        #     output='screen'
        # )

    ])
