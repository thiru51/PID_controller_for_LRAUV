import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_path = get_package_share_directory('pid_controller_1')
    world_path = os.path.join(pkg_path, 'worlds', 'buoyant_lrauv.sdf')
    urdf_path = os.path.join(pkg_path, 'urdf', 'my_lrauv.urdf')
    bridge_yaml = os.path.join(pkg_path, 'bridge', 'bridge_config.yaml')
    model_path = os.path.join(pkg_path, 'models', 'my_lrauv', 'model.sdf')  # Full path to model.sdf

    return LaunchDescription([
        # Launch Gazebo Harmonic with the world
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': open(urdf_path).read()},
                {'use_sim_time': True}
            ],
        ),

        # ROS-Gazebo bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(pkg_path, 'bridge', 'bridge_config.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            output='screen'
        ),


        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='pid_controller_1',
                    executable='trajectory_plotter',
                    name='trajectory_plotter',
                    output='screen'
                )
            ]
        ),

        # PID controller node (after delay)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='pid_controller_1',
                    executable='pid_controller',
                    name='pid_controller',
                    output='screen'
                )
            ]
        ),
    ])