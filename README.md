# PID_controller_for_LRAUV

Short description: PID controller + Gazebo LRAUV setup for ROS 2 (jazzy).

## Quick Start

Build, source and launch the `pid_controller` package from your workspace:

```bash
# Build only the pid_controller package (use --symlink-install for development)
cd ~/ros2_ws && colcon build --symlink-install --packages-select pid_controller

# Source ROS 2 and your workspace overlay, then launch
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && \
	ros2 launch pid_controller pid_controller.launch.py
```

Notes:
- Replace `jazzy` with your ROS 2 distribution if different.
- If you want to build the whole workspace, omit `--packages-select pid_controller`.
- To run the trajectory plotter script directly:
```bash
python3 ~/ros2_ws/src/pid_controller/pid_controller/trajectory_plotter.py
```


