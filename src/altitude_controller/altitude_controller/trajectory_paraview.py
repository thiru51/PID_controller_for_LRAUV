#!/usr/bin/env python3
"""
trajectory_paraview.py

Logs /odom trajectory in 3D (x,y,z), reads waypoints from waypoints.yaml,
writes CSV + ParaView script, runs ParaView in batch mode to render a PNG.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os
import yaml
import subprocess
from pathlib import Path
import shutil
import sys
import math

# ---------- CONFIGURATION ----------
# dynamically find the folder where this script lives
SCRIPT_DIR = Path(__file__).resolve().parent

# Assume waypoints.yaml is next to this script
WAYPOINTS_FILE = SCRIPT_DIR / "waypoints.yaml"

# Output folder (creates 'output' folder inside the package dir)
OUT_DIR = SCRIPT_DIR.parent / "output"
OUT_DIR.mkdir(parents=True, exist_ok=True)

CSV_FILE = OUT_DIR / "trajectory.csv"
WPTS_CSV = OUT_DIR / "waypoints.csv"
PV_SCRIPT_FILE = OUT_DIR / "render_paraview.py"
PNG_FILE = OUT_DIR / "trajectory_3d.png"

# Logging / sampling
DEFAULT_LOG_RATE_HZ = 20.0 

# ---------- Helper utilities ----------
def which_first(names):
    """Return first available executable in PATH or None."""
    for n in names:
        p = shutil.which(n)
        if p:
            return p
    return None

# ---------- Node ----------
class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__("trajectory_logger")

        self.declare_parameter("log_rate", DEFAULT_LOG_RATE_HZ)
        self.log_rate = float(self.get_parameter("log_rate").value)

        self.get_logger().info(f"TrajectoryLogger starting (log_rate={self.log_rate} Hz).")
        self.get_logger().info(f"Output Directory: {OUT_DIR}")

        # odom subscription
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        # internal buffers
        self.last_pose = None  # (x,y,z)
        self.traj = []  # list of [x,y,z]

        # load waypoints
        self.waypoints = self._load_waypoints(WAYPOINTS_FILE)
        if not self.waypoints:
            self.get_logger().warning(f"No waypoints loaded from {WAYPOINTS_FILE}. Plotting skipped.")

        # timer to sample the latest pose
        self.timer = self.create_timer(1.0 / self.log_rate, self._on_timer)

        # detect pvpython/pvbatch and xdg-open
        self.pv_exec = which_first(["pvpython", "pvbatch"])
        if self.pv_exec:
            self.get_logger().info(f"Found ParaView executable: {self.pv_exec}")
        else:
            self.get_logger().warning("No 'pvpython' found. Visualization will not render.")

        self.xdg_open = which_first(["xdg-open", "gio", "open"])

    # ----------------------------
    def _load_waypoints(self, fp):
        path_obj = Path(fp)
        if not path_obj.exists():
            self.get_logger().warning(f"Waypoints file not found: {fp}")
            return []
            
        try:
            with open(path_obj, "r") as fh:
                data = yaml.safe_load(fh)
            
            # FIX: Handle empty file or None result
            if data is None:
                return []
                
            raw = data.get("waypoints", [])
            out = []
            for entry in raw:
                if isinstance(entry, (list, tuple)) and len(entry) >= 3:
                    out.append([float(entry[0]), float(entry[1]), float(entry[2])])
            return out
        except Exception as e:
            self.get_logger().error(f"Failed to read waypoints file '{fp}': {e}")
            return []

    # ----------------------------
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.last_pose = (float(p.x), float(p.y), float(p.z))

    # ----------------------------
    def _on_timer(self):
        if self.last_pose is not None:
            self.traj.append([self.last_pose[0], self.last_pose[1], self.last_pose[2]])

    # ----------------------------
    def dump_outputs(self):
        if len(self.traj) == 0:
            self.get_logger().warning("No trajectory points recorded. Nothing to write.")
            return False

        try:
            self.get_logger().info(f"Writing trajectory CSV -> {CSV_FILE}")
            with open(CSV_FILE, "w", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow(["x", "y", "z"])
                writer.writerows(self.traj)
        except Exception as e:
            self.get_logger().error(f"Failed to write trajectory CSV: {e}")
            return False

        if self.waypoints:
            try:
                self.get_logger().info(f"Writing waypoints CSV -> {WPTS_CSV}")
                with open(WPTS_CSV, "w", newline="") as fh:
                    writer = csv.writer(fh)
                    writer.writerow(["wpx", "wpy", "wpz"])
                    writer.writerows(self.waypoints)
            except Exception as e:
                self.get_logger().warning(f"Failed to write waypoints CSV: {e}")

        return True

    # ----------------------------
    def generate_paraview_script(self):
        self.get_logger().info(f"Generating ParaView script -> {PV_SCRIPT_FILE}")
        
        # FIX: Removed Delaunay1D. Using Points representation instead.
        script = f'''from paraview.simple import *
import math
import os

# --- 1. Load Robot Trajectory ---
traj_path = r"{CSV_FILE}"
if os.path.exists(traj_path):
    traj = CSVReader(FileName=[traj_path])
    trajTable = TableToPoints(Input=traj)
    trajTable.XColumn = "x"
    trajTable.YColumn = "y"
    trajTable.ZColumn = "z"

    # Set up View
    renderView = GetActiveViewOrCreate('RenderView')
    renderView.Background = [1.0, 1.0, 1.0] # White background

    # Show Trajectory as Points
    trajDisplay = Show(trajTable, renderView)
    trajDisplay.Representation = 'Points'
    trajDisplay.PointSize = 4.0
    
    # Color by Z-height
    ColorBy(trajDisplay, ('POINTS', 'z'))
    trajDisplay.SetScalarBarVisibility(renderView, True)
else:
    print(f"Error: Trajectory file not found at {{traj_path}}")

# --- 2. Load Waypoints (if exist) ---
wps_path = r"{WPTS_CSV}"
if os.path.exists(wps_path):
    wps = CSVReader(FileName=[wps_path])
    wpsTable = TableToPoints(Input=wps)
    wpsTable.XColumn = "wpx"
    wpsTable.YColumn = "wpy"
    wpsTable.ZColumn = "wpz"
    
    # Show Waypoints as Spheres (Glyphs)
    glyph = Glyph(Input=wpsTable, GlyphType='Sphere')
    glyph.GlyphType.Radius = 0.15  # Fixed radius for visibility
    glyphDisplay = Show(glyph, renderView)
    glyphDisplay.DiffuseColor = [1.0, 0.0, 0.0] # Red

# --- 3. Reset Camera & Save ---
ResetCamera()
# Optional: Zoom out slightly
GetActiveCamera().Dolly(0.8)

Render()
SaveScreenshot(r"{PNG_FILE}", renderView, ImageResolution=[1920,1080])
print(r"Saved PNG: {PNG_FILE}")
'''
        try:
            with open(PV_SCRIPT_FILE, "w") as fh:
                fh.write(script)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to write ParaView script: {e}")
            return False

    # ----------------------------
    def run_paraview_and_open(self):
        if not self.pv_exec:
            return False

        cmd = [self.pv_exec, str(PV_SCRIPT_FILE)]
        self.get_logger().info(f"Running ParaView...")
        try:
            # Run ParaView
            subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.get_logger().info("ParaView rendering finished.")
            
            # Open Image
            if self.xdg_open and PNG_FILE.exists():
                subprocess.Popen([self.xdg_open, str(PNG_FILE)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            else:
                self.get_logger().info(f"Image saved at: {PNG_FILE}")
                
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"ParaView Error:\n{e.stderr.decode('utf-8', errors='ignore')}")
            return False
        except Exception as e:
            self.get_logger().error(f"Execution Error: {e}")
            return False
        return True

    # ----------------------------
    def destroy_node(self):
        self.get_logger().info("Shutting down... Dumping data & rendering.")
        try:
            if self.dump_outputs():
                if self.generate_paraview_script():
                    self.run_paraview_and_open()
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
        super().destroy_node()


# ---------- main ----------
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Check if rclpy is still valid before shutting down
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()