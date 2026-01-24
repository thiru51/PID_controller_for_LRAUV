from paraview.simple import *
import math
import os

# --- 1. Load Robot Trajectory ---
traj_path = r"/home/thiru_na22b078/ros2_ws/src/PID_PRO_controller/output/trajectory.csv"
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
    print(f"Error: Trajectory file not found at {traj_path}")

# --- 2. Load Waypoints (if exist) ---
wps_path = r"/home/thiru_na22b078/ros2_ws/src/PID_PRO_controller/output/waypoints.csv"
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
SaveScreenshot(r"/home/thiru_na22b078/ros2_ws/src/PID_PRO_controller/output/trajectory_3d.png", renderView, ImageResolution=[1920,1080])
print(r"Saved PNG: /home/thiru_na22b078/ros2_ws/src/PID_PRO_controller/output/trajectory_3d.png")
