import os
os.environ["PYVISTA_USE_QT"] = "True"

import pyvista as pv
import numpy as np
import pandas as pd
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QFileDialog, QSlider, QMessageBox, QCheckBox
)
from PyQt5.QtCore import Qt
import sys
from scipy.spatial.distance import cdist
from scipy.interpolate import splprep, splev

# Try to import ROS packages
try:
    import rospy
    from geometry_msgs.msg import PoseArray, Pose, Point
    from std_msgs.msg import Header
    ROS_AVAILABLE = True
    print("✅ ROS packages available")
except ImportError:
    ROS_AVAILABLE = False
    print("⚠️ ROS not available. Install with: pip install rospy geometry_msgs std_msgs")
    print("   Or ensure ROS environment is sourced.")

# -------------------------------
# GLOBAL VARIABLES
# -------------------------------
mesh = None
selected_surface = None
path_points = []
MODEL_PATH = None
OUTPUT_CSV = "coverage_path.csv"
ros_publisher = None
ui = None


# -------------------------------
# HELPER FUNCTIONS
# -------------------------------
def check_line_collision(p1, p2, mesh, num_samples=20):
    """
    Check if line segment between p1 and p2 intersects with the mesh body.
    Returns True if collision detected (line passes through mesh).
    """
    if mesh is None:
        return False

    # Create points along the line segment
    t = np.linspace(0, 1, num_samples)
    line_points = np.array([p1 + (p2 - p1) * ti for ti in t])

    # Check if any point is inside the mesh
    for point in line_points:
        # Use select_enclosed_points to check if point is inside mesh
        point_cloud = pv.PolyData(point.reshape(1, 3))
        selected = point_cloud.select_enclosed_points(mesh, tolerance=0.0, check_surface=False)

        if selected['SelectedPoints'][0] > 0:  # Point is inside mesh
            return True

    return False


def smooth_path(points, smoothness=0.5):
    """
    Smooth the path using spline interpolation.
    """
    if len(points) < 4:
        return points

    try:
        # Fit a spline to the points
        tck, u = splprep([points[:, 0], points[:, 1], points[:, 2]], s=smoothness, k=min(3, len(points)-1))

        # Evaluate spline at finer resolution
        u_fine = np.linspace(0, 1, len(points))
        smooth_points = splev(u_fine, tck)

        return np.column_stack(smooth_points)
    except:
        print("⚠️ Spline smoothing failed, returning original points")
        return points


# -------------------------------
# PATH GENERATION FUNCTION
# -------------------------------
def generate_surface_coverage_points(surface, mesh_full, num_lines=20, num_points_per_line=20,
                                     smooth=True, collision_check=True):
    """
    Generate coverage path points directly on a curved 3D surface.
    The path will follow the local surface and respect curvature.
    Ensures path doesn't penetrate the mesh body.

    Args:
        surface: The selected surface mesh
        mesh_full: The complete mesh for collision detection
        num_lines: Number of sweep lines
        num_points_per_line: Number of points per line
        smooth: Apply path smoothing
        collision_check: Check for collisions with mesh body
    """
    print(f"🔧 Generating coverage path with {num_lines} lines and {num_points_per_line} points per line")

    bounds = surface.bounds
    x_min, x_max, y_min, y_max, z_min, z_max = bounds

    # Create a planar grid over the bounding box in X-Y plane
    x_grid = np.linspace(x_min, x_max, num_lines)
    y_grid = np.linspace(y_min, y_max, num_points_per_line)

    path_points = []
    flip = False

    for i, x in enumerate(x_grid):
        y_seq = y_grid[::-1] if flip else y_grid
        flip = not flip

        line_points = []
        for y in y_seq:
            # Create a vertical ray from above to project onto surface
            z_start = z_max + (z_max - z_min) * 0.5  # Start above the surface
            point_above = np.array([x, y, z_start])

            # Find closest point on the surface
            closest_point_id = surface.find_closest_cell(point_above)

            if closest_point_id >= 0:
                # Get the actual point coordinates on the surface
                cell = surface.extract_cells(closest_point_id)
                center = cell.center

                # Project point onto surface more accurately
                # Cast a ray from above and find intersection
                ray_start = np.array([x, y, z_max + (z_max - z_min)])
                ray_end = np.array([x, y, z_min - (z_max - z_min)])

                intersection_points, intersection_cells = surface.ray_trace(ray_start, ray_end)

                if len(intersection_points) > 0:
                    # Use the first intersection point (top of surface)
                    projected_point = intersection_points[0]
                else:
                    # Fallback to closest point if ray trace fails
                    projected_point = center

                line_points.append(projected_point)

        # Check for collisions between consecutive points on this line
        if collision_check and len(line_points) > 1:
            valid_points = [line_points[0]]
            for j in range(1, len(line_points)):
                if not check_line_collision(line_points[j-1], line_points[j], mesh_full):
                    valid_points.append(line_points[j])
                else:
                    print(f"⚠️ Collision detected at line {i}, point {j} - skipping")
            line_points = valid_points

        path_points.extend(line_points)

        print(f"  Line {i+1}/{num_lines}: {len(line_points)} valid points")

    path_array = np.array(path_points)

    # Smooth the path if requested
    if smooth and len(path_array) > 3:
        print("🔄 Smoothing path...")
        path_array = smooth_path(path_array, smoothness=len(path_array) * 0.1)

    print(f"✅ Generated {len(path_array)} total path points")
    return path_array


# -------------------------------
# ROS PUBLISHING FUNCTION
# -------------------------------
def publish_path_to_ros(path_points, frame_id="map"):
    """
    Publish path points to ROS as a PoseArray message.
    """
    if not ROS_AVAILABLE:
        print("⚠️ ROS not available. Cannot publish path.")
        return False

    global ros_publisher

    try:
        # Initialize ROS node if not already initialized
        if not rospy.core.is_initialized():
            rospy.init_node('surface_coverage_planner', anonymous=True)
            print("✅ ROS node initialized")

        # Create publisher if not exists
        if ros_publisher is None:
            ros_publisher = rospy.Publisher('/surface_coverage_path', PoseArray, queue_size=10, latch=True)
            rospy.sleep(0.5)  # Give time for publisher to connect
            print("✅ ROS publisher created on topic: /surface_coverage_path")

        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = frame_id

        # Convert path points to poses
        for point in path_points:
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = float(point[2])
            # Set default orientation (can be customized based on path direction)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        # Publish the message
        ros_publisher.publish(pose_array)
        print(f"✅ Published {len(path_points)} poses to ROS topic: /surface_coverage_path")
        return True

    except Exception as e:
        print(f"❌ Failed to publish to ROS: {e}")
        return False


# -------------------------------
# CALLBACK WHEN SURFACE IS PICKED
# -------------------------------
def pick_callback(picked_mesh):
    """Called automatically when you click a surface."""
    global mesh, path_points, ui

    if picked_mesh is None or picked_mesh.n_cells == 0:
        print("⚠️ No valid surface picked.")
        return

    # Make sure it's a clean surface
    picked_surface = picked_mesh.extract_surface().clean()
    print(f"✅ Picked surface: {picked_surface.n_points} points, {picked_surface.n_cells} cells")

    num_lines = ui.num_lines_slider.value()
    num_points = ui.num_points_slider.value()

    # Get smoothing and collision check options
    smooth = ui.smooth_checkbox.isChecked() if hasattr(ui, 'smooth_checkbox') else True
    collision_check = ui.collision_checkbox.isChecked() if hasattr(ui, 'collision_checkbox') else True

    # Generate coverage points directly on this surface
    path_points = generate_surface_coverage_points(
        picked_surface,
        mesh,
        num_lines,
        num_points,
        smooth=smooth,
        collision_check=collision_check
    )
    print(f"✅ Generated {len(path_points)} path points")

    # Save points to CSV
    df = pd.DataFrame(path_points, columns=["x", "y", "z"])
    df.to_csv(OUTPUT_CSV, index=False)
    print(f"💾 Saved to {OUTPUT_CSV}")

    # Publish to ROS if enabled
    if ui.ros_checkbox.isChecked() if hasattr(ui, 'ros_checkbox') else False:
        publish_path_to_ros(path_points)

    # Visualize result
    path_poly = pv.PolyData(path_points)
    plot = pv.Plotter()
    plot.add_mesh(mesh, color="lightgray", opacity=0.4)
    plot.add_mesh(picked_surface, color="orange", opacity=0.8)
    plot.add_points(path_poly, color="blue", point_size=8, render_points_as_spheres=True)

    # Draw lines between consecutive points to show path
    if len(path_points) > 1:
        lines = []
        for i in range(len(path_points) - 1):
            lines.append([2, i, i + 1])
        lines = np.hstack(lines)
        path_line = pv.PolyData(path_points)
        path_line.lines = lines
        plot.add_mesh(path_line, color="red", line_width=3, label="Coverage Path")

    plot.add_text("Surface Coverage Path Generated", font_size=12)
    plot.add_legend()
    plot.show()


# -------------------------------
# MAIN APPLICATION UI
# -------------------------------
class SurfacePlannerUI(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("🧭 3D Surface Coverage Path Planner")
        self.setGeometry(200, 150, 480, 450)

        layout = QVBoxLayout()

        title = QLabel("<h2>3D Surface Coverage Planner</h2>")
        subtitle = QLabel("Select a 3D model and click a surface region to generate paths.")
        subtitle.setStyleSheet("color: gray; font-size: 10pt;")

        # File selector
        file_layout = QHBoxLayout()
        self.file_label = QLabel("No model selected")
        browse_btn = QPushButton("📂 Browse Model")
        browse_btn.clicked.connect(self.load_model)
        file_layout.addWidget(self.file_label)
        file_layout.addWidget(browse_btn)

        # Sliders
        self.num_lines_slider = QSlider(Qt.Horizontal)
        self.num_lines_slider.setRange(5, 100)
        self.num_lines_slider.setValue(20)

        self.num_points_slider = QSlider(Qt.Horizontal)
        self.num_points_slider.setRange(5, 100)
        self.num_points_slider.setValue(20)

        lines_label = QLabel("Number of Lines: 20")
        points_label = QLabel("Points per Line: 20")

        # Update labels when sliders change
        self.num_lines_slider.valueChanged.connect(
            lambda v: lines_label.setText(f"Number of Lines: {v}")
        )
        self.num_points_slider.valueChanged.connect(
            lambda v: points_label.setText(f"Points per Line: {v}")
        )

        # Checkboxes for options
        options_label = QLabel("<b>Options:</b>")
        self.smooth_checkbox = QCheckBox("Apply Path Smoothing")
        self.smooth_checkbox.setChecked(True)
        self.smooth_checkbox.setToolTip("Smooth the generated path using spline interpolation")

        self.collision_checkbox = QCheckBox("Enable Collision Detection")
        self.collision_checkbox.setChecked(True)
        self.collision_checkbox.setToolTip("Check that path segments don't pass through the mesh body")

        self.ros_checkbox = QCheckBox("Publish to ROS")
        self.ros_checkbox.setChecked(ROS_AVAILABLE)
        self.ros_checkbox.setEnabled(ROS_AVAILABLE)
        if ROS_AVAILABLE:
            self.ros_checkbox.setToolTip("Publish path to ROS topic: /surface_coverage_path")
        else:
            self.ros_checkbox.setToolTip("ROS not available - install rospy to enable")

        start_btn = QPushButton("🚀 Launch Viewer")
        start_btn.clicked.connect(self.launch_viewer)
        start_btn.setStyleSheet("background-color: #0078D7; color: white; font-weight: bold;")

        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addLayout(file_layout)
        layout.addWidget(QLabel(""))  # Spacer
        layout.addWidget(lines_label)
        layout.addWidget(self.num_lines_slider)
        layout.addWidget(points_label)
        layout.addWidget(self.num_points_slider)
        layout.addWidget(QLabel(""))  # Spacer
        layout.addWidget(options_label)
        layout.addWidget(self.smooth_checkbox)
        layout.addWidget(self.collision_checkbox)
        layout.addWidget(self.ros_checkbox)
        layout.addWidget(QLabel(""))  # Spacer
        layout.addWidget(start_btn)

        self.setLayout(layout)
        self.setStyleSheet("""
            QWidget { background-color: #f4f6f8; font-family: Segoe UI; }
            QLabel { font-size: 11pt; }
            QPushButton { border-radius: 8px; padding: 8px; }
            QCheckBox { font-size: 10pt; padding: 4px; }
        """)

    def load_model(self):
        global MODEL_PATH, mesh
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select 3D Model", "", "3D Files (*.stl *.obj)"
        )
        if file_path:
            MODEL_PATH = file_path
            self.file_label.setText(os.path.basename(file_path))
            try:
                mesh = pv.read(MODEL_PATH)
                QMessageBox.information(self, "Model Loaded", "✅ 3D model loaded successfully!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load model:\n{e}")

    def launch_viewer(self):
        global mesh
        if mesh is None:
            QMessageBox.warning(self, "No Model", "⚠️ Please select a model first!")
            return

        plotter = pv.Plotter()
        plotter.add_mesh(mesh, color="lightgray")
        plotter.add_text("Click on a surface to generate coverage path", font_size=10)
        plotter.enable_cell_picking(
        callback=pick_callback,
        through=False,           # only front-facing cell
        show_message=True,
        style="surface",         # ensures surface-style picking
)
        plotter.show()


# -------------------------------
# ENTRY POINT
# -------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = SurfacePlannerUI()
    ui.show()
    sys.exit(app.exec_())
