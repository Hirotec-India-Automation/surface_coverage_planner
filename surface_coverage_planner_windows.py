"""
3D Surface Coverage Path Planner - Windows Compatible Version
==============================================================

This version is optimized for Windows operating systems with:
- Cross-platform path handling using pathlib
- Windows-specific error handling
- Enhanced GUI with Windows styling
- Robust file I/O operations
- Optional ROS integration (works without ROS)

Author: Surface Coverage Planner Team
Version: Windows v1.0
License: MIT
"""

import os
import sys
from pathlib import Path

# Set PyVista to use Qt backend (works well on Windows)
os.environ["PYVISTA_USE_QT"] = "True"

import pyvista as pv
import numpy as np
import pandas as pd
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QFileDialog, QSlider, QMessageBox,
    QCheckBox, QProgressBar, QTextEdit, QGroupBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QIcon
from scipy.spatial.distance import cdist
from scipy.interpolate import splprep, splev

# Try to import ROS packages (optional on Windows)
try:
    import rospy
    from geometry_msgs.msg import PoseArray, Pose, Point
    from std_msgs.msg import Header
    ROS_AVAILABLE = True
    print("✅ ROS packages available")
except ImportError:
    ROS_AVAILABLE = False
    print("ℹ️ ROS not available - running in standalone mode")
    print("   (ROS is typically not used on Windows)")

# -------------------------------
# GLOBAL VARIABLES
# -------------------------------
mesh = None
selected_surface = None
path_points = []
MODEL_PATH = None
OUTPUT_DIR = Path.home() / "Documents" / "SurfaceCoveragePlanner"
OUTPUT_CSV = "coverage_path.csv"
ros_publisher = None
ui = None

# Create output directory if it doesn't exist
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
print(f"📁 Output directory: {OUTPUT_DIR}")


# -------------------------------
# HELPER FUNCTIONS
# -------------------------------
def check_line_collision(p1, p2, mesh, num_samples=20):
    """
    Check if line segment between p1 and p2 intersects with the mesh body.
    Returns True if collision detected (line passes through mesh).

    Args:
        p1: Start point of line segment
        p2: End point of line segment
        mesh: The mesh to check collision against
        num_samples: Number of points to sample along the line

    Returns:
        bool: True if collision detected, False otherwise
    """
    if mesh is None:
        return False

    try:
        # Create points along the line segment
        t = np.linspace(0, 1, num_samples)
        line_points = np.array([p1 + (p2 - p1) * ti for ti in t])

        # Check if any point is inside the mesh
        for point in line_points:
            # Use select_enclosed_points to check if point is inside mesh
            point_cloud = pv.PolyData(point.reshape(1, 3))
            selected = point_cloud.select_enclosed_points(
                mesh, tolerance=0.0, check_surface=False
            )

            if selected['SelectedPoints'][0] > 0:  # Point is inside mesh
                return True

        return False
    except Exception as e:
        print(f"⚠️ Warning: Collision check failed: {e}")
        return False


def smooth_path(points, smoothness=0.5):
    """
    Smooth the path using spline interpolation.

    Args:
        points: Nx3 array of path points
        smoothness: Smoothness factor (higher = smoother)

    Returns:
        Smoothed path points
    """
    if len(points) < 4:
        return points

    try:
        # Fit a spline to the points
        tck, u = splprep(
            [points[:, 0], points[:, 1], points[:, 2]],
            s=smoothness,
            k=min(3, len(points)-1)
        )

        # Evaluate spline at finer resolution
        u_fine = np.linspace(0, 1, len(points))
        smooth_points = splev(u_fine, tck)

        return np.column_stack(smooth_points)
    except Exception as e:
        print(f"⚠️ Spline smoothing failed: {e}, returning original points")
        return points


# -------------------------------
# PATH GENERATION FUNCTION
# -------------------------------
def generate_surface_coverage_points(surface, mesh_full, num_lines=20,
                                     num_points_per_line=20, smooth=True,
                                     collision_check=True, progress_callback=None):
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
        progress_callback: Optional callback function for progress updates

    Returns:
        numpy.ndarray: Array of path points (Nx3)
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
        # Update progress if callback provided
        if progress_callback:
            progress = int((i / num_lines) * 100)
            progress_callback(progress)

        y_seq = y_grid[::-1] if flip else y_grid
        flip = not flip

        line_points = []
        for y in y_seq:
            try:
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

                    intersection_points, intersection_cells = surface.ray_trace(
                        ray_start, ray_end
                    )

                    if len(intersection_points) > 0:
                        # Use the first intersection point (top of surface)
                        projected_point = intersection_points[0]
                    else:
                        # Fallback to closest point if ray trace fails
                        projected_point = center

                    line_points.append(projected_point)
            except Exception as e:
                print(f"⚠️ Warning: Failed to project point at ({x}, {y}): {e}")
                continue

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
        if progress_callback:
            progress_callback(95)
        path_array = smooth_path(path_array, smoothness=len(path_array) * 0.1)

    print(f"✅ Generated {len(path_array)} total path points")
    if progress_callback:
        progress_callback(100)

    return path_array


# -------------------------------
# ROS PUBLISHING FUNCTION
# -------------------------------
def publish_path_to_ros(path_points, frame_id="map"):
    """
    Publish path points to ROS as a PoseArray message.
    Note: ROS is typically not used on Windows, this is for compatibility.

    Args:
        path_points: Array of path points
        frame_id: ROS frame ID for the path

    Returns:
        bool: True if published successfully, False otherwise
    """
    if not ROS_AVAILABLE:
        print("ℹ️ ROS not available. Cannot publish path.")
        return False

    global ros_publisher

    try:
        # Initialize ROS node if not already initialized
        if not rospy.core.is_initialized():
            rospy.init_node('surface_coverage_planner', anonymous=True)
            print("✅ ROS node initialized")

        # Create publisher if not exists
        if ros_publisher is None:
            ros_publisher = rospy.Publisher(
                '/surface_coverage_path',
                PoseArray,
                queue_size=10,
                latch=True
            )
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
# PATH GENERATION THREAD (for responsiveness)
# -------------------------------
class PathGenerationThread(QThread):
    """
    Worker thread for path generation to keep GUI responsive on Windows.
    """
    progress = pyqtSignal(int)
    finished = pyqtSignal(np.ndarray)
    error = pyqtSignal(str)

    def __init__(self, surface, mesh_full, num_lines, num_points, smooth, collision_check):
        super().__init__()
        self.surface = surface
        self.mesh_full = mesh_full
        self.num_lines = num_lines
        self.num_points = num_points
        self.smooth = smooth
        self.collision_check = collision_check

    def run(self):
        try:
            path_points = generate_surface_coverage_points(
                self.surface,
                self.mesh_full,
                self.num_lines,
                self.num_points,
                smooth=self.smooth,
                collision_check=self.collision_check,
                progress_callback=self.progress.emit
            )
            self.finished.emit(path_points)
        except Exception as e:
            self.error.emit(str(e))


# -------------------------------
# CALLBACK WHEN SURFACE IS PICKED
# -------------------------------
def pick_callback(picked_mesh):
    """
    Called automatically when you click a surface in the 3D viewer.

    Args:
        picked_mesh: The mesh that was picked by the user
    """
    global mesh, path_points, ui

    if picked_mesh is None or picked_mesh.n_cells == 0:
        print("⚠️ No valid surface picked.")
        return

    # Make sure it's a clean surface
    picked_surface = picked_mesh.extract_surface().clean()
    print(f"✅ Picked surface: {picked_surface.n_points} points, {picked_surface.n_cells} cells")

    num_lines = ui.num_lines_slider.value()
    num_points = ui.num_points_slider.value()

    # Get options
    smooth = ui.smooth_checkbox.isChecked()
    collision_check = ui.collision_checkbox.isChecked()

    # Disable UI during generation
    ui.setEnabled(False)
    ui.progress_bar.setValue(0)
    ui.progress_bar.setVisible(True)
    ui.log_text.append("🔄 Generating coverage path...")

    # Create and start worker thread
    ui.worker = PathGenerationThread(
        picked_surface, mesh, num_lines, num_points, smooth, collision_check
    )
    ui.worker.progress.connect(ui.progress_bar.setValue)
    ui.worker.finished.connect(lambda points: on_path_generated(points, picked_surface))
    ui.worker.error.connect(on_path_error)
    ui.worker.start()


def on_path_generated(path_pts, picked_surface):
    """
    Called when path generation is complete.

    Args:
        path_pts: Generated path points
        picked_surface: The surface that was picked
    """
    global path_points, ui

    path_points = path_pts
    ui.log_text.append(f"✅ Generated {len(path_points)} path points")

    # Save points to CSV in the output directory
    output_file = OUTPUT_DIR / OUTPUT_CSV
    try:
        df = pd.DataFrame(path_points, columns=["x", "y", "z"])
        df.to_csv(output_file, index=False)
        ui.log_text.append(f"💾 Saved to {output_file}")
    except Exception as e:
        ui.log_text.append(f"❌ Failed to save CSV: {e}")

    # Publish to ROS if enabled
    if ui.ros_checkbox.isChecked() and ROS_AVAILABLE:
        if publish_path_to_ros(path_points):
            ui.log_text.append("✅ Published to ROS")
        else:
            ui.log_text.append("⚠️ Failed to publish to ROS")

    # Re-enable UI
    ui.setEnabled(True)
    ui.progress_bar.setVisible(False)

    # Visualize result
    visualize_path(picked_surface, path_points)


def on_path_error(error_msg):
    """
    Called when path generation encounters an error.

    Args:
        error_msg: The error message
    """
    global ui
    ui.log_text.append(f"❌ Error: {error_msg}")
    ui.setEnabled(True)
    ui.progress_bar.setVisible(False)
    QMessageBox.critical(ui, "Error", f"Path generation failed:\n{error_msg}")


def visualize_path(picked_surface, path_pts):
    """
    Visualize the generated path in a 3D viewer.

    Args:
        picked_surface: The surface that was selected
        path_pts: The generated path points
    """
    global mesh

    try:
        path_poly = pv.PolyData(path_pts)
        plot = pv.Plotter()
        plot.add_mesh(mesh, color="lightgray", opacity=0.4, label="Model")
        plot.add_mesh(picked_surface, color="orange", opacity=0.8, label="Selected Surface")
        plot.add_points(
            path_poly,
            color="blue",
            point_size=8,
            render_points_as_spheres=True,
            label="Path Points"
        )

        # Draw lines between consecutive points to show path
        if len(path_pts) > 1:
            lines = []
            for i in range(len(path_pts) - 1):
                lines.append([2, i, i + 1])
            lines = np.hstack(lines)
            path_line = pv.PolyData(path_pts)
            path_line.lines = lines
            plot.add_mesh(path_line, color="red", line_width=3, label="Coverage Path")

        plot.add_text("Surface Coverage Path Generated", font_size=12, position="upper_edge")
        plot.add_legend()
        plot.show()
    except Exception as e:
        print(f"❌ Visualization error: {e}")
        QMessageBox.warning(ui, "Visualization Error", f"Failed to display 3D view:\n{e}")


# -------------------------------
# MAIN APPLICATION UI
# -------------------------------
class SurfacePlannerUI(QWidget):
    """
    Main application window for the Surface Coverage Planner.
    Optimized for Windows with modern styling.
    """

    def __init__(self):
        super().__init__()
        self.worker = None
        self.init_ui()

    def init_ui(self):
        """Initialize the user interface."""
        self.setWindowTitle("3D Surface Coverage Path Planner - Windows Edition")
        self.setGeometry(100, 100, 600, 700)

        # Set Windows-friendly font
        font = QFont("Segoe UI", 9)
        self.setFont(font)

        layout = QVBoxLayout()

        # Title section
        title = QLabel("<h2>🧭 3D Surface Coverage Path Planner</h2>")
        subtitle = QLabel("Windows Compatible Version")
        subtitle.setStyleSheet("color: #0078D7; font-size: 10pt; font-weight: bold;")
        description = QLabel(
            "Select a 3D model and click on a surface region to generate coverage paths."
        )
        description.setStyleSheet("color: gray; font-size: 9pt;")

        # File selector group
        file_group = QGroupBox("1. Load 3D Model")
        file_layout = QVBoxLayout()

        file_select_layout = QHBoxLayout()
        self.file_label = QLabel("No model selected")
        self.file_label.setStyleSheet("padding: 5px; background: white; border: 1px solid #ccc;")
        browse_btn = QPushButton("📂 Browse Model")
        browse_btn.clicked.connect(self.load_model)
        browse_btn.setStyleSheet("padding: 5px;")
        file_select_layout.addWidget(self.file_label, 3)
        file_select_layout.addWidget(browse_btn, 1)

        file_layout.addLayout(file_select_layout)
        file_group.setLayout(file_layout)

        # Path parameters group
        params_group = QGroupBox("2. Path Parameters")
        params_layout = QVBoxLayout()

        # Sliders
        self.num_lines_slider = QSlider(Qt.Horizontal)
        self.num_lines_slider.setRange(5, 100)
        self.num_lines_slider.setValue(20)

        self.num_points_slider = QSlider(Qt.Horizontal)
        self.num_points_slider.setRange(5, 100)
        self.num_points_slider.setValue(20)

        self.lines_label = QLabel("Number of Lines: 20")
        self.points_label = QLabel("Points per Line: 20")

        # Update labels when sliders change
        self.num_lines_slider.valueChanged.connect(
            lambda v: self.lines_label.setText(f"Number of Lines: {v}")
        )
        self.num_points_slider.valueChanged.connect(
            lambda v: self.points_label.setText(f"Points per Line: {v}")
        )

        params_layout.addWidget(self.lines_label)
        params_layout.addWidget(self.num_lines_slider)
        params_layout.addWidget(self.points_label)
        params_layout.addWidget(self.num_points_slider)
        params_group.setLayout(params_layout)

        # Options group
        options_group = QGroupBox("3. Options")
        options_layout = QVBoxLayout()

        self.smooth_checkbox = QCheckBox("Apply Path Smoothing")
        self.smooth_checkbox.setChecked(True)
        self.smooth_checkbox.setToolTip("Smooth the generated path using spline interpolation")

        self.collision_checkbox = QCheckBox("Enable Collision Detection")
        self.collision_checkbox.setChecked(True)
        self.collision_checkbox.setToolTip("Check that path segments don't pass through the mesh body")

        self.ros_checkbox = QCheckBox("Publish to ROS (if available)")
        self.ros_checkbox.setChecked(ROS_AVAILABLE)
        self.ros_checkbox.setEnabled(ROS_AVAILABLE)
        if ROS_AVAILABLE:
            self.ros_checkbox.setToolTip("Publish path to ROS topic: /surface_coverage_path")
        else:
            self.ros_checkbox.setToolTip("ROS not available (typically not used on Windows)")

        options_layout.addWidget(self.smooth_checkbox)
        options_layout.addWidget(self.collision_checkbox)
        options_layout.addWidget(self.ros_checkbox)
        options_group.setLayout(options_layout)

        # Progress bar (hidden by default)
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)

        # Log text area
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        self.log_text.append(f"📁 Output directory: {OUTPUT_DIR}")
        self.log_text.append("✅ Application initialized")
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)

        # Action buttons
        start_btn = QPushButton("🚀 Launch 3D Viewer")
        start_btn.clicked.connect(self.launch_viewer)
        start_btn.setStyleSheet(
            "background-color: #0078D7; color: white; font-weight: bold; padding: 10px;"
        )

        # Add all components to main layout
        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addWidget(description)
        layout.addWidget(QLabel(""))  # Spacer
        layout.addWidget(file_group)
        layout.addWidget(params_group)
        layout.addWidget(options_group)
        layout.addWidget(self.progress_bar)
        layout.addWidget(log_group)
        layout.addWidget(start_btn)

        self.setLayout(layout)

        # Windows-friendly styling
        self.setStyleSheet("""
            QWidget {
                background-color: #f4f6f8;
                font-family: Segoe UI;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #0078D7;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                color: #0078D7;
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QLabel {
                font-size: 9pt;
            }
            QPushButton {
                border-radius: 5px;
                padding: 8px;
                background-color: #e0e0e0;
            }
            QPushButton:hover {
                background-color: #d0d0d0;
            }
            QCheckBox {
                font-size: 9pt;
                padding: 4px;
            }
        """)

    def load_model(self):
        """Load a 3D model file."""
        global MODEL_PATH, mesh

        # Open file dialog with Windows-friendly path handling
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select 3D Model",
            str(Path.home()),
            "3D Model Files (*.stl *.obj);;STL Files (*.stl);;OBJ Files (*.obj);;All Files (*.*)"
        )

        if file_path:
            try:
                # Convert to Path object for cross-platform compatibility
                MODEL_PATH = Path(file_path)
                self.file_label.setText(MODEL_PATH.name)
                self.log_text.append(f"📂 Loading: {MODEL_PATH}")

                # Load the mesh
                mesh = pv.read(str(MODEL_PATH))

                self.log_text.append(
                    f"✅ Model loaded: {mesh.n_points} points, {mesh.n_cells} cells"
                )
                QMessageBox.information(
                    self,
                    "Model Loaded",
                    "✅ 3D model loaded successfully!\n\n"
                    f"Points: {mesh.n_points}\n"
                    f"Cells: {mesh.n_cells}"
                )
            except Exception as e:
                self.log_text.append(f"❌ Failed to load model: {e}")
                QMessageBox.critical(
                    self,
                    "Error",
                    f"Failed to load model:\n\n{e}\n\n"
                    f"Please ensure the file is a valid STL or OBJ file."
                )

    def launch_viewer(self):
        """Launch the 3D viewer for surface selection."""
        global mesh

        if mesh is None:
            QMessageBox.warning(
                self,
                "No Model",
                "⚠️ Please select and load a 3D model first!"
            )
            return

        try:
            self.log_text.append("🚀 Launching 3D viewer...")
            self.log_text.append("💡 Click on a surface to generate coverage path")

            plotter = pv.Plotter()
            plotter.add_mesh(mesh, color="lightgray", label="3D Model")
            plotter.add_text(
                "Click on a surface to generate coverage path",
                font_size=10,
                position="upper_edge"
            )
            plotter.enable_cell_picking(
                callback=pick_callback,
                through=False,           # only front-facing cell
                show_message=True,
                style="surface",         # ensures surface-style picking
                left_clicking=True,      # enables single-click picking
            )
            plotter.show()

            self.log_text.append("✅ Viewer closed")

        except Exception as e:
            self.log_text.append(f"❌ Failed to launch viewer: {e}")
            QMessageBox.critical(
                self,
                "Viewer Error",
                f"Failed to launch 3D viewer:\n\n{e}\n\n"
                f"This may be due to graphics driver issues on Windows."
            )


# -------------------------------
# ENTRY POINT
# -------------------------------
def main():
    """Main entry point for the application."""
    print("=" * 60)
    print("3D Surface Coverage Path Planner - Windows Edition")
    print("=" * 60)
    print(f"Python version: {sys.version}")
    print(f"PyVista version: {pv.__version__}")
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"ROS Available: {ROS_AVAILABLE}")
    print("=" * 60)

    app = QApplication(sys.argv)

    # Set Windows-specific application properties
    app.setApplicationName("Surface Coverage Planner")
    app.setOrganizationName("SurfacePlanner")
    app.setStyle("Fusion")  # Modern style that works well on Windows

    global ui
    ui = SurfacePlannerUI()
    ui.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
