import os
os.environ["PYVISTA_USE_QT"] = "True"

import pyvista as pv
import numpy as np
import pandas as pd
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QFileDialog, QSlider, QMessageBox
)
from PyQt5.QtCore import Qt
import sys

# -------------------------------
# GLOBAL VARIABLES
# -------------------------------
mesh = None
selected_surface = None
path_points = []
MODEL_PATH = None
OUTPUT_CSV = "coverage_path.csv"


# -------------------------------
# PATH GENERATION FUNCTION
# -------------------------------
def generate_surface_coverage_points(surface, num_lines=20, num_points_per_line=20):
    """
    Generate coverage path points directly on a curved 3D surface.
    The path will follow the local surface and respect curvature.
    """

    bounds = surface.bounds
    x_min, x_max, y_min, y_max, z_min, z_max = bounds

    # Create a planar grid over the bounding box in X-Y plane
    x_grid = np.linspace(x_min, x_max, num_lines)
    y_grid = np.linspace(y_min, y_max, num_points_per_line)

    path_points = []
    flip = False

    for x in x_grid:
        y_seq = y_grid[::-1] if flip else y_grid
        flip = not flip
        for y in y_seq:
            z = (z_min + z_max) / 2.0
            point = np.array([x, y, z])
            # Project point onto surface (find closest point)
            projected_point = surface.find_closest_point(point)
            path_points.append(surface.points[projected_point])

    return np.array(path_points)


# -------------------------------
# CALLBACK WHEN SURFACE IS PICKED
# -------------------------------
def pick_callback(picked_mesh):
    """Called automatically when you click a surface."""
    global mesh, path_points, ui

    if picked_mesh is None or picked_mesh.n_cells == 0:
        print("⚠️ No valid surface picked.")
        return

    # Make sure it’s a clean surface
    picked_surface = picked_mesh.extract_surface().clean()
    print(f"✅ Picked surface: {picked_surface.n_points} points, {picked_surface.n_cells} cells")

    num_lines = ui.num_lines_slider.value()
    num_points = ui.num_points_slider.value()

    # Generate coverage points directly on this surface
    path_points = generate_surface_coverage_points(picked_surface, num_lines, num_points)
    print(f"✅ Generated {len(path_points)} path points")

    # Save points
    df = pd.DataFrame(path_points, columns=["x", "y", "z"])
    df.to_csv(OUTPUT_CSV, index=False)
    print(f"💾 Saved to {OUTPUT_CSV}")

    # Visualize result
    path_poly = pv.PolyData(path_points)
    plot = pv.Plotter()
    plot.add_mesh(mesh, color="lightgray", opacity=0.4)
    plot.add_mesh(picked_surface, color="orange", opacity=0.8)
    plot.add_points(path_poly, color="blue", point_size=6)
    plot.add_text("Surface Coverage Path Generated", font_size=12)
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
        self.setGeometry(200, 150, 420, 320)

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

        lines_label = QLabel("Number of Lines")
        points_label = QLabel("Points per Line")

        start_btn = QPushButton("🚀 Launch Viewer")
        start_btn.clicked.connect(self.launch_viewer)
        start_btn.setStyleSheet("background-color: #0078D7; color: white; font-weight: bold;")

        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addLayout(file_layout)
        layout.addWidget(lines_label)
        layout.addWidget(self.num_lines_slider)
        layout.addWidget(points_label)
        layout.addWidget(self.num_points_slider)
        layout.addWidget(start_btn)

        self.setLayout(layout)
        self.setStyleSheet("""
            QWidget { background-color: #f4f6f8; font-family: Segoe UI; }
            QLabel { font-size: 11pt; }
            QPushButton { border-radius: 8px; padding: 6px; }
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
        left_clicking=True,      # <--- enables single-click picking
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
