import os
os.environ["PYVISTA_USE_QT"] = "True"  # Ensure Qt backend for PyVista

import pyvista as pv
import numpy as np
from sklearn.decomposition import PCA
import pandas as pd
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QFileDialog, QSlider, QMessageBox
)
from PyQt5.QtCore import Qt
import sys


# -------------------------------
# GLOBALS
# -------------------------------
mesh = None
selected_region = []
path_points = []
MODEL_PATH = None
OUTPUT_CSV = "coverage_path.csv"
ui = None


# -------------------------------
# COVERAGE PATH GENERATION
# -------------------------------
def generate_coverage_path(points_3d, num_lines=20, num_points_per_line=20):
    if len(points_3d) < 3:
        raise ValueError("Not enough points to generate a coverage path.")

    # PCA projection to 2D
    pca = PCA(n_components=2)
    plane_coords = pca.fit_transform(points_3d)

    # Bounds
    x_min, x_max = plane_coords[:, 0].min(), plane_coords[:, 0].max()
    y_min, y_max = plane_coords[:, 1].min(), plane_coords[:, 1].max()

    x_lines = np.linspace(x_min, x_max, num_lines)
    path_2d = []

    for i, x in enumerate(x_lines):
        y_seq = np.linspace(y_min, y_max, num_points_per_line)
        if i % 2 == 1:
            y_seq = y_seq[::-1]
        path_2d.extend(list(zip([x] * len(y_seq), y_seq)))

    path_3d = pca.inverse_transform(path_2d)
    return np.array(path_3d)


# -------------------------------
# CALLBACK WHEN SURFACE IS PICKED
# -------------------------------
def pick_callback(picked_mesh):
    global selected_region, path_points, mesh, ui

    if picked_mesh is None or picked_mesh.n_points == 0:
        print("⚠️ No valid region selected.")
        return

    selected_region = picked_mesh.points
    print(f"\n✅ Selected region has {len(selected_region)} vertices")

    num_lines = ui.num_lines_slider.value()
    num_points = ui.num_points_slider.value()

    path_points = generate_coverage_path(selected_region, num_lines, num_points)
    print(f"✅ Generated {len(path_points)} path points")

    df = pd.DataFrame(path_points, columns=["x", "y", "z"])
    df.to_csv(OUTPUT_CSV, index=False)
    print(f"💾 Path points saved to {OUTPUT_CSV}")

    # Display coverage path in new window
    path_poly = pv.PolyData(path_points)
    sub_plot = pv.Plotter()
    sub_plot.add_mesh(mesh, color="lightgray", opacity=0.4)
    sub_plot.add_mesh(picked_mesh, color="red", opacity=0.7)
    sub_plot.add_points(path_poly, color="blue", point_size=8)
    sub_plot.add_text("Coverage Path Generated", font_size=12)
    sub_plot.show()


# -------------------------------
# MAIN APPLICATION UI
# -------------------------------
class SurfacePlannerUI(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("🧭 3D Surface Coverage Path Planner")
        self.setGeometry(200, 150, 400, 300)

        main_layout = QVBoxLayout()

        title = QLabel("<h2>3D Surface Coverage Planner</h2>")
        subtitle = QLabel("Select model, then click a region to generate coverage paths.")
        subtitle.setStyleSheet("color: gray; font-size: 11pt;")

        # File selection
        file_layout = QHBoxLayout()
        self.file_label = QLabel("No model selected")
        browse_btn = QPushButton("📂 Browse Model")
        browse_btn.clicked.connect(self.load_model)
        file_layout.addWidget(self.file_label)
        file_layout.addWidget(browse_btn)

        # Sliders
        self.num_lines_slider = QSlider(Qt.Horizontal)
        self.num_lines_slider.setMinimum(5)
        self.num_lines_slider.setMaximum(100)
        self.num_lines_slider.setValue(20)

        self.num_points_slider = QSlider(Qt.Horizontal)
        self.num_points_slider.setMinimum(5)
        self.num_points_slider.setMaximum(100)
        self.num_points_slider.setValue(20)

        lines_label = QLabel("Number of Lines")
        points_label = QLabel("Points per Line")

        # Launch viewer
        start_btn = QPushButton("🚀 Launch Viewer")
        start_btn.clicked.connect(self.launch_viewer)
        start_btn.setStyleSheet("background-color: #0078D7; color: white; font-weight: bold;")

        # Layout arrangement
        main_layout.addWidget(title)
        main_layout.addWidget(subtitle)
        main_layout.addLayout(file_layout)
        main_layout.addWidget(lines_label)
        main_layout.addWidget(self.num_lines_slider)
        main_layout.addWidget(points_label)
        main_layout.addWidget(self.num_points_slider)
        main_layout.addWidget(start_btn)

        self.setLayout(main_layout)
        self.setStyleSheet("""
            QWidget {
                font-family: Segoe UI;
                background-color: #f8f9fa;
            }
            QLabel {
                font-size: 12pt;
            }
            QPushButton {
                border-radius: 8px;
                padding: 6px;
            }
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
        plotter.add_text("Click a region to generate coverage path", font_size=10)
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
