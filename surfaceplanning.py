import pyvista as pv
import numpy as np
import pandas as pd
from sklearn.decomposition import PCA
import panel as pn
from pathlib import Path

pn.extension('vtk')  # Enables 3D rendering support

# -------------------------------
# GLOBAL VARIABLES
# -------------------------------
mesh = None
selected_region = []
path_points = []
plotter = None
plot_pane = None

# -------------------------------
# PATH PLANNING FUNCTION
# -------------------------------
def generate_coverage_path(points_3d, num_lines=20, num_points_per_line=20):
    """
    Generate a 3D coverage path over a selected mesh surface.
    1. Project 3D surface region to 2D via PCA.
    2. Generate a zigzag (lawnmower) pattern.
    3. Back-project to 3D.
    """
    if len(points_3d) < 3:
        raise ValueError("Not enough points to generate a coverage path.")

    pca = PCA(n_components=2)
    plane_coords = pca.fit_transform(points_3d)

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
def pick_callback(picked_mesh, idx):
    global selected_region, path_points, mesh, plot_pane

    picked = picked_mesh.extract_cells(idx)
    selected_region = picked.points

    if len(selected_region) == 0:
        status_pane.object = "⚠️ No valid surface region selected."
        return

    status_pane.object = f"✅ Selected region with {len(selected_region)} vertices. Generating path..."

    try:
        path_points = generate_coverage_path(
            selected_region,
            num_lines_slider.value,
            num_points_slider.value,
        )
    except Exception as e:
        status_pane.object = f"❌ Error generating path: {e}"
        return

    df = pd.DataFrame(path_points, columns=["x", "y", "z"])
    df.to_csv("coverage_path.csv", index=False)

    status_pane.object = f"✅ Generated {len(path_points)} path points and saved to coverage_path.csv"

    # Visualize updated path
    path_poly = pv.PolyData(path_points)
    plotter.add_mesh(picked, color="red", opacity=0.7)
    plotter.add_points(path_poly, color="blue", point_size=8)
    plotter.add_text("Coverage Path Generated", font_size=12)
    plot_pane.object = plotter.ren_win

# -------------------------------
# LOAD MODEL AND SETUP SCENE
# -------------------------------
def load_model(event=None):
    global mesh, plotter, plot_pane

    if not file_input.value:
        status_pane.object = "⚠️ Please upload a .stl or .obj file first."
        return

    temp_path = Path("uploaded_model.stl")
    with open(temp_path, "wb") as f:
        f.write(file_input.value)

    try:
        mesh = pv.read(temp_path)
    except Exception as e:
        status_pane.object = f"❌ Error reading model: {e}"
        return

    plotter = pv.Plotter(notebook=False)
    plotter.add_mesh(mesh, color="lightgray")
    plotter.enable_cell_picking(
        callback=pick_callback,
        through=True,
        show_message=True,
        style="surface",
    )

    plot_pane.object = plotter.ren_win
    status_pane.object = "✅ Model loaded. Click on the surface to select a region."

# -------------------------------
# UI COMPONENTS
# -------------------------------
file_input = pn.widgets.FileInput(accept=".stl,.obj", name="Upload 3D Model")
load_button = pn.widgets.Button(name="Load & Visualize Model", button_type="primary")
load_button.on_click(load_model)

num_lines_slider = pn.widgets.IntSlider(
    name="Number of Coverage Lines", start=5, end=100, value=20, step=1
)

num_points_slider = pn.widgets.IntSlider(
    name="Points per Line", start=5, end=100, value=20, step=1
)

status_pane = pn.pane.Markdown("👋 Upload a 3D model to begin.", style={'color': 'black'})
plot_pane = pn.pane.VTK(height=600, sizing_mode="stretch_both")

# -------------------------------
# LAYOUT
# -------------------------------
dashboard = pn.Column(
    "## 🧭 3D Surface Coverage Path Planner",
    "Upload a 3D model, click on a surface region, and a coverage path will be generated automatically.",
    pn.Row(file_input, load_button),
    pn.Row(num_lines_slider, num_points_slider),
    status_pane,
    plot_pane,
)

if __name__ == "__main__":
    dashboard.servable()
