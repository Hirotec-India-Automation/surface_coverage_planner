import os
os.environ["PYVISTA_USE_QT"] = "True"  # Ensure interactive window backend

import pyvista as pv
import numpy as np
from sklearn.decomposition import PCA
import pandas as pd

# -------------------------------
# CONFIGURATION
# -------------------------------
MODEL_PATH = "1cad_model.stl"  # Change to your 3D model path
OUTPUT_CSV = "coverage_path.csv"

# -------------------------------
# GLOBAL VARIABLES
# -------------------------------
selected_region = []
path_points = []
mesh = None  # make mesh global for callback


# -------------------------------
# PATH PLANNING FUNCTION
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
    global selected_region, path_points, mesh

    if picked_mesh is None or picked_mesh.n_points == 0:
        print("⚠️ No valid region selected.")
        return

    selected_region = picked_mesh.points
    print(f"\n✅ Selected region has {len(selected_region)} vertices")

    # Generate coverage path
    path_points = generate_coverage_path(selected_region)
    print(f"✅ Generated {len(path_points)} path points")

    # Save to CSV
    df = pd.DataFrame(path_points, columns=["x", "y", "z"])
    df.to_csv(OUTPUT_CSV, index=False)
    print(f"💾 Path points saved to {OUTPUT_CSV}")

    # Visualize coverage path
    path_poly = pv.PolyData(path_points)

    sub_plot = pv.Plotter()
    sub_plot.add_mesh(mesh, color="gray", opacity=0.4)
    sub_plot.add_mesh(picked_mesh, color="red", opacity=0.7)
    sub_plot.add_points(path_poly, color="blue", point_size=8)
    sub_plot.add_text("Coverage Path Generated", font_size=12)
    sub_plot.show()


# -------------------------------
# MAIN FUNCTION
# -------------------------------
def main():
    global mesh

    print("\n🧭 3D Surface Coverage Path Planner")
    print("----------------------------------")
    print("🖱  Click on the surface to select a region.")
    print("Once selected, the coverage path will be generated automatically.\n")

    # Load 3D model
    mesh = pv.read(MODEL_PATH)

    # Interactive viewer
    plotter = pv.Plotter()
    plotter.add_mesh(mesh, color="lightgray")
    plotter.enable_cell_picking(
        callback=pick_callback,
        through=True,
        show_message=True,
        style="wireframe",
    )
    plotter.show()


# -------------------------------
# ENTRY POINT
# -------------------------------
if __name__ == "__main__":
    main()
