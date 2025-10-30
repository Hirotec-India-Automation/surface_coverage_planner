#!/usr/bin/env python3
import pyvista as pv
import numpy as np
from sklearn.decomposition import PCA
import pandas as pd
import rospy
from geometry_msgs.msg import PointStamped

# -------------------------------
# GLOBAL VARIABLES
# -------------------------------
selected_region = []
path_points = []
model_path = "1cad_model.stl"  # <-- Change this path to your model file


# -------------------------------
# ROS PUBLISHER INITIALIZATION
# -------------------------------
def init_ros():
    rospy.init_node("surface_coverage_planner", anonymous=True)
    pub = rospy.Publisher("/coverage_path_points", PointStamped, queue_size=10)
    rospy.loginfo("ROS node initialized and publisher active on /coverage_path_points")
    return pub


# -------------------------------
# PATH PLANNING FUNCTION
# -------------------------------
def generate_coverage_path(points_3d, num_lines=20, num_points=20):
    """
    Projects 3D surface region to 2D, generates a zigzag (lawnmower) path,
    and maps it back to 3D space.
    """
    pca = PCA(n_components=2)
    plane_coords = pca.fit_transform(points_3d)

    # Compute bounds of projected surface
    x_min, x_max = plane_coords[:, 0].min(), plane_coords[:, 0].max()
    y_min, y_max = plane_coords[:, 1].min(), plane_coords[:, 1].max()

    x_lines = np.linspace(x_min, x_max, num_lines)
    path = []

    for i, x in enumerate(x_lines):
        y_seq = np.linspace(y_min, y_max, num_points)
        if i % 2 == 1:
            y_seq = y_seq[::-1]
        path += list(zip([x] * len(y_seq), y_seq))

    # Back-project to 3D
    path_3d = pca.inverse_transform(path)
    return np.array(path_3d)


# -------------------------------
# CALLBACK WHEN SURFACE IS PICKED
# -------------------------------
def pick_callback(mesh, idx):
    global selected_region, path_points, pub

    picked = mesh.extract_cells(idx)
    selected_region = picked.points
    print(f"\n✅ Selected region has {len(selected_region)} vertices")

    # Generate coverage path
    path_points = generate_coverage_path(selected_region)
    print(f"✅ Generated {len(path_points)} path points")

    # Save to CSV
    df = pd.DataFrame(path_points, columns=["x", "y", "z"])
    df.to_csv("coverage_path.csv", index=False)
    print("💾 Path points saved to coverage_path.csv")

    # Publish to ROS
    for pt in path_points:
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"  # Change if your frame is different
        msg.point.x, msg.point.y, msg.point.z = pt
        pub.publish(msg)
        rospy.sleep(0.05)  # 20Hz publishing rate

    print("📡 Path points published to ROS topic /coverage_path_points")

    # Visualize selected region + path
    path_poly = pv.PolyData(path_points)
    plotter = pv.Plotter()
    plotter.add_mesh(mesh, color="gray", opacity=0.5)
    plotter.add_mesh(picked, color="red", opacity=0.8)
    plotter.add_points(path_poly, color="blue", point_size=8)
    plotter.show()


# -------------------------------
# MAIN FUNCTION
# -------------------------------
if __name__ == "__main__":
    try:
        # Initialize ROS
        pub = init_ros()

        # Load 3D model
        mesh = pv.read(model_path)

        # Start interactive picking
        print("\n🖱 Click on the surface to select a region.")
        print("Once selected, the coverage path will be generated automatically.\n")

        plotter = pv.Plotter()
        plotter.add_mesh(mesh, color="lightgray")
        plotter.enable_cell_picking(
            callback=pick_callback,
            through=True,
            show_message=True,
            style="surface",
        )
        plotter.show()

    except rospy.ROSInterruptException:
        pass
