# Surface Coverage Path Planner

A 3D surface coverage path planning tool for generating robot paths on selected surfaces from STL/OBJ files. Includes ROS integration for robotic applications.

## Features

### Core Functionality
- **3D Model Loading**: Load STL and OBJ files
- **Interactive Surface Selection**: Click on any surface region to generate a coverage path
- **Smart Path Generation**: Automatically generates optimized coverage paths on curved 3D surfaces
- **Collision Detection**: Ensures generated paths don't penetrate the mesh body
- **Path Smoothing**: Optional spline-based path smoothing for smoother trajectories

### ROS Integration
- **Automatic ROS Publishing**: Publishes path coordinates to ROS topic `/surface_coverage_path`
- **PoseArray Messages**: Compatible with ROS navigation stack
- **Configurable Frame ID**: Default frame is "map" (customizable in code)

### Path Planning Algorithm

The planner uses an advanced ray-tracing approach:

1. **Surface Projection**: Creates a 2D grid over the surface bounding box
2. **Ray Tracing**: Casts vertical rays from above to find exact surface intersections
3. **Boustrophedon Pattern**: Alternating sweep directions for efficient coverage
4. **Collision Checking**: Validates that path segments don't pass through the mesh body
5. **Path Smoothing**: Optional spline interpolation for smoother motion

## Installation

### Requirements

```bash
# Core dependencies
pip install pyvista numpy pandas scipy PyQt5

# Optional: ROS support (requires ROS environment)
# Make sure your ROS environment is sourced:
# source /opt/ros/noetic/setup.bash  # or your ROS version
pip install rospy geometry_msgs std_msgs
```

## Usage

### Running the Application

```bash
python surface_coverage_planner-v4.py
```

### Workflow

1. **Launch Application**: Run the Python script
2. **Load Model**: Click "Browse Model" and select your STL/OBJ file
3. **Configure Parameters**:
   - Adjust "Number of Lines" (coverage density)
   - Adjust "Points per Line" (path resolution)
   - Enable/disable path smoothing
   - Enable/disable collision detection
   - Enable ROS publishing if available
4. **Launch Viewer**: Click "Launch Viewer" to open the 3D viewer
5. **Select Surface**: Click on any surface region in the 3D model
6. **View Results**: The path will be displayed on the selected surface
7. **Export**: Path is automatically saved to `coverage_path.csv`

### Output Files

- **coverage_path.csv**: CSV file with X, Y, Z coordinates of path points
  ```
  x,y,z
  10.5,20.3,15.2
  10.5,21.0,15.3
  ...
  ```

### ROS Integration

If ROS is available and enabled, paths are published to:
- **Topic**: `/surface_coverage_path`
- **Message Type**: `geometry_msgs/PoseArray`
- **Frame ID**: `map` (configurable)

To subscribe in ROS:
```bash
# View published paths
rostopic echo /surface_coverage_path

# Visualize in RViz
rosrun rviz rviz
# Add PoseArray display and set topic to /surface_coverage_path
```

## Key Improvements in v4

### Fixed Issues

1. **Path Projection Bug** (Line 50-51):
   - **Old**: Incorrectly used `find_closest_point()` which returns an index
   - **New**: Uses `ray_trace()` for accurate surface projection

2. **No Collision Detection**:
   - **Old**: Paths could penetrate the mesh body
   - **New**: `check_line_collision()` validates each path segment

3. **Missing ROS Integration**:
   - **Old**: Only CSV export
   - **New**: Direct ROS publishing with PoseArray messages

4. **Poor Surface Following**:
   - **Old**: Simple 2D projection on fixed Z plane
   - **New**: Ray-tracing from above with exact surface intersections

### New Features

- **Path Smoothing**: Spline-based smoothing for smoother robot motion
- **Collision Detection**: Ensures paths stay on surface and don't penetrate body
- **Visual Path Lines**: Red lines connecting points show coverage path clearly
- **Real-time Parameter Updates**: Slider labels update dynamically
- **Better Visualization**: Improved rendering with spheres and lines

## Technical Details

### Path Generation Algorithm

```python
def generate_surface_coverage_points(surface, mesh_full, num_lines, num_points_per_line,
                                     smooth=True, collision_check=True):
    """
    1. Create 2D grid over surface bounding box
    2. For each grid point:
       a. Cast ray from above the surface
       b. Find intersection with surface using ray_trace()
       c. Add intersection point to path
    3. Check consecutive points for collisions with mesh
    4. Apply spline smoothing if enabled
    5. Return final path array
    """
```

### Collision Detection

Uses PyVista's `select_enclosed_points()` to check if any point along a path segment is inside the mesh body. Segments with collisions are removed from the final path.

### ROS Message Format

```python
PoseArray:
  header:
    stamp: current_time
    frame_id: "map"
  poses: [
    Pose(position: Point(x, y, z), orientation: Quaternion(0, 0, 0, 1)),
    ...
  ]
```

## Troubleshooting

### ROS Not Available
If you see "ROS not available":
```bash
# Install ROS packages
pip install rospy geometry_msgs std_msgs

# Or source your ROS environment
source /opt/ros/noetic/setup.bash
```

### Path Doesn't Follow Surface
- Increase "Points per Line" for better surface following
- Enable "Apply Path Smoothing" for better curvature handling
- Check that surface selection is accurate

### Collision Detection Too Aggressive
- Disable collision detection temporarily
- Adjust `num_samples` parameter in `check_line_collision()` (default: 20)
- Check mesh for internal geometry or self-intersections

### Visualization Issues
- Ensure PyVista is using Qt backend (set at top of script)
- Try updating PyVista: `pip install --upgrade pyvista`
- Check that your system has Qt5 installed

## Configuration

### Advanced Parameters

Edit these in the code for fine-tuning:

```python
# Path generation (line 91)
def generate_surface_coverage_points(surface, mesh_full,
                                     num_lines=20,          # Coverage density
                                     num_points_per_line=20, # Path resolution
                                     smooth=True,            # Enable smoothing
                                     collision_check=True):  # Enable collision check

# Collision detection (line 43)
def check_line_collision(p1, p2, mesh, num_samples=20):  # Samples per segment

# Path smoothing (line 67)
def smooth_path(points, smoothness=0.5):  # Smoothness factor

# ROS publishing (line 180)
def publish_path_to_ros(path_points, frame_id="map"):  # Frame ID
```

## Examples

### Simple Surface Coverage
```python
# Load model
mesh = pv.read("model.stl")

# Click on top surface
# Result: Coverage path with 20x20 grid = 400 points
```

### High-Resolution Path
- Set "Number of Lines": 50
- Set "Points per Line": 50
- Result: 2500 path points for detailed coverage

### ROS-Only Workflow
- Enable "Publish to ROS"
- Disable CSV export in code if needed
- Subscribe to `/surface_coverage_path` in your ROS node

## License

MIT License - Feel free to use and modify for your robotics projects.

## Contributing

Contributions welcome! Areas for improvement:
- Adaptive path density based on surface curvature
- Multi-surface selection and path merging
- Tool orientation calculation based on surface normals
- Path optimization for minimum travel time
- Integration with MoveIt! for motion planning

## Support

For issues or questions, please check:
1. Ensure all dependencies are installed
2. Check that STL/OBJ file is valid
3. Verify ROS environment is sourced (if using ROS)
4. Check console output for detailed error messages
