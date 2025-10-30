# 3D Surface Coverage Path Planner - Windows Edition

![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)
![Platform](https://img.shields.io/badge/platform-Windows%2010%2F11-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

## Overview

A powerful 3D surface coverage path planning tool optimized for Windows operating systems. Generate optimal coverage paths on complex 3D surfaces with an intuitive GUI.

## Key Features

- 🖥️ **Windows Optimized**: Native Windows path handling and styling
- 🎯 **Interactive Selection**: Click-to-select surface regions
- 📊 **Real-time Progress**: Multi-threaded generation with progress tracking
- 🎨 **3D Visualization**: Interactive PyVista-based 3D rendering
- 💾 **CSV Export**: Automatic path export to CSV format
- ⚙️ **Configurable**: Adjustable path density and options
- 🔍 **Collision Detection**: Ensures paths don't penetrate mesh
- 🌊 **Path Smoothing**: Spline-based path smoothing
- 📝 **Comprehensive Logging**: Real-time operation feedback

## Quick Start (5 Minutes)

### 1. Install Python

Download and install Python 3.8+ from [python.org](https://www.python.org/downloads/)

**⚠️ Important**: Check "Add Python to PATH" during installation

### 2. Install Dependencies

```cmd
cd path\to\surface_coverage_planner
pip install -r requirements_windows.txt
```

### 3. Run Application

```cmd
python surface_coverage_planner_windows.py
```

### 4. Use Application

1. Click "📂 Browse Model" → Select your STL/OBJ file
2. Adjust sliders for desired path density
3. Click "🚀 Launch 3D Viewer"
4. Click on any surface in the 3D view
5. Wait for path generation (progress bar shows status)
6. View results in new 3D window
7. Find CSV output in `Documents\SurfaceCoveragePlanner\`

## System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| OS | Windows 10 (64-bit) | Windows 11 |
| Python | 3.8 | 3.9 or 3.10 |
| RAM | 4 GB | 8 GB |
| Graphics | OpenGL 3.2+ | Dedicated GPU |
| Disk Space | 500 MB | 1 GB |

## Installation Details

### Option 1: Quick Install (Recommended)

```cmd
# Clone or download repository
cd surface_coverage_planner

# Create virtual environment
python -m venv venv
venv\Scripts\activate.bat

# Install dependencies
pip install -r requirements_windows.txt

# Run
python surface_coverage_planner_windows.py
```

### Option 2: Manual Install

```cmd
pip install pyvista vtk numpy scipy pandas scikit-learn PyQt5 matplotlib
python surface_coverage_planner_windows.py
```

## What's New in Windows Version

### Improvements Over Linux Version

| Feature | Linux Version | Windows Version |
|---------|---------------|-----------------|
| Path Handling | String-based | `pathlib.Path` (cross-platform) |
| Output Location | Current directory | Documents folder (Windows standard) |
| GUI Responsiveness | Blocking operations | Multi-threaded with progress bar |
| Error Messages | Basic | Comprehensive Windows-specific |
| Styling | Default Qt | Fusion style (native Windows look) |
| Logging | Console only | GUI log window + console |

### All Original Features Preserved

✅ Surface-following path generation
✅ Zigzag (lawnmower) pattern
✅ Collision detection
✅ Path smoothing with splines
✅ CSV export
✅ 3D visualization
✅ Configurable parameters
✅ Optional ROS integration

## File Structure

```
surface_coverage_planner/
│
├── surface_coverage_planner_windows.py  # Main Windows application
├── requirements_windows.txt              # Dependencies for Windows
├── WINDOWS_INSTALLATION_GUIDE.md        # Detailed installation guide
├── README_WINDOWS.md                    # This file
│
├── surface_coverage_planner-v4.py       # Original Linux version
├── pathgenerator.py                     # Legacy path generator
└── surfaceplanning.py                   # Legacy surface planner
```

## Usage Example

```python
# The application provides a GUI, but you can also use it programmatically:

import pyvista as pv
from surface_coverage_planner_windows import generate_surface_coverage_points

# Load mesh
mesh = pv.read("model.stl")

# Generate path
path_points = generate_surface_coverage_points(
    surface=mesh,
    mesh_full=mesh,
    num_lines=20,
    num_points_per_line=20,
    smooth=True,
    collision_check=True
)

# Save to CSV
import pandas as pd
df = pd.DataFrame(path_points, columns=["x", "y", "z"])
df.to_csv("output_path.csv", index=False)
```

## Output Format

CSV file with X, Y, Z coordinates:

```csv
x,y,z
10.5,20.3,5.2
10.6,20.4,5.3
10.7,20.5,5.4
...
```

Default output location:
```
C:\Users\<YourUsername>\Documents\SurfaceCoveragePlanner\coverage_path.csv
```

## Configuration

### Adjustable Parameters

- **Number of Lines**: 5-100 (controls coverage density)
- **Points per Line**: 5-100 (controls point density)
- **Path Smoothing**: On/Off (spline interpolation)
- **Collision Detection**: On/Off (prevents mesh penetration)
- **ROS Publishing**: On/Off (optional, requires ROS)

### Modifying Output Directory

Edit in code (line ~36):
```python
OUTPUT_DIR = Path.home() / "Documents" / "SurfaceCoveragePlanner"
```

## Supported 3D Formats

- ✅ STL (Standard Tessellation Language) - `.stl`
- ✅ OBJ (Wavefront Object) - `.obj`
- ✅ Other VTK-supported formats

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| "Python is not recognized" | Add Python to PATH in Environment Variables |
| "No module named 'pyvista'" | Run `pip install -r requirements_windows.txt` |
| "Failed to launch 3D viewer" | Update graphics drivers |
| "DLL load failed" | Install Visual C++ Redistributables |
| Black screen in viewer | Set `PYVISTA_OFF_SCREEN=false` |

See [WINDOWS_INSTALLATION_GUIDE.md](WINDOWS_INSTALLATION_GUIDE.md) for detailed solutions.

## Performance Tips

1. ⚡ Use SSD for faster model loading
2. 🚀 Start with 10-20 lines for testing
3. 🎯 Close background applications for large models
4. 🔧 Update graphics drivers regularly
5. 📊 Monitor progress bar during generation

## ROS Integration (Optional)

While ROS is not commonly used on Windows, the application supports it:

### Windows Subsystem for Linux (WSL)
```bash
# In WSL Ubuntu
sudo apt install ros-noetic-desktop-full
# Run Windows version, will detect ROS if available
```

### ROS2 Native Windows
```cmd
# Download ROS2 from ros.org
pip install rospy geometry-msgs std-msgs
```

Published topic: `/surface_coverage_path` (PoseArray)

## Building Standalone Executable

```cmd
pip install pyinstaller

pyinstaller --onefile --windowed ^
    --name "SurfaceCoveragePlanner" ^
    surface_coverage_planner_windows.py
```

Output: `dist\SurfaceCoveragePlanner.exe`

## Screenshots

### Main Application Window
- Clean Windows-styled interface
- Intuitive parameter controls
- Real-time logging

### 3D Viewer
- Interactive surface selection
- Smooth camera controls
- Visual feedback on selection

### Generated Path Visualization
- Color-coded elements (model, surface, path)
- 3D path lines showing coverage
- Legend for clarity

## Comparison with Other Versions

| File | Platform | GUI | Features |
|------|----------|-----|----------|
| `surface_coverage_planner_windows.py` | Windows | PyQt5 + Progress | All features + Windows optimizations |
| `surface_coverage_planner-v4.py` | Linux | PyQt5 | All features |
| `surface_coverage_planner-v3.py` | Linux | Panel | Web-based interface |
| `pathgenerator.py` | Linux | None | Basic path generation |

## Dependencies

```
pyvista>=0.38.0       # 3D visualization
vtk>=9.1.0            # VTK backend
numpy>=1.21.0         # Numerical operations
scipy>=1.7.0          # Scientific computing
pandas>=1.3.0         # Data handling
scikit-learn>=1.0.0   # PCA for path planning
PyQt5>=5.15.0         # GUI framework
matplotlib>=3.5.0     # Optional plotting
```

## Technical Details

### Path Generation Algorithm

1. **Surface Selection**: User clicks on mesh surface
2. **Bounding Box**: Calculate surface bounds
3. **Grid Generation**: Create 2D grid over bounds
4. **Ray Tracing**: Project grid points onto surface
5. **Zigzag Pattern**: Alternate line directions
6. **Collision Check**: Verify no mesh penetration
7. **Path Smoothing**: Apply spline interpolation
8. **Export**: Save to CSV format

### Threading Model

- **Main Thread**: GUI and user interaction
- **Worker Thread**: Path generation (prevents GUI freezing)
- **Signals**: Progress updates to main thread

## License

MIT License

Copyright (c) 2025 Surface Coverage Planner Team

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

## Support

For issues and questions:

1. Check [WINDOWS_INSTALLATION_GUIDE.md](WINDOWS_INSTALLATION_GUIDE.md)
2. Review error messages in application log window
3. Verify all dependencies are installed correctly
4. Test with a simple model first

## Future Enhancements

- [ ] Multi-surface batch processing
- [ ] Custom path patterns (spiral, outline, etc.)
- [ ] Path optimization algorithms
- [ ] 3D path preview before generation
- [ ] Configurable output formats (JSON, XML)
- [ ] Undo/redo functionality
- [ ] Save/load project settings

## Version History

### v1.0 (Current)
- Initial Windows-compatible release
- All features from Linux v4
- Multi-threaded path generation
- Enhanced error handling
- Windows-specific optimizations

## Acknowledgments

Built with:
- [PyVista](https://docs.pyvista.org/) - 3D visualization
- [VTK](https://vtk.org/) - Visualization Toolkit
- [PyQt5](https://www.riverbankcomputing.com/software/pyqt/) - GUI framework
- [NumPy](https://numpy.org/) - Numerical computing
- [SciPy](https://scipy.org/) - Scientific computing

---

**Ready to generate coverage paths on Windows!** 🚀

For detailed installation instructions, see [WINDOWS_INSTALLATION_GUIDE.md](WINDOWS_INSTALLATION_GUIDE.md)
