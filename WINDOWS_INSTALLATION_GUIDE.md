# Surface Coverage Planner - Windows Installation Guide

## Overview

This guide will help you install and run the Surface Coverage Path Planner on Windows operating systems. The Windows-compatible version includes all features from the original code with enhanced Windows support.

## System Requirements

- **Operating System**: Windows 10 or Windows 11 (64-bit)
- **Python**: Version 3.8 or higher (3.9 or 3.10 recommended)
- **RAM**: Minimum 4 GB (8 GB recommended for large models)
- **Graphics**: OpenGL 3.2+ compatible graphics card
- **Disk Space**: At least 500 MB free space

## Features

The Windows-compatible version includes:

✅ Cross-platform path handling using `pathlib`
✅ Windows-friendly GUI with Fusion styling
✅ Multi-threaded path generation for responsiveness
✅ Comprehensive error handling and logging
✅ Automatic output directory creation in `Documents` folder
✅ Progress bar for long operations
✅ 3D visualization with PyVista
✅ CSV export of generated paths
✅ Optional ROS integration (if needed)
✅ Surface selection with collision detection
✅ Path smoothing with spline interpolation

## Installation Steps

### Step 1: Install Python

1. Download Python from [python.org](https://www.python.org/downloads/)
2. During installation, **make sure to check "Add Python to PATH"**
3. Verify installation by opening Command Prompt and typing:
   ```cmd
   python --version
   ```

### Step 2: Create a Virtual Environment (Recommended)

Open Command Prompt or PowerShell and navigate to your project directory:

```cmd
cd C:\path\to\surface_coverage_planner
python -m venv venv
```

Activate the virtual environment:

```cmd
# Command Prompt
venv\Scripts\activate.bat

# PowerShell
venv\Scripts\Activate.ps1
```

**Note**: If you get an error in PowerShell about execution policies, run:
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### Step 3: Install Required Packages

Install all dependencies using the requirements file:

```cmd
pip install -r requirements_windows.txt
```

Or install packages individually:

```cmd
pip install pyvista vtk numpy scipy pandas scikit-learn PyQt5 matplotlib
```

### Step 4: Verify Installation

Test that PyVista works correctly:

```cmd
python -c "import pyvista as pv; print('PyVista version:', pv.__version__)"
```

## Running the Application

### Method 1: Using Python Command

```cmd
python surface_coverage_planner_windows.py
```

### Method 2: Double-Click (Windows Explorer)

1. Right-click on `surface_coverage_planner_windows.py`
2. Select "Open with" → "Python"

### Method 3: Create a Windows Shortcut

1. Right-click on desktop → "New" → "Shortcut"
2. Enter the target:
   ```
   "C:\path\to\python.exe" "C:\path\to\surface_coverage_planner_windows.py"
   ```
3. Name it "Surface Coverage Planner"

## Usage Guide

### Basic Workflow

1. **Launch Application**: Run the Python script
2. **Load Model**: Click "📂 Browse Model" and select your STL or OBJ file
3. **Configure Parameters**:
   - Adjust "Number of Lines" slider (5-100)
   - Adjust "Points per Line" slider (5-100)
   - Enable/disable options as needed
4. **Launch Viewer**: Click "🚀 Launch 3D Viewer"
5. **Select Surface**: Click on any surface in the 3D viewer
6. **Wait for Generation**: Progress bar shows generation status
7. **View Results**: Automatically displayed in new 3D viewer
8. **Check Output**: Find CSV file in `Documents\SurfaceCoveragePlanner\`

### Output Location

Generated paths are saved to:
```
C:\Users\YourUsername\Documents\SurfaceCoveragePlanner\coverage_path.csv
```

### CSV Format

The output CSV contains three columns:
```
x,y,z
10.5,20.3,5.2
10.6,20.4,5.3
...
```

## Configuration Options

### Path Parameters

- **Number of Lines**: Controls coverage density (more lines = denser coverage)
- **Points per Line**: Controls point density along each line

### Options

- **Apply Path Smoothing**: Uses spline interpolation for smoother paths
- **Enable Collision Detection**: Ensures paths don't pass through mesh interior
- **Publish to ROS**: Enable if using ROS (requires ROS installation)

## Troubleshooting

### Issue: "Python is not recognized"

**Solution**: Add Python to PATH:
1. Search for "Environment Variables" in Windows
2. Edit "Path" variable
3. Add Python installation directory (e.g., `C:\Python39\`)

### Issue: "No module named 'pyvista'"

**Solution**: Install dependencies:
```cmd
pip install -r requirements_windows.txt
```

### Issue: "Failed to launch 3D viewer"

**Possible causes and solutions**:

1. **Outdated graphics drivers**:
   - Update your graphics card drivers from manufacturer website

2. **OpenGL support**:
   - Run: `python -c "import pyvista as pv; print(pv.system_supports_plotting())"`
   - If False, update graphics drivers

3. **PyQt5 conflicts**:
   - Reinstall PyQt5: `pip uninstall PyQt5 && pip install PyQt5`

### Issue: "DLL load failed" errors

**Solution**: Install Visual C++ Redistributables:
- Download from: https://aka.ms/vs/17/release/vc_redist.x64.exe
- Install and restart computer

### Issue: Black screen in 3D viewer

**Solution**:
1. Set environment variable before running:
   ```cmd
   set PYVISTA_OFF_SCREEN=false
   python surface_coverage_planner_windows.py
   ```

2. Or add to beginning of script:
   ```python
   os.environ["PYVISTA_OFF_SCREEN"] = "false"
   ```

### Issue: "Permission denied" when saving files

**Solution**:
- Run Python as Administrator
- Or change output directory in code to a location with write permissions

## Performance Tips for Windows

1. **Use SSD**: Store models on SSD for faster loading
2. **Close Background Apps**: Free up RAM for large models
3. **Update Drivers**: Keep graphics drivers up to date
4. **Adjust Parameters**: Start with lower values (10-20 lines) for testing
5. **Use Progress Bar**: Monitor generation progress

## Advanced Configuration

### Changing Output Directory

Edit the file and modify this line:
```python
OUTPUT_DIR = Path.home() / "Documents" / "SurfaceCoveragePlanner"
```

Example custom locations:
```python
# Desktop
OUTPUT_DIR = Path.home() / "Desktop" / "paths"

# Custom drive
OUTPUT_DIR = Path("D:/Projects/PathPlanning/output")
```

### Adjusting Smoothness

Modify in the code (line ~171):
```python
path_array = smooth_path(path_array, smoothness=len(path_array) * 0.1)
```

Increase multiplier for smoother paths (e.g., `0.2` or `0.3`)

### Collision Detection Sensitivity

Modify in `check_line_collision()` function (line ~44):
```python
def check_line_collision(p1, p2, mesh, num_samples=20):
```

Increase `num_samples` for more thorough checking (slower but more accurate)

## Differences from Linux Version

### What's Changed for Windows:

1. **Path Handling**: Uses `pathlib.Path` instead of string paths
2. **Output Location**: Defaults to `Documents` folder (Windows standard)
3. **GUI Styling**: Uses Fusion style for native Windows look
4. **Multi-threading**: Added `QThread` for responsive GUI
5. **Error Handling**: More comprehensive Windows-specific error messages
6. **Progress Feedback**: Added progress bar for long operations
7. **ROS Optional**: ROS integration is optional (not commonly used on Windows)

### What's Identical:

- All path generation algorithms
- Collision detection logic
- Surface selection mechanism
- Smoothing algorithms
- CSV export format
- 3D visualization capabilities

## Using with ROS on Windows

If you want to use ROS features on Windows:

### Option 1: WSL (Windows Subsystem for Linux)

1. Install WSL2 with Ubuntu
2. Install ROS in WSL
3. Run the Windows version, it will detect ROS if available

### Option 2: ROS2 on Windows (Native)

1. Install ROS2 from [ros.org](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)
2. Install ROS Python packages:
   ```cmd
   pip install rospy geometry-msgs std-msgs
   ```

## Example Workflow

```cmd
# 1. Activate virtual environment
cd C:\Projects\surface_coverage_planner
venv\Scripts\activate.bat

# 2. Run application
python surface_coverage_planner_windows.py

# 3. In the GUI:
#    - Click "Browse Model"
#    - Select your_model.stl
#    - Adjust sliders to desired values
#    - Click "Launch 3D Viewer"
#    - Click on a surface in the 3D view
#    - Wait for path generation
#    - View generated path visualization

# 4. Find output at:
#    C:\Users\YourName\Documents\SurfaceCoveragePlanner\coverage_path.csv
```

## Supported File Formats

- **STL** (Standard Tessellation Language): `.stl`
- **OBJ** (Wavefront Object): `.obj`
- Other formats supported by VTK/PyVista

## Getting Help

If you encounter issues:

1. Check the log window in the application
2. Look for error messages in console/terminal
3. Verify all dependencies are installed
4. Update graphics drivers
5. Try with a simpler/smaller 3D model first

## Building an Executable (Optional)

To create a standalone `.exe` file:

```cmd
pip install pyinstaller

pyinstaller --onefile --windowed ^
    --name "SurfaceCoveragePlanner" ^
    --icon=icon.ico ^
    surface_coverage_planner_windows.py
```

The executable will be in the `dist` folder.

## License

MIT License - Free to use and modify

## Version Information

- **Version**: Windows v1.0
- **Last Updated**: 2025
- **Python**: 3.8+
- **Platform**: Windows 10/11

---

## Quick Reference Commands

```cmd
# Install dependencies
pip install -r requirements_windows.txt

# Run application
python surface_coverage_planner_windows.py

# Check Python version
python --version

# Check installed packages
pip list

# Update a package
pip install --upgrade pyvista

# Deactivate virtual environment
deactivate
```

---

**Enjoy using the Surface Coverage Path Planner on Windows!** 🚀
