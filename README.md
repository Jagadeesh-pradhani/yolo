# YOLO ROS2 Package

A ROS2 package implementing YOLOv7-based object detection using Easy YOLOv7 implementation by Theos AI.

## Description

This package provides real-time object detection capabilities using YOLOv7 in a ROS2 environment. It publishes detection results that can be used by other ROS2 nodes in your robotics system.

## Installation

### Prerequisites
- ROS2 installed on your system
- Python 3.8 or newer
- C++ compiler and development tools

### Setup Instructions

1. Install system dependencies:
```bash
# Install required system packages
sudo apt-get update
sudo apt-get install -y build-essential python3-dev python3-numpy-dev
```

2. Create and activate a virtual environment:
```bash
# Create a virtual environment
python3 -m venv ~/yolo_venv

# Activate the virtual environment
source ~/yolo_venv/bin/activate
```

3. Install required dependencies:
```bash
# Update pip and install build tools
python -m pip install --upgrade pip setuptools wheel numpy

# Install Cython first (required for cython-bbox)
pip install Cython==0.29.32

# Lap folder direct paste
https://github.com/Adnedvid/Lap-pkg-for-YOLO

# Paste in yolo_venv/lib/python3.10/site-packages/


# Install cython-bbox separately
pip install -e git+https://github.com/samson-wang/cython_bbox.git#egg=cython-bbox

# Now install remaining requirements
pip install -r yolo/easy_yolov7/requirements.txt --no-deps

# imutils
pip install imutils


# Additional ROS2 dependencies
pip install --upgrade setuptools
```

4. Build the package:
```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws

# Build the workspace
colcon build --packages-select yolo

# Source the workspace
source install/setup.bash
```

## Usage

1. Make sure your virtual environment is activated:
```bash
source ~/yolo_venv/bin/activate
```

2. Run the detection node:
```bash
ros2 run yolo yolo_detection
```

## Topics

### Published Topics
- `/detections` - Object detection results

### Subscribed Topics
- `/camera/image_raw` - Raw image input for detection

## Troubleshooting

If you encounter compilation errors during installation:

1. For lap package installation issues:
```bash
# Make sure numpy development files are installed
sudo apt-get install python3-numpy-dev

# Try installing with explicit numpy include path
CFLAGS="-I$(python -c 'import numpy; print(numpy.get_include())')" pip install lap
```

2. If cython-bbox installation fails:
```bash
# Make sure Cython is installed first
pip install Cython==0.29.32

# Remove any failed installations
pip uninstall cython-bbox
# Then reinstall using git
pip install -e git+https://github.com/samson-wang/cython_bbox.git#egg=cython-bbox
```

3. For other package installation issues:
- Ensure all system dependencies are installed
- Try installing problematic packages separately with debug output:
```bash
pip install <package-name> -v
```
- If you see compilation errors related to numpy, make sure numpy is installed first:
```bash
pip install numpy
```

## License

This package uses the Easy YOLOv7 implementation by [Theos AI](https://theos.ai). For more information about the base implementation, see `yolo/easy_yolov7/README.md`.
