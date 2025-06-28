# YOLO ROS2 Package

A ROS2 package implementing YOLOv7-based object detection using Easy YOLOv7 implementation by Theos AI.

## Description

This package provides real-time object detection capabilities using YOLOv7 in a ROS2 environment. It publishes detection results that can be used by other ROS2 nodes in your robotics system.

## Installation

### Prerequisites
- ROS2 installed on your system
- Python 3.8 or newer

### Setup Instructions

1. Create and activate a virtual environment:
```bash
# Create a virtual environment
python3 -m venv ~/yolo_venv

# Activate the virtual environment
source ~/yolo_venv/bin/activate
```

2. Install required dependencies:
```bash
# Install requirements from easy_yolov7
pip install -r yolo/easy_yolov7/requirements.txt

# If you encounter issues with cython-bbox, install it using:
pip install -e git+https://github.com/samson-wang/cython_bbox.git#egg=cython-bbox

# Additional ROS2 dependencies
pip install --upgrade setuptools
```

3. Build the package:
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

## License

This package uses the Easy YOLOv7 implementation by [Theos AI](https://theos.ai). For more information about the base implementation, see `yolo/easy_yolov7/README.md`.
