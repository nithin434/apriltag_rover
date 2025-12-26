# apriltag_rover

Prototype scripts and assets for AprilTag-based autonomous docking mission demos. Includes visualization helpers and recording utilities.

## Overview

This project implements an autonomous rover docking system using AprilTag fiducial markers for precise localization and navigation. The system includes mission planning, real-time tracking, and simulated visualization capabilities.

## Project Structure

```
├── src/apriltag_docking/          # ROS2 package source
│   ├── apriltag_docking/          # Python modules
│   ├── launch/                    # ROS2 launch files
│   ├── config/                    # Configuration files
│   └── worlds/                    # Simulation worlds
├── apriltag_follower.py           # AprilTag tracking module
├── mission_controller.py           # Mission state machine
├── camera_recorder.py              # Video recording utility
├── create_demo_video.py            # Demo visualization generator
└── rosbag_to_video.py             # Data conversion utility
```

## Features

- **Autonomous Docking**: Multi-stage mission controller for dock approach and engagement
- **AprilTag Detection**: Real-time marker detection and pose estimation
- **Mission Planning**: Configurable state machine for sequential operations
- **Visualization**: Animated mission replay with matplotlib
- **Recording**: ROS2 bag recording and video conversion utilities

## Installation

```bash
# Clone the repository
git clone https://github.com/nithin434/apriltag_rover.git
cd apriltag_rover

# Build ROS2 package (if using ROS2)
colcon build

# Install Python dependencies
pip install -r requirements.txt
```

## Usage

See individual script documentation for usage examples.

## License

TBD
