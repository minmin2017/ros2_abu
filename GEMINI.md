# Gemini Project Guide

This document contains foundational mandates and project-specific instructions for the Gemini CLI agent and developers working on this codebase.

## Project Overview
This workspace is a ROS 2 (Humble) environment for the Mecanum4 robot simulation and camera processing.

### Key Directories
- `ros2_ws/`: Primary ROS 2 workspace containing robot descriptions and navigation logic.
- `robocon_ws/`: Secondary workspace for tutorial and specific competition logic.
- `my_camera_project/`: Camera driver and YOLO detection logic.
- `gazabo/`: Gazebo models and world files (specifically `abu_stadium`).

## Simulation Utility Commands
The following scripts have been created in the home directory and added to the PATH via `~/.local/bin`. They automatically source the ROS 2 environment and the workspace.

- `sim`: Launches basic simulation bringup.
- `simfull`: Launches the full stadium simulation with robot spawning.
- `simnav`: Launches the navigation and SLAM stack.

### Usage
Simply type the command name in any terminal:
```bash
simnav
```
To pass additional arguments (e.g., to enable the GUI):
```bash
simnav gui:=true
```

## Setup & Installation

Follow these steps to set up the environment on a new machine.

### 1. Install Dependencies
Ensure you have ROS 2 Humble installed. Then, install the required system dependencies:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-cv-bridge \
  ros-humble-rqt-image-view \
  python3-serial \
  python3-pip
```

For YOLO detection:
```bash
pip3 install ultralytics opencv-python
```

### 2. Workspace Sourcing & Build
Pull the repository and build all workspaces in the following order:

#### Workspace 1: `ros2_ws` (Main Robot & Nav)
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

#### Workspace 2: `robocon_ws` (Competition Logic)
```bash
cd ~/robocon_ws
colcon build --symlink-install
```

#### Workspace 3: `my_camera_project` (Standalone ROS 2 Package)
```bash
cd ~/my_camera_project
colcon build --symlink-install
```

### 3. Initialize Utility Scripts
After building, ensure the simulation scripts are executable and the aliases are loaded:

```bash
# Set executable permissions
chmod +x ~/sim ~/simfull ~/simnav

# Source aliases
source ~/.bash_aliases
```

## Engineering Standards

- **Sourcing:** Always ensure `source /opt/ros/humble/setup.bash` and `source ~/ros2_ws/install/setup.bash` are called before running ROS nodes.
- **Paths:** Use `$HOME` instead of hardcoded `/home/minmin` in scripts to ensure portability across different machines.
- **Simulation:** Default `gui` is set to `false` for performance on WSLg; use it explicitly if needed.

## Git Workflow
The utility scripts (`sim`, `simfull`, `simnav`) and `.bash_aliases` are tracked in this repository even if they are in the home directory.
