# Gemini Project Guide

This document contains foundational mandates and project-specific instructions for the Gemini CLI agent and developers working on this codebase.

## Project Overview
This workspace is a ROS 2 (Humble) environment for the Mecanum4 robot simulation, camera processing, and robotic arm vision control.

### Key Directories
- `ros2_ws/`: Primary ROS 2 workspace containing robot descriptions and navigation logic.
- `roboarm_ws/`: Robotic arm vision system workspace (YOLO-based detection and docking).
- `robocon_ws/`: Secondary workspace for tutorial and specific competition logic.
- `my_camera_project/`: Camera driver and YOLO detection logic.
- `gazabo/`: Gazebo models and world files (specifically `abu_stadium`).

## Simulation & Vision Utility Commands
The following scripts have been created in the home directory and added as aliases in `~/.bash_aliases`. They automatically source the ROS 2 environment and the necessary workspaces.

- `sim`: Launches basic simulation bringup.
- `simfull`: Launches the full stadium simulation with robot spawning.
- `simnav`: Launches the navigation and SLAM stack.
- `teleop`: Launches the **Mecanum Command Center**, a GUI for 8-way movement.
- `armdock`: Launches the **YOLO Docking Node**, which detects objects (Spear, Rock, Paper) using GPU and automatically docks the robot.

### Usage
Simply type the command name in any terminal:
```bash
armdock
```

## Robotarm Integration (`robotarm_integrated` branch)
The `robotarm_integrated` branch merges the core robot simulation with the vision-based robotic arm features. 

**Key Features added:**
- **Serial Communication**: Real-time data exchange with hardware controllers.
- **Vision Docking**: A new node `yolo_docking_node` in `my_vision_system` package that uses GPU-accelerated YOLO to detect 'spear' and 'fist/rock/paper' and perform automated docking.
- **GPU Acceleration**: YOLO processing is forced to CUDA if an NVIDIA GPU is detected.

## Setup & Installation

Follow these steps to set up the environment on a new machine. **Note: You must build the workspaces for these commands to work.**

### 1. Install Dependencies
Ensure you have ROS 2 Humble installed. Then, install the required system dependencies and GPU drivers:

```bash
# Install System Dependencies
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
  python3-pip \
  python3-tk

# Install NVIDIA Drivers (Required for Gazebo/RViz/YOLO performance)
sudo apt install -y nvidia-driver-535
```

For YOLO detection:
```bash
pip3 install ultralytics opencv-python torch torchvision
```

### 2. Workspace Sourcing & Build
Pull the repository and build all workspaces:

#### Workspace 1: `ros2_ws` (Main Robot & Nav)
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

#### Workspace 2: `roboarm_ws` (Vision & Docking)
```bash
cd ~/roboarm_ws
colcon build --symlink-install
```

### 3. Initialize Utility Scripts
Ensure the simulation scripts are executable and the aliases are loaded:

```bash
# Set executable permissions
chmod +x ~/sim ~/simfull ~/simnav ~/teleop

# Source aliases
source ~/.bash_aliases
```

## Engineering Standards

- **Sourcing:** Always ensure `source /opt/ros/humble/setup.bash` and the relevant workspace `install/setup.bash` are called.
- **Paths:** Use `$HOME` instead of hardcoded absolute paths in scripts to ensure portability.
- **Vision:** The `yolo_docking_node` subscribes to `/camera/image_raw` and `/scan`. Debug output is published to `/camera/debug_image`.
