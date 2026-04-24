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
- `teleop`: Launches the **Mecanum Command Center**, a graphical interface for 8-way movement and heading alignment.

### Usage
Simply type the command name in any terminal (after sourcing aliases):
```bash
teleop
```

**Features:**
- **8-Way Control Pad**: Support for Forward, Backward, Strafing, and Diagonals.
- **Heading Alignment**: Snap the robot to specific headings (0°, 90°, 180°, -90°) using odometry feedback.
- **Real-time Status**: Monitor current velocities and robot heading directly in the GUI.
- **Keyboard Bindings**: WASD for translation, Q/E or Arrows for rotation, Space for emergency STOP.

To pass additional arguments (e.g., to change the topic):
```bash
teleop --ros-args -p topic:=/cmd_vel_nav
```

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
  python3-pip

# Install NVIDIA Drivers (Required for Gazebo/RViz performance)
sudo apt install -y nvidia-driver-535
# IMPORTANT: Restart the computer after installation!

# Verify installation after restart:
# nvidia-smi
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

## GPU Configuration (WSL vs Native Linux)

Depending on the environment, the GPU configuration in launch files and scripts must be adjusted for optimal performance.

### Native Linux (Ubuntu)
On native Linux with NVIDIA/AMD hybrid graphics, use **PRIME Render Offload** to force the simulation onto the high-performance GPU.
- **Environment Variables**: 
  - `__NV_PRIME_RENDER_OFFLOAD=1`
  - `__GLX_VENDOR_LIBRARY_NAME=nvidia`
- **Launch Files**: Disable WSL-specific drivers. Ensure `GALLIUM_DRIVER` is NOT set to `d3d12` and `MESA_D3D12_DEFAULT_ADAPTER_NAME` is commented out.

### WSLg (Windows Subsystem for Linux)
On WSLg, Gazebo and RViz should use the Windows DirectX 12 backend for hardware acceleration.
- **Environment Variables**: (Typically handled within the launch file)
  - `GALLIUM_DRIVER=d3d12`
  - `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` (or your specific GPU)
  - `LIBGL_ALWAYS_SOFTWARE=0`
- **Known Issue**: In WSLg, `gzclient` and `rviz2` compete for the same d3d12 device. It is recommended to run Gazebo with `gui:=false` if using RViz extensively.

## Engineering Standards

- **Sourcing:** Always ensure `source /opt/ros/humble/setup.bash` and `source ~/ros2_ws/install/setup.bash` are called before running ROS nodes.
- **Paths:** Use `$HOME` instead of hardcoded `/home/minmin` in scripts to ensure portability across different machines.
- **Simulation:** Default `gui` is set to `false` for performance on WSLg; use it explicitly if needed.

## Git Workflow
The utility scripts (`sim`, `simfull`, `simnav`) and `.bash_aliases` are tracked in this repository even if they are in the home directory.
