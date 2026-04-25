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
- `lidar`: Launches the **YDLidar ROS 2 Driver** for the S2PRO lidar.
- `yolostart`: Launches the **Optimized YOLO Detection Node** with 60 FPS performance tuning.

### Usage
Simply type the command name in any terminal:
```bash
lidar
```

## Hardware: YDLidar S2PRO
The robot uses a YDLidar S2PRO connected via USB.

### 1. Connection
- **Port**: `/dev/ttyUSB0` (Mapped to `/dev/ydlidar` via udev).
- **Baudrate**: `128000`.

### 2. SDK & Driver Installation
If the lidar driver is missing or needs to be re-setup:

```bash
# 1. Install YDLidar SDK
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK && mkdir build && cd build
cmake .. && make && sudo make install

# 2. Build ROS 2 Driver
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver --cmake-args -Dydlidar_sdk_DIR=/usr/local/lib/cmake/ydlidar_sdk
```

### 3. Startup Scripts
- **Start Script**: `~/ros2_ws/start_ydlidar.sh`
- **Systemd Service**: `~/ros2_ws/ydlidar.service` (for automated background startup).

### 4. Scan Filtering
**Mandatory**: Must use `laser_filters` or a custom `scan_filter` to exclude angles that hit the robot's own wheels/chassis. Without filtering, the `yolo_docking_node` may receive false distance readings from its own wheels, causing incorrect docking behavior.


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

## Optimized Vision System (`yolo_node`)

The YOLO detection system has been optimized for high-performance real-time tracking:

- **Command**: `yolostart` (Alias for `~/yolostart`)
- **Performance**: Targeted 60 FPS using GPU (GTX 1650).
- **Optimizations**:
  - **Resolution**: 640x480 capture for reduced USB/CPU overhead.
  - **Inference**: 640px input size with FP16 (Half Precision) enabled.
  - **Visual Feedback**: Screen updates and image saves are throttled (every 5 frames) to maximize processing speed.
  - **Multi-Detection**: Supports detecting and publishing multiple objects in a single frame.
- **Class Mapping**: 
  - `0`: PAPER
  - `1`: ROCK
  - `2`: SPEAR

## YOLO Model

- **Path**: `roboarm_ws/src/my_vision_system/my_vision_system/models/best.pt`
- **Installed path** (ที่ node โหลดจริง): `roboarm_ws/install/my_vision_system/share/my_vision_system/models/best.pt`
- **ใช้ร่วมกัน**: `yolo_node`, `yolo_docking_node`, `yolo_select_node` โหลดจาก path เดียวกันผ่าน `get_package_share_directory`
- **Last upgraded**: 2026-04-25 (จาก `models_upgrade/best.pt`)

### อัปเกรดโมเดล

```bash
# วางไฟล์ใหม่ไว้ที่ models_upgrade/ แล้วรัน
cp ~/roboarm_ws/src/my_vision_system/models_upgrade/best.pt \
   ~/roboarm_ws/src/my_vision_system/my_vision_system/models/best.pt

cd ~/roboarm_ws && colcon build --packages-select my_vision_system --symlink-install
```

## Picking Selection Node (`yolo_select_node`)

Node สำหรับอ่าน layout ของวัตถุบน rack แล้วตัดสินใจว่าจะหยิบอันไหน ก่อนส่งผลผ่าน Serial ไป Arduino Mega

**ไฟล์**: `roboarm_ws/src/my_vision_system/my_vision_system/yolo_select_node.py`

**รัน**:
```bash
ros2 run my_vision_system yolo_select_node
```

### Pipeline

1. **รอ Serial** — ถ้า Arduino Mega ยังไม่เชื่อมต่อ node จะหยุดรอและไม่ทำ inference ใดๆ
2. **YOLO inference** — ตรวจจับวัตถุทุก frame (30 fps, MJPG 640×480)
3. **Slot assignment** — sort detections จากซ้ายไปขวา (x-center) แล้ว infer ตำแหน่ง slot จาก gap: gap เล็กสุด = 1 slot unit, gap กว้างกว่า = มี slot ว่างคั่น
4. **Stability check** — ต้องเห็น layout เดิมซ้ำ `stable_frames` (default 10) เฟรมติดต่อกัน
5. **Decision** — เลือก class ตาม `priority_order` (default SPEAR > ROCK > PAPER) แล้วเลือก slot ซ้ายสุดของ class นั้น
6. **Output** — ส่งผลทาง ROS topic และ Serial

### Serial Protocol

| ทิศทาง | Format | ตัวอย่าง |
|---|---|---|
| Pi → Arduino | `{slot}\n` | `1\n` |
| Arduino → Pi | ใดๆ (log เท่านั้น) | `OK\n` |

**Auto-detect Arduino Mega** (ลำดับความสำคัญ):
1. `pyserial list_ports` — ตรวจ VID `0x2341` + PID `{0x0042, 0x0010, 0x0016}`
2. `/dev/serial/by-id/` symlink ที่มีคำว่า `arduino` หรือ `mega`
3. Parameter `serial_port` (fallback)

**Autoreconnect**: ลองเชื่อมต่อใหม่ทุก ~5 วินาที ถ้า port หาย — ถ้าตัดสินใจไปแล้วจะ re-send slot ทันทีที่ reconnect สำเร็จ

### ROS Topics

| Topic | Type | รายละเอียด |
|---|---|---|
| `/picking_position` | `Int32` | slot 1-indexed ที่เลือก (TRANSIENT_LOCAL) |
| `/picking_layout` | `String` | layout เช่น `1:SPEAR,2:ROCK,5:PAPER` (TRANSIENT_LOCAL) |
| `/detected_object` | `Float32MultiArray` | `[cx, cy, cls_id, ...]` ทุก frame (เหมือน yolo_node) |

### Parameters

| Parameter | Default | รายละเอียด |
|---|---|---|
| `stable_frames` | `10` | จำนวน frame ที่ต้องเห็น layout เดิมก่อนตัดสินใจ |
| `max_slots` | `6` | จำนวน slot สูงสุดใน rack |
| `priority_order` | `[2, 1, 0]` | SPEAR > ROCK > PAPER |
| `serial_baud` | `115200` | baud rate ของ Arduino Mega |
| `serial_port` | `''` | hint port ถ้า auto-detect ไม่เจอ |
| `conf_thresh` | `0.5` | confidence threshold ของ YOLO |
| `cap_fps` | `30` | FPS ของกล้อง (Jieli max = 30) |

### ข้อจำกัดของ Slot Inference

- ถ้าเห็นวัตถุเพียง 1 ก้อน → assign เป็น slot 1 เสมอ (ไม่มี gap ให้เทียบ)
- slot ที่หายที่ขอบซ้าย/ขวาสุดจะไม่ถูก detect (ต้องรู้ absolute x ของ slot endpoints)
