# OCR-Guided Treasure Hunter

<img src="ucsd_ros2_logos.png" alt="UCSD ROS2 Logo">

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Key Features](#key-features)
- [Package Structure](#package-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Usage Guide](#usage-guide)
- [Troubleshooting](#troubleshooting)
- [Documentation](#documentation)

---

## Overview

This is the **UCSD Team 16 OCR-Guided Treasure Hunter** metapackage, a comprehensive ROS2-based autonomous vehicle framework designed for autonomous navigation, lane following, and intersection handling. The system integrates multiple sensors, actuators, path planning algorithms, and control systems to enable fully autonomous operation.

**Key Capabilities:**
- Real-time lane detection and following using computer vision
- Intersection detection via blue tape recognition
- OCR-based turn direction recognition (LEFT, RIGHT, STRAIGHT, STOP)
- Autonomous turning at intersections
- VESC motor control for precise throttle and steering
- Modular architecture supporting multiple sensor/actuator configurations

**Tested on:** ROS2 Foxy

---

## System Architecture

### High-Level Control Flow

```
┌─────────────────┐
│  Camera (OAK-D) │───┐
└─────────────────┘   │
                      ▼
              ┌──────────────────┐
              │ lane_detection_  │───► /centroid (Float32)
              │      node        │
              └──────────────────┘
                      │
                      ▼
              ┌──────────────────┐
              │ lane_guidance_   │───► /cmd_vel (Twist)
              │      node        │     [Normal lane following]
              └──────────────────┘
                      │
                      ▼
              ┌──────────────────┐
              │ intersection_   │───► /cmd_vel (Twist)
              │   ocr_node       │     [Intersection control]
              └──────────────────┘
                      │
                      ▼
              ┌──────────────────┐
              │  vesc_twist_     │───► VESC Hardware
              │      node        │     [Motor & Steering]
              └──────────────────┘
```

### State Machine (Intersection OCR Node)

The `intersection_ocr_node` implements a state machine with the following states:

1. **`STATE_LANE_FOLLOWING`**: Normal lane following mode
   - `lane_detection_node` detects lane lines and publishes `/centroid` error
   - `lane_guidance_node` uses PID control to publish `/cmd_vel` commands
   - `intersection_ocr_node` only monitors for blue tape (does not publish commands)

2. **`STATE_STOPPED_AT_INTERSECTION`**: Blue tape detected
   - Vehicle immediately stops
   - `lane_detection_node` is disabled
   - OCR processing begins (waits indefinitely for result)
   - High-frequency stop commands (50Hz) override other control nodes

3. **`STATE_TURNING`**: Executing turn based on OCR result
   - Two-phase turning: fixed angle (60%) + transition (40%)
   - Optional lane detection verification for turn completion

4. **`STATE_TURN_COMPLETE`**: Turn finished
   - Re-enables `lane_detection_node`
   - Returns to `STATE_LANE_FOLLOWING`

5. **`STATE_STOP_COMMAND`**: "STOP" OCR command received
   - Shuts down all ROS2 nodes and terminates the system

---

## Key Features

### 1. Lane Detection & Following
- **HSV color filtering** for lane line detection (yellow lines)
- **Contour detection** and centroid calculation
- **PID control** for smooth steering and throttle scheduling
- Real-time calibration via GUI (`calibration_node`)

### 2. Intersection Detection
- **Blue tape detection** using HSV color space filtering
- Configurable detection threshold and debounce count
- Automatic vehicle stopping upon detection

### 3. OCR-Based Turn Recognition
- **Client-server architecture** for OCR processing
- Recognizes: `LEFT`, `RIGHT`, `STRAIGHT`, `STOP`
- Case-insensitive text matching
- Indefinite wait for OCR results (no timeout)

### 4. Autonomous Turning
- **Two-phase turning mechanism**:
  - Phase 1: Fixed steering angle (60% of turn duration)
  - Phase 2: Smooth transition back to lane following (40% of turn duration)
- Optional lane detection verification
- Configurable turn angles and duration

### 5. Hardware Integration
- **VESC motor controller** support (via `/dev/ttyACM0`)
- RPM and servo angle control
- Configurable steering and throttle polarity
- Real-time connection monitoring

---

## Package Structure

### Core Packages

#### `ucsd_robocar_control2_pkg`
**Purpose**: High-level control and decision-making

**Key Nodes:**
- `intersection_ocr_node`: Main decision node for intersection handling
  - State machine implementation
  - Blue tape detection
  - OCR text processing
  - Turn execution

**Launch Files:**
- `complete_intersection_ocr_launch.launch.py`: Complete system launch file

**Configuration:**
- `intersection_ocr_config.yaml`: Blue tape detection, OCR, and turning parameters

#### `ucsd_robocar_lane_detection2_pkg`
**Purpose**: Lane detection and guidance

**Key Nodes:**
- `lane_detection_node`: Detects lane lines and publishes centroid error
  - Subscribes to `/camera/color/image_raw`
  - Publishes `/centroid` (Float32)
  - Can be enabled/disabled via `/lane_detection_control` (Bool)
  
- `lane_guidance_node`: PID control for lane following
  - Subscribes to `/centroid`
  - Publishes `/cmd_vel` (Twist)
  - Implements steering and throttle PID controllers

- `calibration_node`: GUI tool for parameter tuning
  - Real-time HSV parameter adjustment
  - Blue tape detection visualization
  - Parameter saving to YAML files

**Configuration:**
- `ros_racer_calibration.yaml`: Lane detection and PID parameters

#### `ucsd_robocar_actuator2_pkg`
**Purpose**: Hardware actuator control

**Key Nodes:**
- `vesc_twist_node`: VESC motor controller interface
  - Subscribes to `/cmd_vel` (Twist)
  - Converts `linear.x` to RPM (throttle)
  - Converts `angular.z` to servo angle (steering)
  - Communicates with VESC via serial (`/dev/ttyACM0`)

**Configuration:**
- `vesc_twist_calibration.yaml`: VESC motor parameters

#### `ucsd_robocar_sensor2_pkg`
**Purpose**: Sensor drivers and interfaces

**Supported Sensors:**
- Cameras: OAK-D, Intel RealSense, Webcam
- LiDARs: SICK TIM, Livox, BPearl, RP-LiDAR, LD06
- IMU: Artemis
- GPS: u-blox

#### `ucsd_robocar_nav2_pkg`
**Purpose**: Navigation and system orchestration

**Key Launch Files:**
- `all_components.launch.py`: Dynamically launches hardware components based on `car_config.yaml`
- `all_nodes.launch.py`: Launches all nodes based on `node_config.yaml`

**Configuration:**
- `car_config.yaml`: Enable/disable hardware components (1=true, 0=false)
- `node_config.yaml`: Enable/disable nodes/launch files
- `pkg_locations_ucsd.yaml`: Package and launch file mappings

#### `ocr_remote_client`
**Purpose**: OCR client for text recognition

**Key Nodes:**
- `ocr_client_node`: Communicates with external OCR server
  - Subscribes to `/camera/color/image_raw`
  - Publishes `/ocr_text` (String)

---

## Prerequisites

### Hardware Requirements
- **Raspberry Pi** (or x86 system with Docker)
- **OAK-D camera** (or compatible camera)
- **VESC motor controller** (connected via USB `/dev/ttyACM0`)
- **RC car chassis** with steering servo and motor

### Software Requirements
- **ROS2 Foxy** (or compatible version)
- **Python 3.8+**
- **OpenCV 4.x**
- **NumPy**
- **psutil** (for node shutdown functionality)
- **pyvesc** (for VESC communication)

### Docker (Recommended)
The project provides Docker images for faster setup:
- **Docker Image**: `djnighti/ucsd_robocar`
- **Platforms**: Raspberry Pi (ARM64) and x86_64

---

## Installation

### Option 1: Docker (Recommended)

1. **Pull the Docker image:**
   ```bash
   docker pull djnighti/ucsd_robocar
   ```

2. **Run the container:**
   ```bash
   # For Raspberry Pi
   ./docker_setup/bash_scripts/docker_ucsd_jetson.sh
   
   # For x86
   ./docker_setup/bash_scripts/docker_ucsd_x86.sh
   ```

3. **Inside the container, navigate to workspace:**
   ```bash
   cd /home/projects/ros2_ws/src/ucsd_robocar_hub2
   ```

### Option 2: Manual Installation

1. **Clone the repository:**
   ```bash
   git clone <repository_url>
   cd fall-2025-final-project-team-16
   ```

2. **Install dependencies:**
   ```bash
   # Install ROS2 Foxy (if not already installed)
   # See: https://docs.ros.org/en/foxy/Installation.html
   
   # Install Python dependencies
   pip3 install opencv-python numpy psutil pyvesc
   ```

3. **Build the workspace:**
   ```bash
   cd /path/to/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## Quick Start

### 1. Configure Hardware

Edit `ucsd_robocar_nav2_pkg/config/car_config.yaml` to enable your hardware:
```yaml
oakd: 1      # Enable OAK-D camera
vesc: 1      # Enable VESC motor controller
artemis: 1   # Enable IMU (if available)
```

### 2. Launch the Complete System

```bash
ros2 launch ucsd_robocar_control2_pkg complete_intersection_ocr_launch.launch.py
```

This launch file starts:
- All hardware components (camera, VESC, etc.)
- `lane_detection_node` (lane line detection)
- `lane_guidance_node` (PID control for lane following)
- `ocr_client_node` (OCR text recognition)
- `intersection_ocr_node` (intersection handling)

### 3. Monitor Topics

In separate terminals:
```bash
# Monitor vehicle commands
ros2 topic echo /cmd_vel

# Monitor lane detection
ros2 topic echo /centroid

# Monitor OCR results
ros2 topic echo /ocr_text

# Monitor camera feed
ros2 topic hz /camera/color/image_raw
```

---

## Configuration

### Intersection OCR Configuration

Edit `ucsd_robocar_control2_pkg/config/intersection_ocr_config.yaml`:

```yaml
intersection_ocr_node:
  ros__parameters:
    # Blue tape detection (HSV color space)
    blue_hue_low: 100
    blue_hue_high: 130
    blue_saturation_low: 70
    blue_saturation_high: 255
    blue_value_low: 50
    blue_value_high: 255
    blue_detection_threshold: 0.050  # Minimum blue pixel ratio
    blue_detection_debounce_count: 3  # Consecutive detections needed
    
    # Turn parameters
    turn_angle_left: -1.0   # Steering angle for left turn
    turn_angle_right: 1.0   # Steering angle for right turn
    turn_duration: 1.0      # Seconds to execute turn
    turn_speed: 0.15        # Speed during turn
    turn_blend_ratio: 0.6   # Fixed angle phase ratio (60%)
    
    # Lane following
    lane_following_speed: 0.2
```

### Lane Detection Configuration

Edit `ucsd_robocar_lane_detection2_pkg/config/ros_racer_calibration.yaml`:

```yaml
lane_detection_node:
  ros__parameters:
    Hue_low: 15        # Yellow lane line HSV range
    Hue_high: 35
    Saturation_low: 52
    Saturation_high: 255
    Value_low: 139
    Value_high: 255
    crop_width_decimal: 0.89
    rows_to_watch_decimal: 1.0

lane_guidance_node:
  ros__parameters:
    Kp_steering: 0.5    # PID steering gains
    Ki_steering: 0.0
    Kd_steering: 0.1
    max_throttle: 0.1  # Maximum throttle value
    min_throttle: 0.08 # Minimum throttle value
```

### VESC Configuration

Edit `ucsd_robocar_actuator2_pkg/config/vesc_twist_calibration.yaml`:

```yaml
vesc_twist_node:
  ros__parameters:
    max_rpm: 20000      # Maximum RPM
    max_throttle: 0.2   # Maximum throttle (0-1)
    min_throttle: 0.1   # Minimum throttle (0-1)
    steering_polarity: 1
    throttle_polarity: 1
```

### Calibration Tool

Use the GUI calibration tool to tune parameters in real-time:

```bash
ros2 launch ucsd_robocar_lane_detection2_pkg camera_nav_calibration.launch.py
```

Features:
- Real-time HSV parameter adjustment for lane detection
- Blue tape detection visualization
- Parameter saving to YAML files

---

## Usage Guide

### Basic Lane Following

1. **Launch lane following only:**
   ```bash
   ros2 launch ucsd_robocar_lane_detection2_pkg camera_nav.launch.py
   ```

2. **The vehicle will:**
   - Detect yellow lane lines
   - Calculate centroid error
   - Use PID control to follow lanes
   - Publish `/cmd_vel` commands to VESC

### Intersection OCR System

1. **Launch complete system:**
   ```bash
   ros2 launch ucsd_robocar_control2_pkg complete_intersection_ocr_launch.launch.py
   ```

2. **System behavior:**
   - **Normal operation**: Vehicle follows lanes using `lane_guidance_node`
   - **Blue tape detected**: Vehicle stops, OCR processing begins
   - **OCR result received**: Vehicle executes turn (LEFT/RIGHT/STRAIGHT) or stops (STOP)
   - **Turn complete**: Vehicle returns to lane following

### Manual Control

For testing, you can manually publish commands:

```bash
# Stop the vehicle
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Turn left
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: -0.5}}"
```

### Debugging

**Check node status:**
```bash
ros2 node list
ros2 node info /intersection_ocr_node
```

**Monitor topic frequencies:**
```bash
ros2 topic hz /cmd_vel
ros2 topic hz /centroid
ros2 topic hz /ocr_text
```

**View node logs:**
```bash
# All nodes output to screen by default
# Check terminal output for detailed logs
```

---

## Troubleshooting

### Vehicle Not Moving

**Symptoms:** Vehicle detects lanes but doesn't move.

**Diagnosis:**
1. Check if `/cmd_vel` is being published:
   ```bash
   ros2 topic echo /cmd_vel
   ```

2. Check VESC connection:
   ```bash
   ls -l /dev/ttyACM0
   ```

3. Check `vesc_twist_node` logs for RPM values:
   - Look for: `rpm: <value>`
   - If RPM is very low (< 500), check throttle parameters

**Solutions:**
- Verify `max_throttle` and `min_throttle` in `ros_racer_calibration.yaml`
- Check `vesc_twist_node` parameters match your VESC configuration
- Ensure VESC is properly connected and powered

### Blue Tape Not Detected

**Symptoms:** Vehicle doesn't stop at intersections.

**Diagnosis:**
1. Check blue tape detection parameters in `intersection_ocr_config.yaml`
2. Use calibration tool to visualize blue tape detection:
   ```bash
   ros2 launch ucsd_robocar_lane_detection2_pkg camera_nav_calibration.launch.py
   ```

**Solutions:**
- Adjust `blue_hue_low` and `blue_hue_high` for your blue tape color
- Lower `blue_detection_threshold` if detection is too strict
- Increase `blue_detection_debounce_count` if false positives occur

### OCR Not Working

**Symptoms:** Vehicle stops but doesn't receive OCR results.

**Diagnosis:**
1. Check if `ocr_client_node` is running:
   ```bash
   ros2 node list | grep ocr
   ```

2. Check OCR topic:
   ```bash
   ros2 topic echo /ocr_text
   ```

3. Verify OCR server is running and accessible

**Solutions:**
- Ensure OCR server is running and configured correctly
- Check network connectivity if using remote OCR server
- Verify camera feed is being published to `/camera/color/image_raw`

### Turning Direction Reversed

**Symptoms:** Vehicle turns left when it should turn right (or vice versa).

**Solution:**
- Swap `turn_angle_left` and `turn_angle_right` signs in `intersection_ocr_config.yaml`:
  ```yaml
  turn_angle_left: 1.0   # Was -1.0
  turn_angle_right: -1.0 # Was 1.0
  ```

### Lane Detection Not Working

**Symptoms:** No `/centroid` messages published.

**Diagnosis:**
1. Check if `lane_detection_node` is enabled:
   ```bash
   ros2 topic echo /lane_detection_control
   ```

2. Check camera feed:
   ```bash
   ros2 topic hz /camera/color/image_raw
   ```

**Solutions:**
- Verify camera is connected and publishing
- Adjust HSV parameters in `ros_racer_calibration.yaml`
- Use calibration tool to tune parameters in real-time

### VESC Connection Issues

**Symptoms:** `vesc_twist_node` fails to start or reports connection errors.

**Solutions:**
- Check USB connection: `ls -l /dev/ttyACM0`
- Verify permissions: `sudo chmod 666 /dev/ttyACM0`
- Check VESC power and configuration
- Review `vesc_twist_node` logs for specific error messages

---

## Documentation

### Additional Resources

- **UCSD Robocar Framework Guidebook**: [Google Doc](https://docs.google.com/document/d/1YS5YGbo8evIo9Mlb0J-w2r3bZfju37Zl4UmdaN2CD2A/edit?usp=sharing)
- **ROS2 Guide Book**: [Google Doc](https://docs.google.com/document/d/1DJgVLnu_vN-IXKD3QrQVF3W-JC6RiQPVugHeFAioB58/edit?usp=sharing)
- **Docker Image**: [Docker Hub](https://hub.docker.com/r/djnighti/ucsd_robocar)

### Package-Specific READMEs

Each package contains its own README with detailed information:
- `ucsd_robocar_lane_detection2_pkg/README.md`
- `ucsd_robocar_control2_pkg/README.md`
- `ucsd_robocar_actuator2_pkg/README.md`
- `ucsd_robocar_sensor2_pkg/README.md`
- `ucsd_robocar_nav2_pkg/README.md`

### Key Topics

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Camera node | Lane detection, OCR | Camera feed |
| `/centroid` | `std_msgs/Float32` | `lane_detection_node` | `lane_guidance_node` | Lane centroid error |
| `/cmd_vel` | `geometry_msgs/Twist` | `lane_guidance_node`, `intersection_ocr_node` | `vesc_twist_node` | Vehicle control commands |
| `/ocr_text` | `std_msgs/String` | `ocr_client_node` | `intersection_ocr_node` | OCR recognition results |
| `/lane_detection_control` | `std_msgs/Bool` | `intersection_ocr_node` | `lane_detection_node` | Enable/disable lane detection |

---

## Contributing

This is a research project for UCSD. For contributions or questions, please contact the maintainers.

---

## License

TODO: License declaration

---

## Acknowledgments

- UCSD Robocar Team
- ROS2 Community
- VESC Project

---

**Last Updated**: 2025
