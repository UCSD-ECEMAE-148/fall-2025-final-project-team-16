# Robot Car Startup Guide

## Complete Startup Procedure

This guide provides step-by-step instructions for starting the autonomous robot car with lane following, blue tape intersection detection, and OCR-based turn decision system.

---

## Prerequisites Checklist

Before starting, ensure you have:

- [ ] ROS2 workspace set up (or Docker container running)
- [ ] Camera connected and working
- [ ] Actuators (steering/throttle) connected and calibrated
- [ ] OCR server running (on Mac or another machine)
- [ ] Blue tape placed at intersections
- [ ] Network connection between robot and OCR server

---

## Step-by-Step Startup Procedure

### Step 1: Environment Setup

#### Option A: Using Docker (Recommended)
```bash
# Pull and run the Docker container
# Follow instructions from: https://hub.docker.com/r/djnighti/ucsd_robocar
docker run -it --privileged --network host <docker_image>
```

#### Option B: Native ROS2 Installation
```bash
# Navigate to ROS2 workspace
cd ~/ros2_ws  # or your workspace path
source /opt/ros/foxy/setup.bash  # or your ROS2 distro
```

---

### Step 2: Hardware Verification

#### 2.1 Check Camera
```bash
# Verify camera is detected
ls /dev/video*  # or check camera device

# Test camera feed (optional)
ros2 run ucsd_robocar_basics2_pkg sub_camera_node
```

#### 2.2 Check Actuators
```bash
# For Adafruit PWM board
sudo i2cdetect -y 1

# For VESC
ls /dev/ttyACM0

# Verify topics are publishing
ros2 topic echo /steering
ros2 topic echo /throttle
```

#### 2.3 Verify Sensor Topics
```bash
# Check camera topic
ros2 topic list | grep camera

# Should see: /camera/color/image_raw
```

---

### Step 3: Configure Parameters

#### 3.1 Configure Blue Tape Detection

Edit `ucsd_robocar_control2_pkg/config/intersection_ocr_config.yaml`:

```yaml
intersection_ocr_node:
  ros__parameters:
    # Blue tape detection (adjust based on your blue tape color)
    blue_tape_detection_enabled: true
    blue_hue_low: 100        # Adjust if detection fails
    blue_hue_high: 130
    blue_saturation_low: 50
    blue_saturation_high: 255
    blue_value_low: 50
    blue_value_high: 255
    blue_detection_threshold: 0.05  # Lower = more sensitive
    
    # Other parameters...
    stop_duration: 1.0
    turn_angle_left: 1.0
    turn_angle_right: -1.0
    turn_duration: 3.0
    turn_speed: 0.15
    lane_following_speed: 0.2
```

**Tip**: Use calibration tools to find optimal HSV values for your blue tape.

#### 3.2 Configure OCR Server Connection

Edit `ocr_remote_client/ocr_remote_client/ocr_client_node.py` or override via launch:

```python
# Default values in ocr_client_node.py
server_host = '192.168.1.67'  # Change to your OCR server IP
server_port = 5001
```

Or override via parameters:
```bash
ros2 run ocr_remote_client ocr_client_node \
  --ros-args \
  -p server_host:=192.168.1.67 \
  -p server_port:=5001
```

#### 3.3 Configure Lane Detection (if needed)

Edit `ucsd_robocar_lane_detection2_pkg/config/ros_racer_calibration.yaml`:
- Adjust HSV values for lane line detection
- Adjust crop regions
- Calibrate using `calibration_node` if needed

---

### Step 4: Build Packages

```bash
# Navigate to workspace root
cd ~/ros2_ws  # or your workspace path

# Build required packages
colcon build --packages-select ucsd_robocar_control2_pkg
colcon build --packages-select ucsd_robocar_lane_detection2_pkg
colcon build --packages-select ocr_remote_client

# Source the workspace
source install/setup.bash
```

**Note**: If you modify any Python files, rebuild the packages.

---

### Step 5: Start OCR Server

On your Mac or OCR server machine:

```bash
# Start your OCR server
# (Follow your OCR server setup instructions)
python ocr_server.py  # or your OCR server command
```

**Verify**: OCR server should be listening on the configured IP and port (default: port 5001).

---

### Step 6: Launch the System

#### 6.1 Launch Complete System

```bash
# Make sure you're in the workspace and sourced
source install/setup.bash

# Launch the complete system
ros2 launch ucsd_robocar_control2_pkg intersection_ocr_launch.launch.py
```

This will start:
- `lane_detection_node`: Detects lane lines
- `ocr_client_node`: Connects to OCR server
- `intersection_ocr_node`: Main decision node

#### 6.2 Verify Nodes Are Running

In a new terminal:
```bash
# Check nodes
ros2 node list
# Should see:
# - lane_detection_node
# - ocr_client_node
# - intersection_ocr_node

# Check topics
ros2 topic list
# Should see:
# - /camera/color/image_raw
# - /centroid
# - /ocr_text
# - /cmd_vel (or /drive)
```

---

### Step 7: Monitor System Status

#### 7.1 Monitor Topics

```bash
# Monitor OCR text recognition
ros2 topic echo /ocr_text

# Monitor control commands
ros2 topic echo /cmd_vel

# Monitor lane detection
ros2 topic echo /centroid
```

#### 7.2 Check Logs

Watch the terminal output for:
- State transitions: `LANE_FOLLOWING` → `APPROACHING_INTERSECTION` → `STOPPED_AT_INTERSECTION` → `OCR_PROCESSING` → `TURNING`
- Blue tape detection: `Blue tape detected! Blue ratio: X.XXX`
- OCR results: `Received OCR text: ...`
- Turn decisions: `Transitioning to TURNING - direction: left/right/straight`

---

### Step 8: Testing and Calibration

#### 8.1 Test Blue Tape Detection

1. Place blue tape horizontally in the camera's field of view (lower portion)
2. Watch for log message: `Blue tape detected!`
3. If not detected:
   - Adjust `blue_hue_low/high` in config
   - Adjust `blue_detection_threshold` (lower = more sensitive)
   - Check lighting conditions

#### 8.2 Test OCR Recognition

1. Place text sign with turn instruction (left/right/straight) at intersection
2. Watch for log: `Received OCR text: ...`
3. Verify parsing: `Parsed OCR text "..." as LEFT/RIGHT/STRAIGHT turn`
4. If OCR fails:
   - Check OCR server connection
   - Verify network connectivity
   - Check image quality
   - Increase `ocr_processing_timeout`

#### 8.3 Test Turn Execution

1. Observe vehicle behavior at intersection
2. Adjust turn parameters if needed:
   - `turn_angle_left/right`: Steering angle
   - `turn_duration`: How long to turn
   - `turn_speed`: Speed during turn

---

## Quick Reference Commands

### Start System
```bash
source install/setup.bash
ros2 launch ucsd_robocar_control2_pkg intersection_ocr_launch.launch.py
```

### Monitor Topics
```bash
ros2 topic echo /ocr_text          # OCR results
ros2 topic echo /cmd_vel           # Control commands
ros2 topic echo /centroid          # Lane detection
ros2 topic list                     # All topics
```

### Check Nodes
```bash
ros2 node list                      # All nodes
ros2 node info /intersection_ocr_node  # Node details
```

### Emergency Stop
```bash
# Press Ctrl+C in launch terminal
# Or publish stop command:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## Troubleshooting

### Blue Tape Not Detected
- **Check**: Camera image quality, lighting
- **Fix**: Adjust HSV parameters, lower `blue_detection_threshold`
- **Test**: Use calibration tools to find exact HSV values

### OCR Not Working
- **Check**: OCR server running, network connection
- **Fix**: Verify IP/port, check firewall, increase timeout
- **Test**: `ros2 topic echo /ocr_text` to see if messages arrive

### Vehicle Not Moving
- **Check**: Actuator topics publishing, control commands being sent
- **Fix**: Verify hardware connections, check `/cmd_vel` or `/drive` topics
- **Test**: Manual control to verify actuators work

### Lane Following Not Working
- **Check**: Lane detection calibration, camera view
- **Fix**: Recalibrate using `calibration_node`, adjust crop regions
- **Test**: Monitor `/centroid` topic for lane detection output

---

## Safety Reminders

⚠️ **Before Testing:**
- Ensure emergency stop mechanism is ready
- Test in safe, controlled environment
- Have manual override available
- Verify all safety systems are functional

⚠️ **During Testing:**
- Monitor system logs continuously
- Be ready to stop the vehicle manually
- Watch for unexpected behavior
- Keep clear of vehicle path

---

## Next Steps

After successful startup:
1. Fine-tune parameters based on actual performance
2. Test under various lighting conditions
3. Calibrate blue tape detection for your specific tape
4. Optimize turn parameters for your vehicle
5. Add additional safety checks if needed

---

## Additional Resources

- **Main README**: `INTERSECTION_OCR_README.md`
- **UCSD Robocar Guidebook**: [Link](https://docs.google.com/document/d/1YS5YGbo8evIo9Mlb0J-w2r3bZfju37Zl4UmdaN2CD2A/edit?usp=sharing)
- **ROS2 Guidebook**: [Link](https://docs.google.com/document/d/1DJgVLnu_vN-IXKD3QrQVF3W-JC6RiQPVugHeFAioB58/edit?usp=sharing)

