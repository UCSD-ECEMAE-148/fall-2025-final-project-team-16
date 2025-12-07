# Troubleshooting: Vehicle Not Moving

## Problem Diagnosis

If the vehicle is not moving after launching the system, follow these diagnostic steps:

## Step 1: Check if Control Commands are Being Published

```bash
# In a new terminal, check if /cmd_vel is publishing
ros2 topic echo /cmd_vel

# You should see messages like:
# linear:
#   x: 0.2
# angular:
#   z: 0.0
```

**If no messages appear**: The `intersection_ocr_node` is not publishing commands. Go to Step 2.

**If messages appear but are all zeros**: The node is publishing stop commands. Go to Step 2.

**If messages appear with non-zero values**: Commands are being published. Go to Step 4 (check actuators).

## Step 2: Check Lane Detection

The vehicle only moves when it receives centroid error from lane detection:

```bash
# Check if /centroid topic is publishing
ros2 topic echo /centroid

# You should see Float32 values (typically between -1.0 and 1.0)
```

**If no messages appear**:
- Camera node may not be running
- Lane detection may not be detecting lane lines
- Check camera feed: `ros2 topic echo /camera/color/image_raw` (should see image messages)

**If messages appear**: Lane detection is working. Go to Step 3.

## Step 3: Check Camera Feed

```bash
# Check if camera is publishing images
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw

# Should see: /camera/color/image_raw publishing at ~30 Hz
```

**If camera topic doesn't exist or has 0 Hz**:
- **Camera node is not running!** You need to start the camera node separately.

**To start camera node** (depends on your setup):
```bash
# For Intel RealSense (example)
ros2 launch realsense2_camera rs_launch.py

# For other cameras, check your camera package documentation
```

## Step 4: Check Actuator Nodes

Even if `/cmd_vel` is publishing, the vehicle won't move if actuator nodes aren't running:

```bash
# Check for actuator nodes
ros2 node list | grep -E "vesc|adafruit|actuator"

# Check actuator topics
ros2 topic list | grep -E "drive|steering|throttle"
```

**If no actuator nodes are running**:
- You need to start the actuator node separately
- For VESC: `ros2 launch ucsd_robocar_actuator2_pkg vesc_twist.launch.py`
- For Adafruit: Check your actuator package launch files

## Step 5: Check Node Logs

Look at the launch terminal output for clues:

### Expected Log Messages:

**If lane detection is working**:
```
[lane_detection_node-1] [INFO] ... (various lane detection parameters)
```

**If intersection_ocr_node is receiving centroid**:
- No specific log, but check if `/cmd_vel` is publishing non-zero values

**If centroid is NOT received**:
- `intersection_ocr_node` will publish stop commands (linear.x = 0.0)
- Check logs for: "Camera initialized for intersection detection"

## Common Issues and Solutions

### Issue 1: No Camera Node Running

**Symptom**: 
- `/camera/color/image_raw` topic doesn't exist
- `/centroid` topic doesn't exist
- Vehicle doesn't move

**Solution**:
```bash
# Start camera node (example for Intel RealSense)
ros2 launch realsense2_camera rs_launch.py

# Or check your camera package for the correct launch command
```

### Issue 2: Lane Detection Not Finding Lane Lines

**Symptom**:
- `/camera/color/image_raw` exists and publishing
- `/centroid` topic exists but not publishing (or publishing 0.0)
- Vehicle doesn't move

**Solution**:
1. Check lane detection calibration:
   ```bash
   # Use calibration node to adjust HSV values
   ros2 run ucsd_robocar_lane_detection2_pkg calibration_node
   ```

2. Verify lane lines are visible in camera view
3. Adjust HSV parameters in `ros_racer_calibration.yaml`

### Issue 3: No Actuator Node Running

**Symptom**:
- `/cmd_vel` is publishing with non-zero values
- Vehicle still doesn't move

**Solution**:
```bash
# Start actuator node (example for VESC with Twist)
ros2 launch ucsd_robocar_actuator2_pkg vesc_twist.launch.py

# Or for your specific actuator setup
```

### Issue 4: Centroid Not Received

**Symptom**:
- `intersection_ocr_node` logs show it's initialized
- `/cmd_vel` is publishing but all values are 0.0
- Code logic: `if self.centroid_received:` is False

**Solution**:
- Ensure `lane_detection_node` is running (it should be from launch file)
- Check if `/centroid` topic is publishing
- Verify camera is working and lane lines are detected

## Quick Diagnostic Script

Run these commands in sequence to diagnose:

```bash
# 1. Check all nodes
ros2 node list

# 2. Check all topics
ros2 topic list

# 3. Check camera
ros2 topic hz /camera/color/image_raw

# 4. Check centroid
ros2 topic echo /centroid --once

# 5. Check control commands
ros2 topic echo /cmd_vel --once

# 6. Check actuator topics (if applicable)
ros2 topic list | grep -E "drive|steering|throttle"
```

## Expected System State

For the vehicle to move, you need:

1. ✅ **Camera node running** → publishes `/camera/color/image_raw`
2. ✅ **Lane detection node running** → subscribes to camera, publishes `/centroid`
3. ✅ **Intersection OCR node running** → subscribes to `/centroid`, publishes `/cmd_vel`
4. ✅ **Actuator node running** → subscribes to `/cmd_vel`, controls vehicle hardware

## Missing Components

Based on your launch file, it only starts:
- `lane_detection_node` ✅
- `ocr_client_node` ✅
- `intersection_ocr_node` ✅

**It does NOT start**:
- ❌ Camera node (needs to be started separately)
- ❌ Actuator node (needs to be started separately)

## Solution: Update Launch File

You may want to add camera and actuator nodes to the launch file, or start them separately before launching the intersection OCR system.

