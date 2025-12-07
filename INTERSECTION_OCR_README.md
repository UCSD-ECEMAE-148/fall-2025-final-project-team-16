# Intersection OCR Decision System

## Overview

This system integrates lane following, intersection detection, OCR text recognition, and turn decision making. The workflow is as follows:

1. **Lane Following**: Vehicle normally follows lane lines
2. **Intersection Detection**: Detects blue tape using camera image processing
3. **Stop**: Stops the vehicle when blue tape is detected
4. **OCR Recognition**: Uses OCR to recognize turn instructions (left/right/straight) at intersections
5. **Turn Execution**: Executes the appropriate turn based on OCR results

## System Architecture

### Node Components

1. **lane_detection_node**: Detects lane lines and publishes centroid error
2. **ocr_client_node**: OCR client that recognizes text from camera images
3. **intersection_ocr_node**: Main decision node that integrates all functionality

### Topics

- `/camera/color/image_raw` (Image): Camera image input
- `/centroid` (Float32): Centroid error from lane detection
- `/ocr_text` (String): OCR recognized text results
- `/cmd_vel` (Twist) or `/drive` (AckermannDriveStamped): Control command output

## Installation and Build

Ensure all dependent packages are installed:

```bash
# In ROS2 workspace root directory
colcon build --packages-select ucsd_robocar_control2_pkg
colcon build --packages-select ucsd_robocar_lane_detection2_pkg
colcon build --packages-select ocr_remote_client
source install/setup.bash
```

## Usage

### 1. Launch Complete System

```bash
ros2 launch ucsd_robocar_control2_pkg intersection_ocr_launch.launch.py
```

This will launch:
- Lane detection node
- OCR client node
- Intersection OCR decision node

### 2. Configure Parameters

Edit `ucsd_robocar_control2_pkg/config/intersection_ocr_config.yaml` to adjust parameters:

```yaml
intersection_ocr_node:
  ros__parameters:
    # Use AckermannDriveStamped (true) or Twist/cmd_vel (false)
    use_ackermann: false
    
    # Blue tape detection parameters (HSV color space)
    blue_tape_detection_enabled: true
    blue_hue_low: 100        # HSV lower bound for blue color (Hue: 0-179)
    blue_hue_high: 130       # HSV upper bound for blue color
    blue_saturation_low: 50  # Saturation lower bound (0-255)
    blue_saturation_high: 255
    blue_value_low: 50       # Value (brightness) lower bound (0-255)
    blue_value_high: 255
    blue_detection_threshold: 0.05  # minimum fraction of image that must be blue (0.0-1.0)
    
    # Intersection behavior
    stop_duration: 1.0  # seconds to wait after stopping before OCR processing
    
    # OCR processing
    ocr_processing_timeout: 5.0  # seconds to wait for OCR result
    
    # Turn parameters
    turn_angle_left: 1.0   # steering angle for left turn
    turn_angle_right: -1.0  # steering angle for right turn
    turn_duration: 3.0  # seconds to execute turn
    turn_speed: 0.15  # speed during turn
    
    # Lane following parameters
    lane_following_speed: 0.2  # normal lane following speed
```

### 3. OCR Server Setup

Ensure the OCR server is running. The `ocr_client_node` needs to connect to the OCR server.

Default server configuration is in `ocr_remote_client/ocr_remote_client/ocr_client_node.py`:
- `server_host`: OCR server IP address
- `server_port`: OCR server port (default 5001)

You can override via parameters:
```bash
ros2 run ocr_remote_client ocr_client_node --ros-args -p server_host:=192.168.1.67 -p server_port:=5001
```

## State Machine

The system uses a state machine to manage behavior:

1. **LANE_FOLLOWING**: Normal lane following
2. **APPROACHING_INTERSECTION**: Blue tape detected, preparing to stop
3. **STOPPED_AT_INTERSECTION**: Stopped at intersection
4. **OCR_PROCESSING**: Waiting for OCR recognition results
5. **TURNING**: Executing turn action
6. **TURN_COMPLETE**: Turn completed, resuming lane following
7. **STOP_COMMAND**: Stop command received, shutting down system

## OCR Text Recognition Format

The system supports the following text formats to recognize turn instructions:

### Left Turn
- "左", "left", "l", "turn left", "左转"

### Right Turn
- "右", "right", "r", "turn right", "右转"

### Straight
- "直", "straight", "s", "go straight", "直行", "forward"

### Stop (Shutdown)
- "stop", "停止", "停", "halt", "end", "finish"

**Note**: When "stop" command is recognized, the vehicle will stop and the entire system will shut down safely.

## Intersection Detection Method

The current implementation uses **blue tape detection**:
- Converts image to HSV color space
- Creates a mask for blue color using HSV range filtering
- Focuses on the lower portion of the image (bottom 50%) where blue tape typically appears
- Calculates the ratio of blue pixels in the region of interest
- If the blue ratio exceeds the threshold, an intersection is detected

The blue color range in HSV is configurable:
- **Hue**: 100-130 (blue range in HSV, where 0-179 represents the full hue circle)
- **Saturation**: 50-255 (color intensity)
- **Value**: 50-255 (brightness)

### Calibrating Blue Tape Detection

If the blue tape is not detected correctly, adjust the HSV parameters:
1. Use a color picker tool or OpenCV trackbars to find the exact HSV values of your blue tape
2. Adjust `blue_hue_low` and `blue_hue_high` to match your blue tape's hue
3. Adjust `blue_saturation_low/high` and `blue_value_low/high` to filter out unwanted colors
4. Adjust `blue_detection_threshold` to control sensitivity (lower = more sensitive)

If you need other detection methods (e.g., position-based intersection detection), you can modify the `detect_intersection()` method.

## Debugging

### View Node Status

```bash
# View all nodes
ros2 node list

# View topics
ros2 topic list

# View OCR text
ros2 topic echo /ocr_text

# View control commands
ros2 topic echo /cmd_vel
```

### Log Output

The node outputs detailed state transition logs, including:
- State transition information
- OCR recognition results
- Turn decisions
- Errors and warnings

## Troubleshooting

### OCR Recognition Failure
- Check if OCR server is running
- Check network connection
- Check camera image quality
- Increase `ocr_processing_timeout` parameter

### Blue Tape Detection Not Accurate
- Adjust `blue_hue_low`, `blue_hue_high` parameters to match your blue tape color
- Adjust `blue_saturation_low/high` and `blue_value_low/high` to filter better
- Adjust `blue_detection_threshold` parameter (lower for more sensitive, higher for less sensitive)
- Check camera image quality and lighting conditions
- Ensure blue tape is placed in the lower portion of the camera view
- Consider using a calibration tool to find optimal HSV values

### Turn Not Accurate
- Adjust `turn_angle_left` and `turn_angle_right` parameters
- Adjust `turn_duration` parameter
- Check the vehicle's actual steering response

## Extending Functionality

### Adding New Detection Methods

You can add to the `detect_intersection()` method in `intersection_ocr_node.py`:
- Odometry-based position detection
- LiDAR-based intersection detection
- Depth camera-based distance detection

### Improving OCR Parsing

You can add to the `parse_ocr_text()` method:
- More complex text matching rules
- Confidence scoring
- Multi-language support

## Important Notes

1. **Safety First**: Ensure emergency stop mechanisms are in place before actual testing
2. **Parameter Tuning**: Adjust all parameters according to your actual vehicle and environment
3. **OCR Accuracy**: OCR recognition may not always be accurate; consider adding manual confirmation or backup plans
4. **Network Latency**: OCR communicates over network, which may have delays; ensure timeout settings are reasonable
5. **Blue Tape Placement**: Place blue tape horizontally in the lower portion of the camera's field of view for best detection
6. **Lighting Conditions**: Blue tape detection is sensitive to lighting; test under various lighting conditions and adjust HSV parameters accordingly

## Contribution

For issues or improvement suggestions, please submit an issue or pull request.
