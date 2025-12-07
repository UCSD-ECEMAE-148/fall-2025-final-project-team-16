# Stop Command Feature

## Overview

The system now supports recognizing a "stop" command via OCR. When the "stop" command is detected, the vehicle will stop and the entire system will shut down safely.

## How It Works

### 1. OCR Recognition

When the vehicle stops at an intersection (blue tape detected), the OCR system reads the text. If it recognizes a stop command, the system enters a shutdown sequence.

### 2. Supported Stop Keywords

The system recognizes the following keywords as stop commands (case-insensitive):
- **English**: "stop", "halt", "end", "finish"
- **Chinese**: "停止", "停"

**Priority**: Stop command has the highest priority and is checked first before turn commands.

### 3. State Machine Flow

```
正常行驶 (LANE_FOLLOWING)
    ↓
检测到蓝色胶带 → 停止 (STOPPED_AT_INTERSECTION)
    ↓
OCR处理 (OCR_PROCESSING)
    ↓
识别到 "stop" → 停止命令状态 (STOP_COMMAND)
    ↓
等待 stop_command_duration 秒
    ↓
安全关闭系统
```

### 4. Safety Features

- **Immediate Stop**: Vehicle stops immediately when stop command is detected
- **Wait Period**: Waits `stop_command_duration` seconds (default: 2.0) before shutdown to ensure vehicle is fully stopped
- **Multiple Stop Commands**: Publishes stop command multiple times to ensure vehicle stops
- **Clean Shutdown**: Properly destroys node and shuts down ROS2

## Configuration

In `intersection_ocr_config.yaml`:

```yaml
intersection_ocr_node:
  ros__parameters:
    # Stop command parameters
    stop_command_duration: 2.0  # seconds to wait before shutdown after stop command
```

## Usage Example

### Scenario: End of Route

1. Place blue tape at the final intersection/stop point
2. Place a sign with "STOP" text visible to the camera
3. Vehicle approaches and detects blue tape
4. Vehicle stops
5. OCR reads "STOP"
6. System enters stop command state
7. After 2 seconds, system shuts down safely

### Log Output

When stop command is recognized, you'll see:
```
[INFO] Parsed OCR text "stop" as STOP command
[WARN] STOP command received! Shutting down system...
[ERROR] STOP command executed. Shutting down node...
[INFO] Stopping vehicle and shutting down...
[INFO] Intersection OCR node shut down successfully.
```

## Testing

### Test Stop Command

1. Start the system:
   ```bash
   ros2 launch ucsd_robocar_control2_pkg intersection_ocr_launch.launch.py
   ```

2. Place blue tape at a test intersection

3. Place a sign with "STOP" text

4. Observe:
   - Vehicle stops at blue tape
   - OCR recognizes "stop"
   - System logs stop command
   - Vehicle remains stopped
   - System shuts down after configured duration

### Verify Shutdown

After stop command:
- Vehicle should be completely stopped
- All nodes should shut down
- ROS2 topics should stop publishing
- System should exit cleanly

## Important Notes

1. **Priority**: Stop command takes priority over all other commands (left/right/straight)

2. **Safety**: The system ensures vehicle is stopped before shutdown by:
   - Publishing stop command multiple times
   - Waiting for a configured duration
   - Verifying shutdown sequence

3. **Manual Override**: You can still manually stop the system using Ctrl+C

4. **Restart**: After shutdown, you need to manually restart the system to resume operation

## Troubleshooting

### Stop Command Not Recognized
- Check OCR text recognition: `ros2 topic echo /ocr_text`
- Verify the text contains one of the stop keywords
- Check OCR server is working correctly
- Ensure text is clearly visible to camera

### System Doesn't Shutdown
- Check logs for stop command recognition
- Verify `stop_command_duration` is set correctly
- Check if there are errors preventing shutdown
- Try manual shutdown (Ctrl+C) as backup

### Vehicle Doesn't Stop
- Verify stop command is being published: `ros2 topic echo /cmd_vel`
- Check actuator connections
- Ensure emergency stop mechanisms are functional

## Integration with Other Systems

If you're using a launch file that starts multiple nodes, the stop command will only shut down the `intersection_ocr_node`. Other nodes will continue running unless they also detect the shutdown signal.

To shut down the entire system, you may need to:
1. Use a custom shutdown service
2. Modify the launch file to handle shutdown signals
3. Use ROS2 lifecycle nodes for coordinated shutdown

