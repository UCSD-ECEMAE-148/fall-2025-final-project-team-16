# Debug: Vehicle Not Moving Despite Commands

## Problem Analysis

From your logs:
- `vesc_twist_node` shows: `rpm: 499, steering_angle: 0.111`
- `lane_detection_node` shows: `tracking error: -0.776` (detecting lane lines)
- Vehicle is NOT moving

## Calculation

From `vesc_twist_node` code:
- `rpm = int(self.max_rpm * msg.linear.x)`
- If `max_rpm = max_throttle * 20000 = 0.382 * 20000 = 7640`
- Then `msg.linear.x = 499 / 7640 = 0.065`

But `intersection_ocr_node` should publish `lane_following_speed = 0.2`!

## Possible Causes

### 1. intersection_ocr_node Not Publishing
Check if the node is actually publishing:
```bash
ros2 topic echo /cmd_vel
```

### 2. Another Node Overriding Commands
Check what nodes are publishing to `/cmd_vel`:
```bash
ros2 topic info /cmd_vel
# Should show publishers

ros2 node list
# Check if there are other control nodes running
```

### 3. RPM Value Too Low
`rpm: 499` might be too low to move the vehicle. Check:
- VESC configuration
- Minimum RPM threshold
- Motor calibration

## Quick Diagnostic Commands

```bash
# 1. Check what's publishing to /cmd_vel
ros2 topic info /cmd_vel

# 2. See actual cmd_vel values
ros2 topic echo /cmd_vel

# 3. Check intersection_ocr_node status
ros2 node info /intersection_ocr_node

# 4. Check if centroid is being received
ros2 topic echo /centroid
```

## Expected Behavior

If `intersection_ocr_node` is working correctly:
- Should publish `linear.x = 0.2` (lane_following_speed)
- Should publish `angular.z = -0.776` (based on centroid error)
- `vesc_twist_node` should show: `rpm = 7640 * 0.2 = 1528`

But you're seeing `rpm: 499`, which suggests:
- Either `linear.x = 0.065` (much lower than expected)
- Or `max_rpm` is different than calculated

## Solution

Check the actual `/cmd_vel` topic to see what values are being published.

