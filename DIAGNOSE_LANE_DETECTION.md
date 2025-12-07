# Diagnosing "Nothing detected" in Lane Detection

## Problem
`lane_detection_node` is showing "Nothing detected", meaning it's not finding any yellow lane lines.

## Quick Diagnosis Steps

### 1. Check if Camera is Publishing
```bash
# Check camera topic
ros2 topic hz /camera/color/image_raw

# Should see ~30 Hz. If 0 Hz or topic doesn't exist, camera node is not working.
```

### 2. Check if Lane Detection Node is Receiving Images
```bash
# Check node info
ros2 node info /lane_detection_node

# Should show subscription to /camera/color/image_raw
```

### 3. Enable Debug Visualization
The lane detection node has a `debug_cv` parameter that shows the processed images.

**Option A: Update config file**
Edit `ucsd_robocar_lane_detection2_pkg/config/ros_racer_calibration.yaml`:
```yaml
lane_detection_node:
  ros__parameters:
    debug_cv: 1  # Enable debug windows
    # ... other parameters
```

**Option B: Use calibration node**
```bash
ros2 run ucsd_robocar_lane_detection2_pkg calibration_node
```
This will show:
- Original image
- HSV mask (showing what yellow is detected)
- Processed black/white image

### 4. Check HSV Parameters
Current config shows:
- Hue: 15-35 (yellow range)
- Saturation: 52-255
- Value: 139-255

If these don't match your yellow tape, adjust them using the calibration node.

### 5. Verify Yellow Tape is Visible
- Is yellow tape in the camera's field of view?
- Is it in the lower portion of the image (where the node looks)?
- Is lighting adequate?

## Common Issues

### Issue 1: Camera Not Publishing
**Symptom**: `ros2 topic hz /camera/color/image_raw` shows 0 Hz

**Solution**: 
- Check if camera node is running: `ros2 node list | grep camera`
- Check camera hardware connection
- Restart camera node

### Issue 2: Wrong HSV Parameters
**Symptom**: Camera is publishing but "Nothing detected"

**Solution**:
1. Run calibration node: `ros2 run ucsd_robocar_lane_detection2_pkg calibration_node`
2. Adjust HSV sliders until yellow tape is clearly visible in mask window
3. Save the values to config file

### Issue 3: Yellow Tape Not in View
**Symptom**: Everything works but no detection

**Solution**:
- Check camera angle/position
- Verify yellow tape is in the lower portion of camera view
- Adjust `rows_offset_decimal` and `rows_to_watch_decimal` in config

### Issue 4: Image Processing Issues
**Symptom**: Yellow detected in mask but no lines found

**Solution**:
- Adjust `gray_lower` threshold
- Adjust `kernal_size`, `erosion_itterations`, `dilation_itterations`
- Adjust `Width_min` and `Width_max` to match tape width

## Recommended Action

1. **First, enable debug visualization**:
   ```bash
   # Edit config file to set debug_cv: 1
   # Or use calibration node
   ros2 run ucsd_robocar_lane_detection2_pkg calibration_node
   ```

2. **Check what the node sees**:
   - Original image window
   - HSV mask window (should show yellow areas)
   - Processed image window

3. **If yellow is not visible in mask**:
   - Adjust HSV parameters using sliders
   - Save to config file

4. **If yellow is visible but no lines detected**:
   - Adjust image processing parameters
   - Check `Width_min` and `Width_max` match your tape width

## Quick Fix: Use Calibration Node

The easiest way to fix this is to use the calibration node:

```bash
ros2 run ucsd_robocar_lane_detection2_pkg calibration_node
```

This will:
- Show live camera feed
- Show HSV mask
- Allow real-time parameter adjustment
- Save parameters to config file

Adjust until yellow tape is clearly detected, then save.

