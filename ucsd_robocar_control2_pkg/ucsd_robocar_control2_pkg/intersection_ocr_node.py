#!/usr/bin/env python3
"""
Intersection OCR Decision Node

This node integrates lane following with intersection detection, OCR text recognition,
and turn decision making. It:
1. Monitors lane following state
2. Detects intersections (via stop line detection or other methods)
3. Stops the vehicle at intersections
4. Uses OCR to recognize turn instructions (left/right/straight)
5. Executes the appropriate turn based on OCR results
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import re
import time
import os
import subprocess
import sys

NODE_NAME = 'intersection_ocr_node'
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
CENTROID_TOPIC_NAME = '/centroid'
OCR_TEXT_TOPIC_NAME = '/ocr_text'
CMD_VEL_TOPIC_NAME = '/cmd_vel'
DRIVE_TOPIC_NAME = '/drive'


class IntersectionOcrNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        
        # State machine states
        self.STATE_LANE_FOLLOWING = 'lane_following'
        self.STATE_APPROACHING_INTERSECTION = 'approaching_intersection'
        self.STATE_STOPPED_AT_INTERSECTION = 'stopped_at_intersection'
        self.STATE_OCR_PROCESSING = 'ocr_processing'
        self.STATE_TURNING = 'turning'
        self.STATE_TURN_COMPLETE = 'turn_complete'
        self.STATE_STOP_COMMAND = 'stop_command'  # Stop command received
        
        self.current_state = self.STATE_LANE_FOLLOWING
        
        # Stop command handling
        self.stop_command_received = False
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, CAMERA_TOPIC_NAME, self.camera_callback, 10
        )
        self.centroid_sub = self.create_subscription(
            Float32, CENTROID_TOPIC_NAME, self.centroid_callback, 10
        )
        self.ocr_text_sub = self.create_subscription(
            String, OCR_TEXT_TOPIC_NAME, self.ocr_text_callback, 10
        )
        
        # Publishers - support both cmd_vel and drive topics
        self.declare_parameter('use_ackermann', False)
        self.use_ackermann = self.get_parameter('use_ackermann').get_parameter_value().bool_value
        
        if self.use_ackermann:
            self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC_NAME, 10)
            self.drive_cmd = AckermannDriveStamped()
            self.drive_cmd.header.frame_id = 'base_link'
        else:
            self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC_NAME, 10)
            self.cmd_vel_cmd = Twist()
            self.get_logger().info(f'Created cmd_vel publisher on topic: {CMD_VEL_TOPIC_NAME}')
        
        # Image processing
        self.bridge = CvBridge()
        self.camera_init = False
        self.latest_image = None
        
        # OCR processing
        self.latest_ocr_text = ""
        self.ocr_text_received = False
        self.ocr_processing_timeout = 5.0  # seconds
        self.ocr_start_time = None
        
        # Intersection detection parameters (blue tape detection)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('blue_tape_detection_enabled', True),
                ('blue_hue_low', 100),  # HSV lower bound for blue color
                ('blue_hue_high', 130),
                ('blue_saturation_low', 50),
                ('blue_saturation_high', 255),
                ('blue_value_low', 50),
                ('blue_value_high', 255),
                ('blue_detection_threshold', 0.05),  # minimum fraction of image that must be blue
                ('blue_detection_debounce_count', 5),  # number of consecutive detections needed to confirm
                ('intersection_distance_threshold', 2.0),  # meters (if using odometry)
                ('stop_duration', 1.0),  # seconds to wait after stopping
                ('ocr_confidence_threshold', 0.5),
                ('turn_angle_left', 1.0),  # steering angle for left turn
                ('turn_angle_right', -1.0),  # steering angle for right turn
                ('turn_duration', 3.0),  # seconds to execute turn
                ('turn_speed', 0.15),  # speed during turn
                ('lane_following_speed', 0.2),  # normal lane following speed
                ('turn_lane_detection_enabled', True),  # Use lane detection to verify turn completion
                ('turn_lane_detection_timeout', 5.0),  # Max time to wait for lane detection during turn
                ('turn_lane_error_threshold', 0.15),  # Centroid error threshold to consider lane found
                ('turn_blend_ratio', 0.6),  # Ratio of turn_duration for fixed angle phase (0.6 = 60%)
                ('stop_command_duration', 2.0),  # Seconds to wait before shutdown after stop command
                ('vesc_check_enabled', True),  # Enable VESC connection checking
                ('vesc_device_path', '/dev/ttyACM0'),  # VESC device file path
                ('vesc_check_interval', 2.0),  # Seconds between VESC connection checks
                ('vesc_node_name', 'vesc_twist_node'),  # Name of VESC node to check
            ]
        )
        
        self.blue_tape_detection_enabled = self.get_parameter('blue_tape_detection_enabled').get_parameter_value().bool_value
        self.blue_hue_low = self.get_parameter('blue_hue_low').get_parameter_value().integer_value
        self.blue_hue_high = self.get_parameter('blue_hue_high').get_parameter_value().integer_value
        self.blue_saturation_low = self.get_parameter('blue_saturation_low').get_parameter_value().integer_value
        self.blue_saturation_high = self.get_parameter('blue_saturation_high').get_parameter_value().integer_value
        self.blue_value_low = self.get_parameter('blue_value_low').get_parameter_value().integer_value
        self.blue_value_high = self.get_parameter('blue_value_high').get_parameter_value().integer_value
        self.blue_detection_threshold = self.get_parameter('blue_detection_threshold').get_parameter_value().double_value
        self.blue_detection_debounce_count = self.get_parameter('blue_detection_debounce_count').get_parameter_value().integer_value
        self.stop_duration = self.get_parameter('stop_duration').get_parameter_value().double_value
        self.turn_angle_left = self.get_parameter('turn_angle_left').get_parameter_value().double_value
        self.turn_angle_right = self.get_parameter('turn_angle_right').get_parameter_value().double_value
        self.turn_duration = self.get_parameter('turn_duration').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.lane_following_speed = self.get_parameter('lane_following_speed').get_parameter_value().double_value
        self.turn_lane_detection_enabled = self.get_parameter('turn_lane_detection_enabled').get_parameter_value().bool_value
        self.turn_lane_detection_timeout = self.get_parameter('turn_lane_detection_timeout').get_parameter_value().double_value
        self.turn_lane_error_threshold = self.get_parameter('turn_lane_error_threshold').get_parameter_value().double_value
        self.turn_blend_ratio = self.get_parameter('turn_blend_ratio').get_parameter_value().double_value
        self.stop_command_duration = self.get_parameter('stop_command_duration').get_parameter_value().double_value
        
        # VESC connection checking
        self.vesc_check_enabled = self.get_parameter('vesc_check_enabled').get_parameter_value().bool_value
        self.vesc_device_path = self.get_parameter('vesc_device_path').get_parameter_value().string_value
        self.vesc_check_interval = self.get_parameter('vesc_check_interval').get_parameter_value().double_value
        self.vesc_node_name = self.get_parameter('vesc_node_name').get_parameter_value().string_value
        self.vesc_connection_failed = False
        
        # State tracking
        self.stop_start_time = None
        self.turn_start_time = None
        self.detected_turn_direction = None  # 'left', 'right', 'straight', 'stop', None
        self.turn_lane_detected = False  # Track if lane is detected during turn
        self.turn_lane_detection_start_time = None
        self.stop_command_start_time = None
        
        # Blue tape detection debouncing (avoid false positives)
        self.blue_tape_detection_count = 0  # Count consecutive detections
        self.blue_tape_detection_threshold_count = 5  # Need N consecutive detections to confirm
        
        # Centroid tracking for lane following
        self.latest_centroid_error = 0.0
        self.centroid_received = False
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        # VESC connection check timer (if enabled)
        if self.vesc_check_enabled:
            self.vesc_check_timer = self.create_timer(self.vesc_check_interval, self.check_vesc_connection)
            # Perform initial VESC check
            if not self.check_vesc_connection_initial():
                self.get_logger().error('VESC connection check failed at startup. Terminating program...')
                self.vesc_connection_failed = True
                self.shutdown_due_to_vesc_failure()
        
        # Debug counters
        self._cmd_log_counter = 0
        self._centroid_log_counter = 0
        self._stop_log_counter = 0
        
        self.get_logger().info(
            f'Intersection OCR Node initialized.\n'
            f'State: {self.current_state}\n'
            f'Using Ackermann: {self.use_ackermann}\n'
            f'Blue tape detection: {self.blue_tape_detection_enabled}\n'
            f'VESC check enabled: {self.vesc_check_enabled}'
        )
    
    def camera_callback(self, msg: Image):
        """Process camera images for intersection detection"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if not self.camera_init:
                self.camera_init = True
                self.get_logger().info('Camera initialized for intersection detection')
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')
    
    def centroid_callback(self, msg: Float32):
        """Receive centroid error from lane detection"""
        self.latest_centroid_error = msg.data
        self.centroid_received = True
        # Log periodically to confirm we're receiving centroid
        if hasattr(self, '_centroid_log_counter'):
            self._centroid_log_counter += 1
        else:
            self._centroid_log_counter = 0
        if self._centroid_log_counter % 50 == 0:  # Log every 50 messages
            self.get_logger().info(f'Received centroid: {msg.data:.3f}')
    
    def ocr_text_callback(self, msg: String):
        """Receive OCR text recognition results"""
        self.latest_ocr_text = msg.data.strip()
        self.ocr_text_received = True
        self.get_logger().info(f'Received OCR text: {self.latest_ocr_text}')
    
    def detect_blue_tape(self, image):
        """
        Detect blue tape in the image using HSV color space filtering.
        Returns True if blue tape is detected (with debouncing to avoid false positives).
        """
        if image is None:
            self.blue_tape_detection_count = 0  # Reset count if no image
            return False
        
        # Convert BGR to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for blue color in HSV
        lower_blue = np.array([self.blue_hue_low, self.blue_saturation_low, self.blue_value_low])
        upper_blue = np.array([self.blue_hue_high, self.blue_saturation_high, self.blue_value_high])
        
        # Create mask for blue color
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Focus on lower portion of image (where blue tape typically appears)
        height, width = blue_mask.shape
        roi_bottom = blue_mask[int(height * 0.5):, :]  # Check bottom 50% of image
        
        # Calculate the fraction of blue pixels in the ROI
        total_pixels = roi_bottom.shape[0] * roi_bottom.shape[1]
        blue_pixels = np.count_nonzero(roi_bottom)
        blue_ratio = blue_pixels / total_pixels if total_pixels > 0 else 0.0
        
        # Debouncing: require consecutive detections to avoid false positives
        if blue_ratio >= self.blue_detection_threshold:
            self.blue_tape_detection_count += 1
            # Log every 5th detection to help with debugging
            if self.blue_tape_detection_count % 5 == 0:
                self.get_logger().info(f'Blue detected: ratio={blue_ratio:.3f}, count={self.blue_tape_detection_count}/{self.blue_detection_debounce_count} (threshold={self.blue_detection_threshold:.3f})')
            
            # Confirm detection only after N consecutive detections
            if self.blue_tape_detection_count >= self.blue_detection_debounce_count:
                self.get_logger().info(f'Blue tape confirmed! Blue ratio: {blue_ratio:.3f} (threshold: {self.blue_detection_threshold}, confirmed after {self.blue_detection_debounce_count} detections)')
                return True
        else:
            # Reset count if detection fails
            if self.blue_tape_detection_count > 0:
                self.get_logger().debug(f'Blue detection reset. Last ratio: {blue_ratio:.3f} (below threshold: {self.blue_detection_threshold:.3f})')
            self.blue_tape_detection_count = 0
        
        # Log blue ratio periodically for debugging (every 50 frames to avoid spam)
        if hasattr(self, '_debug_frame_count'):
            self._debug_frame_count += 1
        else:
            self._debug_frame_count = 0
        
        if self._debug_frame_count % 50 == 0 and blue_ratio > 0.01:  # Only log if there's some blue detected
            self.get_logger().info(f'Blue ratio: {blue_ratio:.3f} (threshold: {self.blue_detection_threshold:.3f}) - {"ABOVE" if blue_ratio >= self.blue_detection_threshold else "BELOW"} threshold')
        
        return False
    
    def detect_intersection(self):
        """
        Detect if we're approaching or at an intersection by detecting blue tape.
        Returns True if intersection (blue tape) is detected.
        """
        if not self.blue_tape_detection_enabled:
            return False
        
        if self.latest_image is None:
            return False
        
        return self.detect_blue_tape(self.latest_image)
    
    def check_vesc_device(self):
        """
        Check if VESC device file exists.
        Returns True if device file exists, False otherwise.
        """
        return os.path.exists(self.vesc_device_path)
    
    def check_vesc_node(self):
        """
        Check if vesc_twist_node is running.
        Returns True if node is running, False otherwise.
        """
        try:
            # Use ros2 node list to check if vesc_twist_node is running
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=2.0
            )
            if result.returncode == 0:
                node_list = result.stdout
                return self.vesc_node_name in node_list
            else:
                self.get_logger().warn(f'Failed to get node list: {result.stderr}')
                return False
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Timeout while checking VESC node status')
            return False
        except Exception as e:
            self.get_logger().warn(f'Error checking VESC node: {e}')
            return False
    
    def check_vesc_connection_initial(self):
        """
        Initial VESC connection check at startup.
        Returns True if VESC is connected, False otherwise.
        """
        if not self.vesc_check_enabled:
            return True  # Skip check if disabled
        
        self.get_logger().info('Performing initial VESC connection check...')
        
        # Check device file
        device_ok = self.check_vesc_device()
        if not device_ok:
            self.get_logger().error(f'VESC device file not found: {self.vesc_device_path}')
            self.get_logger().error('Please check:')
            self.get_logger().error('  1. VESC is powered on')
            self.get_logger().error('  2. USB cable is connected')
            self.get_logger().error('  3. Device permissions are correct (may need to add user to dialout group)')
            return False
        
        self.get_logger().info(f'VESC device file found: {self.vesc_device_path}')
        
        # Wait a bit for node to start (if it hasn't started yet)
        time.sleep(1.0)
        
        # Check if node is running
        node_ok = self.check_vesc_node()
        if not node_ok:
            self.get_logger().error(f'VESC node "{self.vesc_node_name}" is not running')
            self.get_logger().error('Please ensure vesc_twist_node is launched')
            return False
        
        self.get_logger().info(f'VESC node "{self.vesc_node_name}" is running')
        self.get_logger().info('VESC connection check passed')
        return True
    
    def check_vesc_connection(self):
        """
        Periodic VESC connection check.
        If VESC connection fails, terminates the program.
        """
        if not self.vesc_check_enabled:
            return
        
        if self.vesc_connection_failed:
            return  # Already failed, don't check again
        
        # Check device file
        device_ok = self.check_vesc_device()
        if not device_ok:
            self.get_logger().error(f'VESC device file lost: {self.vesc_device_path}')
            self.vesc_connection_failed = True
            self.shutdown_due_to_vesc_failure()
            return
        
        # Check if node is running
        node_ok = self.check_vesc_node()
        if not node_ok:
            self.get_logger().error(f'VESC node "{self.vesc_node_name}" stopped running')
            self.vesc_connection_failed = True
            self.shutdown_due_to_vesc_failure()
            return
    
    def shutdown_due_to_vesc_failure(self):
        """
        Shutdown the program due to VESC connection failure.
        Sets flag to trigger shutdown in main loop.
        """
        self.get_logger().error('=' * 60)
        self.get_logger().error('VESC CONNECTION FAILURE DETECTED')
        self.get_logger().error('=' * 60)
        self.get_logger().error('Terminating program for safety...')
        self.get_logger().error('')
        self.get_logger().error('Please check:')
        self.get_logger().error('  1. VESC is powered on')
        self.get_logger().error('  2. USB cable is properly connected')
        self.get_logger().error('  3. vesc_twist_node is running')
        self.get_logger().error('  4. Device permissions (may need: sudo usermod -a -G dialout $USER)')
        self.get_logger().error('')
        
        # Stop the vehicle immediately
        try:
            self.publish_stop_command()
            time.sleep(0.5)
        except:
            pass  # Ignore errors if node is already shutting down
        
        # Mark as failed - main loop will detect this and exit
        self.vesc_connection_failed = True
        
        # Shutdown ROS2 to stop the spin loop
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore errors if already shutting down
    
    def parse_ocr_text(self, text):
        """
        Parse OCR text to determine turn direction or stop command.
        Returns 'left', 'right', 'straight', 'stop', or None
        """
        if not text:
            return None
        
        text_lower = text.lower()
        
        # Keywords for stop command (check first, highest priority)
        stop_keywords = ['STop', '停止', '停', 'halt', 'end', 'finish']
        for keyword in stop_keywords:
            if keyword in text_lower:
                self.get_logger().info(f'Parsed OCR text "{text}" as STOP command')
                return 'stop'
        
        # Keywords for left turn
        left_keywords = ['左', 'LEFT', 'l', 'turn left', '左转']
        # Keywords for right turn
        right_keywords = ['右', 'RIGHT', 'r', 'turn right', '右转']
        # Keywords for straight
        straight_keywords = ['直', 'straight', 's', 'go straight', '直行', 'forward']
        
        # Check for left turn
        for keyword in left_keywords:
            if keyword in text_lower:
                self.get_logger().info(f'Parsed OCR text "{text}" as LEFT turn')
                return 'left'
        
        # Check for right turn
        for keyword in right_keywords:
            if keyword in text_lower:
                self.get_logger().info(f'Parsed OCR text "{text}" as RIGHT turn')
                return 'right'
        
        # Check for straight
        for keyword in straight_keywords:
            if keyword in text_lower:
                self.get_logger().info(f'Parsed OCR text "{text}" as STRAIGHT')
                return 'straight'
        
        self.get_logger().warn(f'Could not parse OCR text: "{text}"')
        return None
    
    def publish_stop_command(self):
        """Publish stop command (zero velocity)"""
        if self.use_ackermann:
            self.drive_cmd.header.stamp = self.get_clock().now().to_msg()
            self.drive_cmd.drive.speed = 0.0
            self.drive_cmd.drive.steering_angle = 0.0
            self.drive_pub.publish(self.drive_cmd)
        else:
            self.cmd_vel_cmd.linear.x = 0.0
            self.cmd_vel_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel_cmd)
            # Only log stop commands periodically to avoid spam
            if hasattr(self, '_stop_cmd_log_counter'):
                self._stop_cmd_log_counter += 1
            else:
                self._stop_cmd_log_counter = 0
            if self._stop_cmd_log_counter % 50 == 0:  # Log every 2.5 seconds
                self.get_logger().info(f'[CMD] Published STOP: linear.x=0.0, angular.z=0.0')
    
    def publish_lane_following_command(self):
        """Publish lane following command based on centroid error"""
        # Simple proportional control for lane following
        Kp = 1.0
        steering = Kp * self.latest_centroid_error
        
        # Clamp steering
        steering = np.clip(steering, -1.0, 1.0)
        
        if self.use_ackermann:
            self.drive_cmd.header.stamp = self.get_clock().now().to_msg()
            self.drive_cmd.drive.speed = self.lane_following_speed
            self.drive_cmd.drive.steering_angle = steering
            self.drive_pub.publish(self.drive_cmd)
            # Debug log
            self.get_logger().info(f'[CMD] Published Ackermann: speed={self.lane_following_speed:.3f}, steering={steering:.3f}')
        else:
            self.cmd_vel_cmd.linear.x = self.lane_following_speed
            self.cmd_vel_cmd.angular.z = steering
            self.cmd_vel_pub.publish(self.cmd_vel_cmd)
            # Debug log - log every time to see what's being published
            self.get_logger().info(f'[CMD] Published cmd_vel: linear.x={self.cmd_vel_cmd.linear.x:.3f}, angular.z={self.cmd_vel_cmd.angular.z:.3f}, centroid_error={self.latest_centroid_error:.3f}')
    
    def publish_turn_command(self, direction):
        """Publish turn command based on direction"""
        if direction == 'left':
            steering = self.turn_angle_left
        elif direction == 'right':
            steering = self.turn_angle_right
        else:  # straight
            steering = 0.0
        
        if self.use_ackermann:
            self.drive_cmd.header.stamp = self.get_clock().now().to_msg()
            self.drive_cmd.drive.speed = self.turn_speed
            self.drive_cmd.drive.steering_angle = steering
            self.drive_pub.publish(self.drive_cmd)
        else:
            self.cmd_vel_cmd.linear.x = self.turn_speed
            self.cmd_vel_cmd.angular.z = steering
            self.cmd_vel_pub.publish(self.cmd_vel_cmd)
    
    def publish_blended_turn_command(self, direction, blend_factor):
        """
        Blend between fixed turn angle and lane following.
        blend_factor: 0.0 = pure turn, 1.0 = pure lane following
        """
        # Fixed turn component
        if direction == 'left':
            turn_steering = self.turn_angle_left
        elif direction == 'right':
            turn_steering = self.turn_angle_right
        else:
            turn_steering = 0.0
        
        # Lane following component (if centroid available)
        if self.centroid_received:
            Kp = 1.0
            lane_steering = Kp * self.latest_centroid_error
            lane_steering = np.clip(lane_steering, -1.0, 1.0)
        else:
            lane_steering = 0.0
        
        # Blend: transition from turn to lane following
        steering = turn_steering * (1.0 - blend_factor) + lane_steering * blend_factor
        steering = np.clip(steering, -1.0, 1.0)
        
        # Publish blended command
        if self.use_ackermann:
            self.drive_cmd.header.stamp = self.get_clock().now().to_msg()
            self.drive_cmd.drive.speed = self.turn_speed
            self.drive_cmd.drive.steering_angle = steering
            self.drive_pub.publish(self.drive_cmd)
        else:
            self.cmd_vel_cmd.linear.x = self.turn_speed
            self.cmd_vel_cmd.angular.z = steering
            self.cmd_vel_pub.publish(self.cmd_vel_cmd)
    
    def control_loop(self):
        """Main control loop implementing state machine"""
        # Check if VESC connection has failed
        if self.vesc_connection_failed:
            # Don't execute any control commands if VESC is not connected
            if hasattr(self, '_vesc_fail_log_counter'):
                self._vesc_fail_log_counter += 1
            else:
                self._vesc_fail_log_counter = 0
            if self._vesc_fail_log_counter % 100 == 0:  # Log every 5 seconds
                self.get_logger().error('[CONTROL] VESC connection failed - not publishing commands')
            return
        
        current_time = time.time()
        
        # Debug: Log state and centroid status periodically
        if hasattr(self, '_state_log_counter'):
            self._state_log_counter += 1
        else:
            self._state_log_counter = 0
        if self._state_log_counter % 100 == 0:  # Log every 5 seconds
            self.get_logger().info(f'[CONTROL] State={self.current_state}, centroid_received={self.centroid_received}, centroid_error={self.latest_centroid_error:.3f}, vesc_failed={self.vesc_connection_failed}')
        
        # State machine logic
        if self.current_state == self.STATE_LANE_FOLLOWING:
            """
            Lane Following State:
            - lane_detection_node: Detects yellow lane lines, publishes /centroid (error)
            - lane_guidance_node: Uses PID control based on /centroid, publishes /cmd_vel
            - intersection_ocr_node: Only monitors for blue tape, does NOT publish any commands
            - Vehicle control is completely handled by lane_guidance_node
            """
            # Check for blue tape detection (intersection approaching)
            if self.detect_intersection():
                # Blue tape detected - intersection_ocr_node takes control
                self.current_state = self.STATE_APPROACHING_INTERSECTION
                self.get_logger().info(
                    'Blue tape detected! Transitioning to APPROACHING_INTERSECTION. '
                    'intersection_ocr_node taking control from lane_guidance_node'
                )
            else:
                # No blue tape detected - continue normal lane following
                # lane_detection_node and lane_guidance_node handle all vehicle control
                # intersection_ocr_node does NOT publish any commands in this state
                # Log periodically for debugging (every 100 control loops ~5 seconds at 20Hz)
                if hasattr(self, '_monitor_log_counter'):
                    self._monitor_log_counter += 1
                else:
                    self._monitor_log_counter = 0
                if self._monitor_log_counter % 100 == 0:
                    self.get_logger().debug(
                        'STATE_LANE_FOLLOWING: Monitoring for blue tape. '
                        'Vehicle control: lane_detection_node → lane_guidance_node → /cmd_vel'
                    )
        
        elif self.current_state == self.STATE_APPROACHING_INTERSECTION:
            # Stop the vehicle
            self.publish_stop_command()
            self.current_state = self.STATE_STOPPED_AT_INTERSECTION
            self.stop_start_time = current_time
            self.get_logger().info('Transitioning to STOPPED_AT_INTERSECTION')
        
        elif self.current_state == self.STATE_STOPPED_AT_INTERSECTION:
            # Wait for stop duration
            if current_time - self.stop_start_time >= self.stop_duration:
                self.current_state = self.STATE_OCR_PROCESSING
                self.ocr_start_time = current_time
                self.ocr_text_received = False
                self.get_logger().info('Transitioning to OCR_PROCESSING - waiting for OCR result')
            else:
                self.publish_stop_command()
        
        elif self.current_state == self.STATE_OCR_PROCESSING:
            # Wait for OCR result or timeout
            if self.ocr_text_received:
                # Parse OCR text
                self.detected_turn_direction = self.parse_ocr_text(self.latest_ocr_text)
                if self.detected_turn_direction == 'stop':
                    # Stop command received - transition to stop state
                    self.current_state = self.STATE_STOP_COMMAND
                    self.stop_command_start_time = current_time
                    self.get_logger().warn('STOP command received! Shutting down system...')
                elif self.detected_turn_direction in ['left', 'right', 'straight']:
                    # Normal turn command
                    self.current_state = self.STATE_TURNING
                    self.turn_start_time = current_time
                    self.get_logger().info(f'Transitioning to TURNING - direction: {self.detected_turn_direction}')
                else:
                    # If parsing failed, wait a bit more or proceed with default
                    if current_time - self.ocr_start_time > self.ocr_processing_timeout:
                        self.get_logger().warn('OCR parsing failed, proceeding straight')
                        self.detected_turn_direction = 'straight'
                        self.current_state = self.STATE_TURNING
                        self.turn_start_time = current_time
            elif current_time - self.ocr_start_time > self.ocr_processing_timeout:
                # Timeout - proceed straight
                self.get_logger().warn('OCR timeout, proceeding straight')
                self.detected_turn_direction = 'straight'
                self.current_state = self.STATE_TURNING
                self.turn_start_time = current_time
            
            # Keep stopped while processing
            self.publish_stop_command()
        
        elif self.current_state == self.STATE_TURNING:
            # Execute turn with improved lane detection feedback
            elapsed = current_time - self.turn_start_time
            fixed_turn_duration = self.turn_duration * self.turn_blend_ratio  # e.g., 60% of total time
            
            # Phase 1: Fixed angle turn (initial phase)
            if elapsed < fixed_turn_duration:
                self.publish_turn_command(self.detected_turn_direction)
                # Start checking for lane detection
                if self.turn_lane_detection_enabled and not self.turn_lane_detected:
                    if self.centroid_received:
                        self.turn_lane_detected = True
                        self.turn_lane_detection_start_time = current_time
                        self.get_logger().info('Lane detected during turn - transitioning to lane following')
            
            # Phase 2: Transition phase - blend turn with lane following if lane detected
            elif self.turn_lane_detection_enabled and self.turn_lane_detected:
                # Check if we're following the lane well
                if self.centroid_received and abs(self.latest_centroid_error) < self.turn_lane_error_threshold:
                    # Successfully aligned with lane - complete turn early
                    self.current_state = self.STATE_TURN_COMPLETE
                    self.get_logger().info('Turn complete - successfully aligned with new lane')
                elif elapsed >= self.turn_duration:
                    # Timeout - complete anyway
                    self.current_state = self.STATE_TURN_COMPLETE
                    self.get_logger().info('Turn complete - timeout')
                else:
                    # Blend between turn and lane following
                    transition_duration = self.turn_duration - fixed_turn_duration
                    blend_factor = (elapsed - fixed_turn_duration) / transition_duration
                    blend_factor = np.clip(blend_factor, 0.0, 1.0)
                    self.publish_blended_turn_command(self.detected_turn_direction, blend_factor)
            
            # Phase 3: Timeout fallback (no lane detection or lane detection disabled)
            elif elapsed >= self.turn_duration:
                self.current_state = self.STATE_TURN_COMPLETE
                self.get_logger().info('Turn complete - timeout (no lane detection)')
            else:
                # Continue fixed turn
                self.publish_turn_command(self.detected_turn_direction)
        
        elif self.current_state == self.STATE_TURN_COMPLETE:
            # Resume lane following
            self.current_state = self.STATE_LANE_FOLLOWING
            self.detected_turn_direction = None
            self.turn_lane_detected = False
            self.turn_lane_detection_start_time = None
            self.get_logger().info('Resumed lane following')
        
        elif self.current_state == self.STATE_STOP_COMMAND:
            # Stop command received - stop vehicle and prepare for shutdown
            self.publish_stop_command()
            elapsed = current_time - self.stop_command_start_time
            if elapsed >= self.stop_command_duration:
                # Shutdown after waiting period
                self.get_logger().error('STOP command executed. Shutting down node...')
                # Stop vehicle one more time
                self.publish_stop_command()
                # Signal shutdown
                self.stop_command_received = True
                # This will be handled in main() loop


def main(args=None):
    rclpy.init(args=args)
    node = IntersectionOcrNode()
    
    try:
        # Spin with timeout to check for stop command or VESC failure
        while rclpy.ok() and not node.stop_command_received and not node.vesc_connection_failed:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # If VESC connection failed, exit immediately
        if node.vesc_connection_failed:
            node.get_logger().error('VESC connection failure detected in main loop. Exiting...')
            sys.exit(1)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt - shutting down intersection OCR node...')
        node.publish_stop_command()
        time.sleep(0.5)
    except Exception as e:
        node.get_logger().error(f'Error in main loop: {e}')
        node.publish_stop_command()
        time.sleep(0.5)
    finally:
        # Ensure vehicle is stopped
        node.get_logger().info('Stopping vehicle and shutting down...')
        node.publish_stop_command()
        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info('Intersection OCR node shut down successfully.')


if __name__ == '__main__':
    main()

