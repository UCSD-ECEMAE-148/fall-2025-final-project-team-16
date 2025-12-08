import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Int32MultiArray
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import os.path
import time
import re
import yaml

# Nodes in this program
CALIBRATION_NODE_NAME = 'calibration_node'

# Nodes listening to rosparameters
LG_NODE_NAME = 'lane_guidance_node'
LD_NODE_NAME = 'lane_detection_node'
VESC_NODE_NAME = 'vesc_twist_node'
ADA_NODE_NAME = 'adafruit_twist_node'

# Topics subscribed/published to in this program
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
ACTUATOR_TOPIC_NAME = '/cmd_vel'

IMG_WINDOW_NAME = 'img'
BW_WINDOW_NAME = 'blackAndWhiteImage'
MASK_WINDOW_NAME = 'mask'
THR_STR_WINDOW_NAME = 'throttle_and_steering'
BLUE_TAPE_WINDOW_NAME = 'blue_tape_detection'

cv2.namedWindow(IMG_WINDOW_NAME)
cv2.namedWindow(BW_WINDOW_NAME)
cv2.namedWindow(MASK_WINDOW_NAME)
cv2.namedWindow(THR_STR_WINDOW_NAME)
cv2.namedWindow(BLUE_TAPE_WINDOW_NAME)

#
def callback(x):
    pass


def slider_to_normalized(slider_input):
    input_start = 0
    input_end = 2000
    output_start = -1
    output_end = 1
    normalized_output = float(output_start + (slider_input - input_start) * (
            (output_end - output_start) / (input_end - input_start)))
    return normalized_output


lowH = 0
highH = 179
lowS = 0
highS = 255
lowV = 0
highV = 255

glow = 0
ghigh = 255

not_inverted = 0
inverted = 1

blur_kernal_min = 1
blur_kernal_max = 50

dilation_min = 1
dilation_max = 10

min_width = 10
max_width = 500

max_number_of_lines = 100
max_error_threshold = 100
default_error_threshold = 20

min_camera_centerline = 50
max_camera_centerline = 100

min_frame_height = 1
max_frame_height = 100
default_frame_width = 100
max_frame_width = 100
default_min_rows = 50
max_rows = 100
default_min_offset = 50
max_offset = 100

steer_left = 0
steer_straight = 1000
steer_right = 2000

Kp_steering_max = 100
Kp_steering_default = 100
Ki_steering_max = 100
Ki_steering_default = 100
Kd_steering_max = 100
Kd_steering_default = 100

throttle_reverse = 0
throttle_neutral = 1000
throttle_forward = 2000

zero_throttle_mode = 0
max_throttle_mode = 1
min_throttle_mode = 2

max_left_steering_mode = 0
straight_steering_mode = 1
max_right_steering_mode = 2

max_rpm = 20000
zero_rpm = 0

steering_polarity_normal = 1
steering_polarity_reversed = 0
throttle_polarity_normal = 1
throttle_polarity_reversed = 0

# CV filtering 
cv2.createTrackbar('lowH', MASK_WINDOW_NAME , lowH, highH, callback)
cv2.createTrackbar('highH', MASK_WINDOW_NAME , highH, highH, callback)
cv2.createTrackbar('lowS', MASK_WINDOW_NAME , lowS, highS, callback)
cv2.createTrackbar('highS', MASK_WINDOW_NAME , highS, highS, callback)
cv2.createTrackbar('lowV', MASK_WINDOW_NAME , lowV, highV, callback)
cv2.createTrackbar('highV', MASK_WINDOW_NAME , highV, highV, callback)

cv2.createTrackbar('gray_lower', BW_WINDOW_NAME , glow, ghigh, callback)
cv2.createTrackbar('Inverted_filter', BW_WINDOW_NAME , not_inverted, inverted, callback)
cv2.createTrackbar('kernal_size', BW_WINDOW_NAME , blur_kernal_min, blur_kernal_max, callback)
cv2.createTrackbar('erosion_itterations', BW_WINDOW_NAME , dilation_min, dilation_max, callback)
cv2.createTrackbar('dilation_itterations', BW_WINDOW_NAME , dilation_min, dilation_max, callback)

# Tracking constraints
cv2.createTrackbar('min_width', IMG_WINDOW_NAME, min_width, max_width, callback)
cv2.createTrackbar('max_width', IMG_WINDOW_NAME, max_width, max_width, callback)
cv2.createTrackbar('number_of_lines', IMG_WINDOW_NAME, max_number_of_lines, max_number_of_lines, callback)
cv2.createTrackbar('error_threshold', IMG_WINDOW_NAME, default_error_threshold, max_error_threshold, callback)
cv2.createTrackbar('camera_centerline', IMG_WINDOW_NAME, min_camera_centerline, max_camera_centerline, callback)

# Image dims
cv2.createTrackbar('frame_width', IMG_WINDOW_NAME, default_frame_width, max_frame_width, callback)
cv2.createTrackbar('rows_to_watch', IMG_WINDOW_NAME, default_min_rows, max_rows, callback)
cv2.createTrackbar('rows_offset', IMG_WINDOW_NAME, default_min_offset, max_offset, callback)

# Steering
cv2.createTrackbar('Kp_steering', THR_STR_WINDOW_NAME, Kp_steering_default, Kp_steering_max, callback)
cv2.createTrackbar('Ki_steering', THR_STR_WINDOW_NAME, Ki_steering_default, Ki_steering_max, callback)
cv2.createTrackbar('Kd_steering', THR_STR_WINDOW_NAME, Kd_steering_default, Kd_steering_max, callback)
cv2.createTrackbar('Steering_mode', THR_STR_WINDOW_NAME, straight_steering_mode, max_right_steering_mode, callback)
cv2.createTrackbar('Steering_value', THR_STR_WINDOW_NAME, steer_straight, steer_right, callback)

# Throttle
cv2.createTrackbar('Throttle_mode', THR_STR_WINDOW_NAME, zero_throttle_mode, min_throttle_mode, callback)
cv2.createTrackbar('Throttle_value', THR_STR_WINDOW_NAME, throttle_neutral, throttle_forward, callback)
cv2.createTrackbar('max_rpm', THR_STR_WINDOW_NAME, max_rpm, max_rpm, callback)

# Actuators polarity
cv2.createTrackbar('steering_polarity', THR_STR_WINDOW_NAME, steering_polarity_normal, steering_polarity_normal, callback)
cv2.createTrackbar('throttle_polarity', THR_STR_WINDOW_NAME, throttle_polarity_normal, throttle_polarity_normal, callback)

# Test motor control
cv2.createTrackbar('test_motor_control', THR_STR_WINDOW_NAME, 0, 1, callback)

# Blue tape detection trackbars
blue_hue_low_default = 100
blue_hue_high_default = 130
blue_sat_low_default = 70
blue_sat_high_default = 255
blue_val_low_default = 50
blue_val_high_default = 255
blue_threshold_default = 8  # 0.08 * 100 for trackbar (0-100 range)
blue_debounce_default = 6

cv2.createTrackbar('blue_hue_low', BLUE_TAPE_WINDOW_NAME, blue_hue_low_default, 179, callback)
cv2.createTrackbar('blue_hue_high', BLUE_TAPE_WINDOW_NAME, blue_hue_high_default, 179, callback)
cv2.createTrackbar('blue_sat_low', BLUE_TAPE_WINDOW_NAME, blue_sat_low_default, 255, callback)
cv2.createTrackbar('blue_sat_high', BLUE_TAPE_WINDOW_NAME, blue_sat_high_default, 255, callback)
cv2.createTrackbar('blue_val_low', BLUE_TAPE_WINDOW_NAME, blue_val_low_default, 255, callback)
cv2.createTrackbar('blue_val_high', BLUE_TAPE_WINDOW_NAME, blue_val_high_default, 255, callback)
cv2.createTrackbar('blue_threshold', BLUE_TAPE_WINDOW_NAME, blue_threshold_default, 100, callback)
cv2.createTrackbar('blue_debounce', BLUE_TAPE_WINDOW_NAME, blue_debounce_default, 20, callback)


class Calibration(Node):
    def __init__(self):
        super().__init__(CALIBRATION_NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.live_calibration_values, 10)
        self.camera_subscriber
        self.bridge = CvBridge()
        self.ek = 0 # initial error for centroid
        self.ek_1 = 0
        self.Ts = float(1/20) # sample period for motors
        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        self.integral_max = 1E-8

        # declare parameters
        

        self.declare_parameters(
            namespace='',
            parameters=[
                ('Hue_low',1),
                ('Hue_high',1),
                ('Saturation_low',1),
                ('Saturation_high',1),
                ('Value_low',1),
                ('Value_high',1),
                ('gray_lower',1),
                ('inverted_filter',0),
                ('kernal_size',1),
                ('erosion_itterations',1),
                ('dilation_itterations',1),
                ('Width_min',1),
                ('Width_max',1),
                ('number_of_lines',1),
                ('crop_width_decimal',0.8),
                ('rows_to_watch_decimal',0.2),
                ('rows_offset_decimal',0.5),
                ('camera_centerline',0.5),
                ('error_threshold', 0.15),
                ('Kp_steering', 1.0),
                ('Ki_steering', 0.0),
                ('Kd_steering', 0.0),
                ('max_right_steering', 1.0),
                ('straight_steering', 0.0),
                ('max_left_steering', -1.0),
                ('zero_throttle',0.0),
                ('max_throttle', 0.2),
                ('min_throttle', 0.1),
                ('max_rpm',1000),
                ('steering_polarity',1),
                ('throttle_polarity',1)
            ])

        # Get previously set params
        self.Hue_low = self.get_parameter('Hue_low').value
        self.Hue_high = self.get_parameter('Hue_high').value
        self.Saturation_low = self.get_parameter('Saturation_low').value
        self.Saturation_high = self.get_parameter('Saturation_high').value
        self.Value_low = self.get_parameter('Value_low').value
        self.Value_high = self.get_parameter('Value_high').value
        self.gray_lower = self.get_parameter('gray_lower').value
        self.inverted_filter = self.get_parameter('inverted_filter').value
        self.kernal_size = self.get_parameter('kernal_size').value
        self.erosion_itterations = self.get_parameter('erosion_itterations').value
        self.dilation_itterations = self.get_parameter('dilation_itterations').value
        self.number_of_lines = self.get_parameter('number_of_lines').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.min_width = self.get_parameter('Width_min').value
        self.max_width = self.get_parameter('Width_max').value

        self.crop_width_decimal = self.get_parameter('crop_width_decimal').value
        self.rows_to_watch_decimal = self.get_parameter('rows_to_watch_decimal').value
        self.rows_offset_decimal = self.get_parameter('rows_offset_decimal').value
        
        self.camera_centerline = self.get_parameter('camera_centerline').value
        self.error_threshold = self.get_parameter('error_threshold').value 

        self.Kp_steering = self.get_parameter('Kp_steering').value
        self.Ki_steering = self.get_parameter('Ki_steering').value
        self.Kd_steering = self.get_parameter('Kd_steering').value
        self.max_right_steering = self.get_parameter('max_right_steering').value
        self.straight_steering = self.get_parameter('straight_steering').value
        self.max_left_steering = self.get_parameter('max_left_steering').value
        
        self.zero_throttle = self.get_parameter('zero_throttle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.min_throttle = self.get_parameter('min_throttle').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.steering_polarity = self.get_parameter('steering_polarity').value
        self.throttle_polarity = self.get_parameter('throttle_polarity').value

        self.get_logger().info(
            f'\nHue_low: {self.Hue_low}'
            f'\nHue_high: {self.Hue_high}'
            f'\nSaturation_low: {self.Saturation_low}'
            f'\nSaturation_high: {self.Saturation_high}'
            f'\nValue_low: {self.Value_low}'
            f'\nValue_high: {self.Value_high}'
            f'\ngray_lower: {self.gray_lower}'
            f'\ninverted_filter: {self.inverted_filter}'
            f'\nkernal_size: {self.kernal_size}'
            f'\nerosion_itterations: {self.erosion_itterations}'
            f'\ndilation_itterations: {self.dilation_itterations}'
            f'\nnumber_of_lines: {self.number_of_lines}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nmin_width: {self.min_width}'
            f'\nmax_width: {self.max_width}'
            f'\ncamera_centerline: {self.camera_centerline}')

        try:
            # Set trackbars to previously saved config
            cv2.setTrackbarPos('lowH', MASK_WINDOW_NAME, self.Hue_low)
            cv2.setTrackbarPos('highH', MASK_WINDOW_NAME, self.Hue_high)
            cv2.setTrackbarPos('lowS', MASK_WINDOW_NAME, self.Saturation_low)
            cv2.setTrackbarPos('highS', MASK_WINDOW_NAME, self.Saturation_high)
            cv2.setTrackbarPos('lowV', MASK_WINDOW_NAME, self.Value_low)
            cv2.setTrackbarPos('highV', MASK_WINDOW_NAME, self.Value_high)
            cv2.setTrackbarPos('gray_lower', BW_WINDOW_NAME, self.gray_lower)
            cv2.setTrackbarPos('Inverted_filter', BW_WINDOW_NAME, self.inverted_filter)
            cv2.setTrackbarPos('kernal_size', BW_WINDOW_NAME, self.kernal_size)
            cv2.setTrackbarPos('erosion_itterations', BW_WINDOW_NAME, self.erosion_itterations)
            cv2.setTrackbarPos('dilation_itterations', BW_WINDOW_NAME, self.dilation_itterations)
            cv2.setTrackbarPos('min_width', IMG_WINDOW_NAME, self.min_width)
            cv2.setTrackbarPos('max_width', IMG_WINDOW_NAME, self.max_width)
            cv2.setTrackbarPos('number_of_lines', IMG_WINDOW_NAME, self.number_of_lines)
            cv2.setTrackbarPos('error_threshold', IMG_WINDOW_NAME, int(self.error_threshold*100))
            cv2.setTrackbarPos('camera_centerline', IMG_WINDOW_NAME, int(self.camera_centerline*100))

            cv2.setTrackbarPos('frame_width', IMG_WINDOW_NAME, int(self.crop_width_decimal * 100))
            cv2.setTrackbarPos('rows_to_watch', IMG_WINDOW_NAME, int(self.rows_to_watch_decimal * 100))
            cv2.setTrackbarPos('rows_offset', IMG_WINDOW_NAME, int(self.rows_offset_decimal * 100))

            cv2.setTrackbarPos('Kp_steering', THR_STR_WINDOW_NAME, int(self.Kp_steering * 100))
            cv2.setTrackbarPos('Ki_steering', THR_STR_WINDOW_NAME, int(self.Ki_steering * 100))
            cv2.setTrackbarPos('Kd_steering', THR_STR_WINDOW_NAME, int(self.Kd_steering * 100))
            cv2.setTrackbarPos('max_rpm', THR_STR_WINDOW_NAME, int(self.max_rpm))
            cv2.setTrackbarPos('steering_polarity', THR_STR_WINDOW_NAME, int(self.steering_polarity))
            cv2.setTrackbarPos('throttle_polarity', THR_STR_WINDOW_NAME, int(self.throttle_polarity))
            
            # Try to load blue tape parameters from intersection_ocr_config.yaml
            intersection_config_path = str('/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_control2_pkg/config/intersection_ocr_config.yaml')
            try:
                with open(intersection_config_path, 'r') as f:
                    intersection_config = yaml.safe_load(f)
                    if 'intersection_ocr_node' in intersection_config and 'ros__parameters' in intersection_config['intersection_ocr_node']:
                        params = intersection_config['intersection_ocr_node']['ros__parameters']
                        blue_hue_low_default = params.get('blue_hue_low', 100)
                        blue_hue_high_default = params.get('blue_hue_high', 130)
                        blue_sat_low_default = params.get('blue_saturation_low', 70)
                        blue_sat_high_default = params.get('blue_saturation_high', 255)
                        blue_val_low_default = params.get('blue_value_low', 50)
                        blue_val_high_default = params.get('blue_value_high', 255)
                        blue_threshold_default = int(params.get('blue_detection_threshold', 0.08) * 100)
                        blue_debounce_default = params.get('blue_detection_debounce_count', 6)
                        
                        cv2.setTrackbarPos('blue_hue_low', BLUE_TAPE_WINDOW_NAME, blue_hue_low_default)
                        cv2.setTrackbarPos('blue_hue_high', BLUE_TAPE_WINDOW_NAME, blue_hue_high_default)
                        cv2.setTrackbarPos('blue_sat_low', BLUE_TAPE_WINDOW_NAME, blue_sat_low_default)
                        cv2.setTrackbarPos('blue_sat_high', BLUE_TAPE_WINDOW_NAME, blue_sat_high_default)
                        cv2.setTrackbarPos('blue_val_low', BLUE_TAPE_WINDOW_NAME, blue_val_low_default)
                        cv2.setTrackbarPos('blue_val_high', BLUE_TAPE_WINDOW_NAME, blue_val_high_default)
                        cv2.setTrackbarPos('blue_threshold', BLUE_TAPE_WINDOW_NAME, blue_threshold_default)
                        cv2.setTrackbarPos('blue_debounce', BLUE_TAPE_WINDOW_NAME, blue_debounce_default)
                        self.get_logger().info(f'Loaded blue tape parameters from {intersection_config_path}')
            except (FileNotFoundError, KeyError, TypeError) as e:
                # Use defaults if file doesn't exist or parameters not found
                self.get_logger().info(f'Using default blue tape parameters (file not found or invalid: {e})')
        except TypeError:
            pass


    def live_calibration_values(self, data):
        
        # get trackbar positions
        self.Hue_low = cv2.getTrackbarPos('lowH', MASK_WINDOW_NAME)
        self.Hue_high = cv2.getTrackbarPos('highH', MASK_WINDOW_NAME)
        self.Saturation_low = cv2.getTrackbarPos('lowS', MASK_WINDOW_NAME)
        self.Saturation_high = cv2.getTrackbarPos('highS', MASK_WINDOW_NAME)
        self.Value_low = cv2.getTrackbarPos('lowV', MASK_WINDOW_NAME)
        self.Value_high = cv2.getTrackbarPos('highV', MASK_WINDOW_NAME)
        self.gray_lower = cv2.getTrackbarPos('gray_lower', BW_WINDOW_NAME)
        self.inverted_filter = cv2.getTrackbarPos('Inverted_filter', BW_WINDOW_NAME)
        self.kernal_size = cv2.getTrackbarPos('kernal_size', BW_WINDOW_NAME)
        self.erosion_itterations = cv2.getTrackbarPos('erosion_itterations', BW_WINDOW_NAME)
        self.dilation_itterations = cv2.getTrackbarPos('dilation_itterations', BW_WINDOW_NAME)
        self.min_width = cv2.getTrackbarPos('min_width', IMG_WINDOW_NAME)
        self.max_width = cv2.getTrackbarPos('max_width', IMG_WINDOW_NAME)
        self.number_of_lines = cv2.getTrackbarPos('number_of_lines', IMG_WINDOW_NAME)
        
        
        self.crop_width_decimal =  max(float(cv2.getTrackbarPos('frame_width', IMG_WINDOW_NAME)/100), 0.01)
        self.rows_to_watch_decimal = max(float(cv2.getTrackbarPos('rows_to_watch', IMG_WINDOW_NAME)/100), 0.01)
        self.rows_offset_decimal = max(float(cv2.getTrackbarPos('rows_offset', IMG_WINDOW_NAME)/100), 0.01)


        self.error_threshold = float(cv2.getTrackbarPos('error_threshold', IMG_WINDOW_NAME)/100)
        self.camera_centerline = float(cv2.getTrackbarPos('camera_centerline', IMG_WINDOW_NAME)/100)
        
        # Motor parameters
        steering_mode = cv2.getTrackbarPos('Steering_mode', THR_STR_WINDOW_NAME)
        steer_input = cv2.getTrackbarPos('Steering_value', THR_STR_WINDOW_NAME)
        self.Kp_steering = float(cv2.getTrackbarPos('Kp_steering', THR_STR_WINDOW_NAME)/100)
        self.Ki_steering = float(cv2.getTrackbarPos('Ki_steering', THR_STR_WINDOW_NAME)/1000000)
        self.Kd_steering = float(cv2.getTrackbarPos('Kd_steering', THR_STR_WINDOW_NAME)/1000000)
        throttle_mode = cv2.getTrackbarPos('Throttle_mode', THR_STR_WINDOW_NAME)
        throttle_input = cv2.getTrackbarPos('Throttle_value', THR_STR_WINDOW_NAME)
        self.max_rpm = int(cv2.getTrackbarPos('max_rpm', THR_STR_WINDOW_NAME))
        
        # Motor polarities
        steering_pol_slider = int(cv2.getTrackbarPos('steering_polarity', THR_STR_WINDOW_NAME))
        throttle_pol_slider = int(cv2.getTrackbarPos('throttle_polarity', THR_STR_WINDOW_NAME))

        # Test Motor parameters
        test_motor_control = int(cv2.getTrackbarPos('test_motor_control', THR_STR_WINDOW_NAME))
        
        if steering_pol_slider == 0:
            self.steering_polarity = int(-1)
        else:
            self.steering_polarity = int(1)
        if throttle_pol_slider == 0:
            self.throttle_polarity = int(-1)
        else:
            self.throttle_polarity = int(1)
        
        # Setting throttle limits based on mode
        if throttle_mode == 0:
            self.zero_throttle = slider_to_normalized(throttle_input)
        elif throttle_mode == 1:
            self.max_throttle = slider_to_normalized(throttle_input)
        elif throttle_mode == 2:
            self.min_throttle = slider_to_normalized(throttle_input)

        # Setting steering limits based on mode
        if steering_mode == 0:
            self.max_left_steering = slider_to_normalized(steer_input)
        elif steering_mode == 1:
            self.straight_steering = slider_to_normalized(steer_input)
        elif steering_mode == 2:
            self.max_right_steering = slider_to_normalized(steer_input)

        if test_motor_control==1:
            # Throttle gain scheduling (function of error)
            self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
            throttle_float_raw = (self.min_throttle - self.max_throttle) * abs(self.ek) + self.inf_throttle
            throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

            # Steering PID terms
            self.proportional_error = self.Kp_steering * self.ek
            self.derivative_error = self.Kd_steering * (self.ek - self.ek_1) / self.Ts
            self.integral_error += self.Ki_steering * self.ek * self.Ts
            self.integral_error = self.clamp(self.integral_error, self.integral_max)
            steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
            steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)

            # Publish actuator control signals
            self.twist_cmd.angular.z = self.steering_polarity * steering_float + self.straight_steering
            self.twist_cmd.linear.x = self.throttle_polarity * throttle_float
            self.twist_publisher.publish(self.twist_cmd)

            # shift error term
            self.ek_1 = self.ek

        elif test_motor_control==0:
            self.ek_1 = 0
            self.integral_error = 0
            # Publish constrained actuator values
            self.twist_cmd.angular.z = self.steering_polarity * slider_to_normalized(steer_input)
            self.twist_cmd.linear.x = self.throttle_polarity * slider_to_normalized(throttle_input)
            self.twist_publisher.publish(self.twist_cmd)

        # Image processing from slider values
        frame = self.bridge.imgmsg_to_cv2(data)
        height, width, channels = frame.shape

        # Setting lower constraints on camera values
        # Vertical crop/pan
        rows_to_watch = int(height * self.rows_to_watch_decimal)
        rows_offset = int(height * (1 - self.rows_offset_decimal))

        # Horizontal crop
        start_height = int(height - rows_offset)
        bottom_height = int(start_height + rows_to_watch)
        left_width = int((width / 2) * (1 - self.crop_width_decimal))
        right_width = int((width / 2) * (1 + self.crop_width_decimal))

        img = frame[start_height:bottom_height, left_width:right_width]
        image_width = right_width-left_width
        image_height = bottom_height-start_height

        # Changing color space to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([self.Hue_low, self.Saturation_low, self.Value_low])
        higher = np.array([self.Hue_high, self.Saturation_high, self.Value_high])
        mask = cv2.inRange(hsv, lower, higher)

        # Inverting color filter option
        if self.inverted_filter == 1:
            bitwise_mask = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_not(mask))
        else:
            bitwise_mask = cv2.bitwise_and(hsv, hsv, mask=mask)

        # Changing to gray color space
        gray = cv2.cvtColor(bitwise_mask, cv2.COLOR_BGR2GRAY)

        # Changing to black and white color space
        gray_upper = 255
        (dummy, blackAndWhiteImage) = cv2.threshold(gray, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        # blackAndWhiteImage = cv2.adaptiveThreshold(gray, gray_upper, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
        
        # Get rid of white noise from grass
        kernel = np.ones((self.kernal_size, self.kernal_size), np.uint8)
        blurred = cv2.blur(blackAndWhiteImage,(self.kernal_size, self.kernal_size))
        erosion = cv2.erode(blurred, kernel, iterations = self.erosion_itterations)
        dilation = cv2.dilate(erosion, kernel, iterations = self.dilation_itterations)

        # Black and white image
        (dummy, blackAndWhiteImage) = cv2.threshold(dilation, self.gray_lower, gray_upper, cv2.THRESH_BINARY)

        # Finding contours in image
        contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Creating points to be drawn on image
        cam_center_line_x = int(image_width * self.camera_centerline)
        start_point = (cam_center_line_x, 0)
        end_point = (cam_center_line_x, int(bottom_height))

        start_point_thresh_pos_x = int(cam_center_line_x -  (self.error_threshold * image_width/2))
        start_point_thresh_neg_x = int(cam_center_line_x + (self.error_threshold * image_width/2))
        
        start_point_thresh_pos = (start_point_thresh_pos_x, 0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x, 0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(bottom_height))

        # Initialize lists
        cx_list = []
        cy_list = []

        # plotting contours and their centroids
        for contour in contours[:self.number_of_lines]:
            [x, y], [w, h], phi = cv2.minAreaRect(contour)
            rect = cv2.minAreaRect(contour)
            if self.min_width < w < self.max_width:
                try:
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    img = cv2.drawContours(img,[box], 0, (0, 255, 0), 3)
                    m = cv2.moments(contour) # moment of contour
                    cx = int(m['m10'] / m['m00']) # x_pos of centroid
                    cy = int(m['m01'] / m['m00']) # y_pos of centroid
                    cx_list.append(cx)
                    cy_list.append(cy)
                    cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
                    img = cv2.line(img, start_point, end_point, (0,255,0), 4)
                    img = cv2.line(img, start_point_thresh_pos, end_point_thresh_pos, (0,0,255), 2)
                    img = cv2.line(img, start_point_thresh_neg, end_point_thresh_neg, (0,0,255), 2)
                except ZeroDivisionError:
                    pass
        try:
            # When more than 1 road mark is found
            if len(cx_list) > 1:
                error_list = []
                count = 0

                # calculate normalized errors for all detected road lines
                for cx_pos in cx_list:
                    error = float((cx_pos - cam_center_line_x) / cam_center_line_x)
                    error_list.append(error)

                # finding average normalized error of all road lines
                avg_error = (sum(error_list) / float(len(error_list)))

                # check difference in normalized error from closest to furthest road line
                p_horizon_diff = abs(error_list[0] - error_list[-1]) 
                
                # if path is approximately straight, then steer towards average normalized error
                if abs(p_horizon_diff) <= self.error_threshold:
                    self.ek = avg_error
                    pixel_error = int(cam_center_line_x * (1 + self.ek))
                    mid_x, mid_y = pixel_error, int((image_height / 2))
                    self.get_logger().info(f"Straight curve: [tracking error: {self.ek}], [tracking angle: {phi}]")
                
                # if path is curved, then steer towards minimum normalized error
                else: 
                    # exclude any road lines within error threshold by making their error large
                    for error in error_list:
                        if abs(error) < self.error_threshold:
                            error = 1
                            error_list[count] = error
                        count+=1
                    
                    # min error (closest roadline)
                    self.ek = min(error_list, key=abs)

                    # get index of min error for plotting
                    error_x_index = error_list.index(min(error_list, key=abs))
                    mid_x, mid_y = cx_list[error_x_index], cy_list[error_x_index]
                    self.get_logger().info(f"Curvy road: [tracking error: {self.ek}], [tracking angle: {phi}]")

                # ploting lines and circles of contours 
                cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
                start_point_error = (cam_center_line_x, mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)

            # When only 1 road mark was found    
            elif len(cx_list) == 1:
                mid_x, mid_y = cx_list[0], cy_list[0]
                start_point_error = (cam_center_line_x, mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
                cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)

                self.ek = float((mid_x - cam_center_line_x) / cam_center_line_x)
                self.get_logger().info(f"Only detected one line: [tracking error: {self.ek}], [tracking angle: {phi}]")

            # When Nothing was found
            else:
                self.get_logger().info(f"Nothing detected")
            
            # clean slate
            cx_list = []
            cy_list = []
        except ValueError:
            pass

        # plotting results
        cv2.imshow(IMG_WINDOW_NAME, img)
        cv2.imshow(MASK_WINDOW_NAME, mask)
        cv2.imshow(BW_WINDOW_NAME, blackAndWhiteImage)
        
        # Blue tape detection visualization
        # Get blue tape trackbar values
        blue_hue_low = cv2.getTrackbarPos('blue_hue_low', BLUE_TAPE_WINDOW_NAME)
        blue_hue_high = cv2.getTrackbarPos('blue_hue_high', BLUE_TAPE_WINDOW_NAME)
        blue_sat_low = cv2.getTrackbarPos('blue_sat_low', BLUE_TAPE_WINDOW_NAME)
        blue_sat_high = cv2.getTrackbarPos('blue_sat_high', BLUE_TAPE_WINDOW_NAME)
        blue_val_low = cv2.getTrackbarPos('blue_val_low', BLUE_TAPE_WINDOW_NAME)
        blue_val_high = cv2.getTrackbarPos('blue_val_high', BLUE_TAPE_WINDOW_NAME)
        blue_threshold_slider = cv2.getTrackbarPos('blue_threshold', BLUE_TAPE_WINDOW_NAME)
        blue_debounce_save = cv2.getTrackbarPos('blue_debounce', BLUE_TAPE_WINDOW_NAME)
        blue_threshold = float(blue_threshold_slider) / 100.0  # Convert from 0-100 to 0.0-1.0
        
        # Process full frame for blue tape detection (not cropped)
        hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([blue_hue_low, blue_sat_low, blue_val_low])
        upper_blue = np.array([blue_hue_high, blue_sat_high, blue_val_high])
        blue_mask = cv2.inRange(hsv_full, lower_blue, upper_blue)
        
        # Focus on lower portion of image (where blue tape typically appears)
        height_full, width_full = blue_mask.shape
        roi_bottom = blue_mask[int(height_full * 0.5):, :]  # Check bottom 50% of image
        
        # Calculate the fraction of blue pixels in the ROI
        total_pixels = roi_bottom.shape[0] * roi_bottom.shape[1]
        blue_pixels = np.count_nonzero(roi_bottom)
        blue_ratio = blue_pixels / total_pixels if total_pixels > 0 else 0.0
        
        # Create visualization image
        blue_detection_img = frame.copy()
        # Draw ROI rectangle
        cv2.rectangle(blue_detection_img, (0, int(height_full * 0.5)), (width_full, height_full), (255, 255, 0), 2)
        
        # Overlay blue mask on original image with transparency
        blue_overlay = np.zeros_like(blue_detection_img, dtype=np.uint8)
        blue_overlay[blue_mask > 0] = [255, 0, 0]  # Blue color for overlay
        blue_detection_img = cv2.addWeighted(blue_detection_img, 1.0, blue_overlay, 0.5, 0)
        
        # Add text showing detection status
        detection_status = "DETECTED" if blue_ratio >= blue_threshold else "NOT DETECTED"
        color = (0, 255, 0) if blue_ratio >= blue_threshold else (0, 0, 255)
        cv2.putText(blue_detection_img, f"Status: {detection_status}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(blue_detection_img, f"Blue Ratio: {blue_ratio:.3f} (Threshold: {blue_threshold:.3f})", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(blue_detection_img, f"HSV: H[{blue_hue_low}-{blue_hue_high}] S[{blue_sat_low}-{blue_sat_high}] V[{blue_val_low}-{blue_val_high}]", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(blue_detection_img, f"Pixels: {blue_pixels}/{total_pixels}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(blue_detection_img, f"Debounce: {blue_debounce_save}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(blue_detection_img, "Press 's' to save parameters", (10, height_full - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Draw a progress bar for blue ratio
        bar_width = 200
        bar_height = 20
        bar_x = width_full - bar_width - 10
        bar_y = 30
        cv2.rectangle(blue_detection_img, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), (100, 100, 100), -1)  # Grey background
        
        threshold_pos = int(bar_x + blue_threshold * bar_width)
        cv2.line(blue_detection_img, (threshold_pos, bar_y), (threshold_pos, bar_y + bar_height), (0, 255, 255), 2)  # Yellow threshold line
        
        current_ratio_pos = int(bar_x + blue_ratio * bar_width)
        cv2.rectangle(blue_detection_img, (bar_x, bar_y), (current_ratio_pos, bar_y + bar_height), color, -1)  # Green/Red fill
        
        cv2.imshow(BLUE_TAPE_WINDOW_NAME, blue_detection_img)
        cv2.imshow('blue_mask', blue_mask)  # Separate mask window
        cv2.waitKey(1)

        # Get blue tape parameters for saving
        blue_hue_low_save = cv2.getTrackbarPos('blue_hue_low', BLUE_TAPE_WINDOW_NAME)
        blue_hue_high_save = cv2.getTrackbarPos('blue_hue_high', BLUE_TAPE_WINDOW_NAME)
        blue_sat_low_save = cv2.getTrackbarPos('blue_sat_low', BLUE_TAPE_WINDOW_NAME)
        blue_sat_high_save = cv2.getTrackbarPos('blue_sat_high', BLUE_TAPE_WINDOW_NAME)
        blue_val_low_save = cv2.getTrackbarPos('blue_val_low', BLUE_TAPE_WINDOW_NAME)
        blue_val_high_save = cv2.getTrackbarPos('blue_val_high', BLUE_TAPE_WINDOW_NAME)
        blue_threshold_save = float(cv2.getTrackbarPos('blue_threshold', BLUE_TAPE_WINDOW_NAME)) / 100.0
        blue_debounce_save = cv2.getTrackbarPos('blue_debounce', BLUE_TAPE_WINDOW_NAME)
        
        # Write files to yaml file for storage. TODO write individual yaml files for each node
        color_config_path = str('/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/config/ros_racer_calibration.yaml')
        f = open(color_config_path, "w")
        f.write(
            f"{LD_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    Hue_low : {self.Hue_low} \n"
            f"    Hue_high : {self.Hue_high} \n"
            f"    Saturation_low : {self.Saturation_low} \n"
            f"    Saturation_high : {self.Saturation_high} \n"
            f"    Value_low : {self.Value_low} \n"
            f"    Value_high : {self.Value_high} \n"
            f"    number_of_lines : {self.number_of_lines} \n"
            f"    error_threshold : {self.error_threshold} \n"
            f"    Width_min : {self.min_width} \n"
            f"    Width_max : {self.max_width} \n"
            f"    gray_lower : {self.gray_lower} \n"
            f"    inverted_filter : {self.inverted_filter} \n"
            f"    kernal_size : {self.kernal_size} \n"
            f"    erosion_itterations : {self.erosion_itterations} \n"
            f"    dilation_itterations : {self.dilation_itterations} \n"
            f"    crop_width_decimal : {self.crop_width_decimal} \n"
            f"    rows_to_watch_decimal : {self.rows_to_watch_decimal} \n"
            f"    rows_offset_decimal : {self.rows_offset_decimal} \n"
            f"    camera_centerline : {self.camera_centerline} \n"
            f"{CALIBRATION_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    Hue_low : {self.Hue_low} \n"
            f"    Hue_high : {self.Hue_high} \n"
            f"    Saturation_low : {self.Saturation_low} \n"
            f"    Saturation_high : {self.Saturation_high} \n"
            f"    Value_low : {self.Value_low} \n"
            f"    Value_high : {self.Value_high} \n"
            f"    number_of_lines : {self.number_of_lines} \n"
            f"    error_threshold : {self.error_threshold} \n"
            f"    Width_min : {self.min_width} \n"
            f"    Width_max : {self.max_width} \n"
            f"    gray_lower : {self.gray_lower} \n"
            f"    inverted_filter : {self.inverted_filter} \n"
            f"    kernal_size : {self.kernal_size} \n"
            f"    erosion_itterations : {self.erosion_itterations} \n"
            f"    dilation_itterations : {self.dilation_itterations} \n"
            f"    crop_width_decimal : {self.crop_width_decimal} \n"
            f"    rows_to_watch_decimal : {self.rows_to_watch_decimal} \n"
            f"    rows_offset_decimal : {self.rows_offset_decimal} \n"
            f"    camera_centerline : {self.camera_centerline} \n"
            f"{LG_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    Kp_steering : {self.Kp_steering} \n"
            f"    Ki_steering : {self.Ki_steering} \n"
            f"    Kd_steering : {self.Kd_steering} \n"
            f"    zero_throttle  : {self.zero_throttle } \n"
            f"    max_throttle : {self.max_throttle} \n"
            f"    min_throttle : {self.min_throttle} \n"
            f"    error_threshold : {self.error_threshold} \n"
            f"    max_right_steering : {self.max_right_steering} \n"
            f"    max_left_steering : {self.max_left_steering} \n"
            f"{VESC_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    max_rpm : {self.max_rpm} \n"
            f"    steering_polarity : {self.steering_polarity} \n"
            f"    throttle_polarity : {self.throttle_polarity} \n"
            f"    zero_throttle  : {self.zero_throttle } \n"
            f"    max_throttle : {self.max_throttle} \n"
            f"    min_throttle : {self.min_throttle} \n"
            f"    max_right_steering : {self.max_right_steering} \n"
            f"    straight_steering : {self.straight_steering} \n"
            f"    max_left_steering : {self.max_left_steering} \n"
            f"{ADA_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    steering_polarity : {self.steering_polarity} \n"
            f"    throttle_polarity : {self.throttle_polarity} \n"
            f"    zero_throttle  : {self.zero_throttle } \n"
            f"    max_throttle : {self.max_throttle} \n"
            f"    min_throttle : {self.min_throttle} \n"
            f"    max_right_steering : {self.max_right_steering} \n"
            f"    straight_steering : {self.straight_steering} \n"
            f"    max_left_steering : {self.max_left_steering} \n"
        )
        f.close()
        
        # Also save blue tape parameters to intersection_ocr_config.yaml
        intersection_config_path = str('/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_control2_pkg/config/intersection_ocr_config.yaml')
        try:
            # Read existing file to preserve other parameters
            with open(intersection_config_path, 'r') as existing_file:
                existing_content = existing_file.read()
            
            # Check if intersection_ocr_node section exists
            if 'intersection_ocr_node:' in existing_content:
                # Update blue tape parameters in the file
                import re
                # Replace blue tape parameters
                patterns = [
                    (r'blue_hue_low:\s*\d+', f'blue_hue_low: {blue_hue_low_save}'),
                    (r'blue_hue_high:\s*\d+', f'blue_hue_high: {blue_hue_high_save}'),
                    (r'blue_saturation_low:\s*\d+', f'blue_saturation_low: {blue_sat_low_save}'),
                    (r'blue_saturation_high:\s*\d+', f'blue_saturation_high: {blue_sat_high_save}'),
                    (r'blue_value_low:\s*\d+', f'blue_value_low: {blue_val_low_save}'),
                    (r'blue_value_high:\s*\d+', f'blue_value_high: {blue_val_high_save}'),
                    (r'blue_detection_threshold:\s*[\d.]+', f'blue_detection_threshold: {blue_threshold_save:.3f}'),
                    (r'blue_detection_debounce_count:\s*\d+', f'blue_detection_debounce_count: {blue_debounce_save}'),
                ]
                
                updated_content = existing_content
                for pattern, replacement in patterns:
                    updated_content = re.sub(pattern, replacement, updated_content)
                
                with open(intersection_config_path, 'w') as intersection_file:
                    intersection_file.write(updated_content)
                self.get_logger().info(f'Blue tape parameters saved to {intersection_config_path}')
            else:
                self.get_logger().warn(f'intersection_ocr_node section not found in {intersection_config_path}, skipping blue tape parameter save')
        except FileNotFoundError:
            self.get_logger().warn(f'File {intersection_config_path} not found, creating new file with blue tape parameters')
            # Create new file with blue tape parameters
            with open(intersection_config_path, 'w') as intersection_file:
                intersection_file.write(
                    f"intersection_ocr_node:\n"
                    f"  ros__parameters:\n"
                    f"    blue_tape_detection_enabled: true\n"
                    f"    blue_hue_low: {blue_hue_low_save}\n"
                    f"    blue_hue_high: {blue_hue_high_save}\n"
                    f"    blue_saturation_low: {blue_sat_low_save}\n"
                    f"    blue_saturation_high: {blue_sat_high_save}\n"
                    f"    blue_value_low: {blue_val_low_save}\n"
                    f"    blue_value_high: {blue_val_high_save}\n"
                    f"    blue_detection_threshold: {blue_threshold_save:.3f}\n"
                    f"    blue_detection_debounce_count: {blue_debounce_save}\n"
                )
        except Exception as e:
            self.get_logger().error(f'Error saving blue tape parameters: {e}')

    def clamp(self, data, upper_bound, lower_bound=None):
            if lower_bound==None:
                lower_bound = -upper_bound # making lower bound symmetric about zero
            if data < lower_bound:
                data_c = lower_bound
            elif data > upper_bound:
                data_c = upper_bound
            else:
                data_c = data
            return data_c 


def main(args=None):
    rclpy.init(args=args)
    CAL_publisher = Calibration()
    try:
        rclpy.spin(CAL_publisher)
        CAL_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print(f"\nShutting down {CALIBRATION_NODE_NAME}...")

        # Stop the car
        CAL_publisher.twist_cmd.linear.x = CAL_publisher.zero_throttle
        CAL_publisher.twist_publisher.publish(CAL_publisher.twist_cmd)

        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        CAL_publisher.destroy_node()
        rclpy.shutdown()
        print(f"{CALIBRATION_NODE_NAME} shut down successfully.")


if __name__ == '__main__':
    main()
