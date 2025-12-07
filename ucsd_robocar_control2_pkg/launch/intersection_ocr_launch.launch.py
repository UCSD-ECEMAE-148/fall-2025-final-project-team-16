import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the complete lane following + intersection OCR system.
    
    This launches:
    1. Lane detection node
    2. OCR client node
    3. Intersection OCR decision node
    """
    
    # Package names
    control_package = 'ucsd_robocar_control2_pkg'
    lane_detection_package = 'ucsd_robocar_lane_detection2_pkg'
    ocr_package = 'ocr_remote_client'
    
    # Config files
    intersection_config = os.path.join(
        get_package_share_directory(control_package),
        'config',
        'intersection_ocr_config.yaml'
    )
    
    lane_detection_config = os.path.join(
        get_package_share_directory(lane_detection_package),
        'config',
        'ros_racer_calibration.yaml'
    )
    
    ld = LaunchDescription()
    
    # Lane detection node
    lane_detection_node = Node(
        package=lane_detection_package,
        executable='lane_detection_node',
        output='screen',
        parameters=[lane_detection_config],
        name='lane_detection_node'
    )
    
    # OCR client node
    ocr_client_node = Node(
        package=ocr_package,
        executable='ocr_client_node',
        output='screen',
        name='ocr_client_node'
    )
    
    # Intersection OCR decision node
    intersection_ocr_node = Node(
        package=control_package,
        executable='intersection_ocr_node',
        output='screen',
        parameters=[intersection_config],
        name='intersection_ocr_node'
    )
    
    # Note: lane_guidance_node is NOT included here because
    # intersection_ocr_node handles the control commands.
    # If you want to use lane_guidance_node for normal lane following
    # and only use intersection_ocr_node for intersection handling,
    # you would need to modify the architecture (e.g., using a mux).
    
    ld.add_action(lane_detection_node)
    ld.add_action(ocr_client_node)
    ld.add_action(intersection_ocr_node)
    
    return ld

