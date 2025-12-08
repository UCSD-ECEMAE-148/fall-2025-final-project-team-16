import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Complete launch file for intersection OCR system with all hardware components.
    
    This launch file includes:
    1. All hardware components from all_components.launch.py (camera, actuators, etc.)
    2. Camera navigation calibration node (calibration_node)
    3. Lane detection node
    4. OCR client node
    5. Intersection OCR decision node
    """
    
    # Package names
    control_package = 'ucsd_robocar_control2_pkg'
    lane_detection_package = 'ucsd_robocar_lane_detection2_pkg'
    ocr_package = 'ocr_remote_client'
    nav_package = 'ucsd_robocar_nav2_pkg'
    
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
    
    # 1. Include all_components.launch.py - this starts all hardware components
    # (camera, actuators like vesc_twist_node, etc.) based on car_config.yaml
    all_components_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(nav_package),
                'launch',
                'all_components.launch.py'
            )
        )
    )
    ld.add_action(all_components_launch)
    
    # 2. Include camera_nav_calibration.launch.py - this starts calibration_node
    # (GUI tool for calibrating lane detection parameters)
    # DISABLED: Commented out to avoid GUI calibration node
    # camera_nav_calibration_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory(lane_detection_package),
    #             'launch',
    #             'camera_nav_calibration.launch.py'
    #         )
    #     )
    # )
    # ld.add_action(camera_nav_calibration_launch)
    
    # 3. Lane detection node - publishes /centroid error for lane following
    lane_detection_node = Node(
        package=lane_detection_package,
        executable='lane_detection_node',
        output='screen',
        parameters=[lane_detection_config],
        name='lane_detection_node'
    )
    ld.add_action(lane_detection_node)
    
    # 4. OCR client node - communicates with external OCR server
    ocr_client_node = Node(
        package=ocr_package,
        executable='ocr_client_node',
        output='screen',
        name='ocr_client_node'
    )
    ld.add_action(ocr_client_node)
    
    # 5. Intersection OCR decision node - main control node that handles
    # lane following, intersection detection, OCR, and turning decisions
    intersection_ocr_node = Node(
        package=control_package,
        executable='intersection_ocr_node',
        output='screen',
        parameters=[intersection_config],
        name='intersection_ocr_node'
    )
    ld.add_action(intersection_ocr_node)
    
    return ld

