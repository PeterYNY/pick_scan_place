"""Launch file for Pick-Scan-Place system."""

# Standard OS library (used for building file paths)
import os

# ROS2 launch core components
from launch import LaunchDescription

# Actions used to structure launch execution
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument

# Used to include another launch file
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Allows dynamic parameters from launch arguments
from launch.substitutions import LaunchConfiguration

# Used to launch ROS2 nodes
from launch_ros.actions import Node

# Used to get package paths
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get path to Panda MoveIt configuration package
    pkg_panda = get_package_share_directory('moveit_resources_panda_moveit_config')

    # ---------------- Launch Argument ----------------
    # Allows user to change QR content from terminal when launching
    qr_arg = DeclareLaunchArgument(
        'qr_data', default_value='category_a',
        description='QR code content: category_a, category_b, or category_c')

    # Path to RViz configuration file
    rviz_config = os.path.join(
        get_package_share_directory('pick_scan_place'),
        'rviz',
        'pick_scan_place.rviz'
    )

    # ---------------- MoveIt Launch ----------------
    # Launch Panda robot with MoveIt (motion planning)
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_panda, 'launch', 'demo.launch.py')),
        launch_arguments={
            'launch_rviz': 'false'  # Disable default RViz (we use custom one)
        }.items()
    )

    # ---------------- Scene Setup ----------------
    # Starts the visualization environment (tables, conveyor, bins, etc.)
    # Delayed to ensure MoveIt is fully initialized first
    scene = TimerAction(period=5.0, actions=[
        Node(package='pick_scan_place', executable='scene_setup_node',
             name='scene_setup_node', output='screen'),
    ])

    # ---------------- QR Test Publisher ----------------
    # Publishes a fake QR image to simulate camera input
    # Uses qr_data argument to change category
    qr_pub = TimerAction(period=6.0, actions=[
        Node(package='pick_scan_place', executable='qr_test_publisher',
             name='qr_test_publisher', output='screen',
             parameters=[{
                 'camera_topic': '/camera/image_raw',
                 'qr_data': LaunchConfiguration('qr_data'),
             }]),
    ])

    # ---------------- QR Scanner ----------------
    # Subscribes to camera and decodes QR codes
    qr_scan = TimerAction(period=8.0, actions=[
        Node(package='pick_scan_place', executable='qr_scanner_node',
             name='qr_scanner_node', output='screen',
             parameters=[{'camera_topic': '/camera/image_raw'}]),
    ])

    # ---------------- Pick & Place Node ----------------
    # Main logic: robot picks object, scans QR, places in bin
    # Delayed the most to ensure all other nodes are ready
    pick_place = TimerAction(period=12.0, actions=[
        Node(package='pick_scan_place', executable='pick_scan_place_node',
             name='pick_scan_place_node', output='screen'),
    ])

    # ---------------- Launch Description ----------------
    # Order matters: argument + all nodes/actions
    return LaunchDescription([
        qr_arg,
        moveit,
        scene,
        qr_pub,
        qr_scan,
        pick_place,
    ])
