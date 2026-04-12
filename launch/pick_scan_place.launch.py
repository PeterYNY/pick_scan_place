"""
Launch file for the complete Pick-Scan-Place system.
All visualized in RViz with collision objects for table, bins, and object.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_panda = get_package_share_directory('moveit_resources_panda_moveit_config')

    # 1. MoveIt 2 + Panda + RViz
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_panda, 'launch', 'demo.launch.py')),
    )

    # 2. Scene setup (adds table, bins, object to RViz)
    scene_setup = TimerAction(period=5.0, actions=[
        Node(
            package='pick_scan_place',
            executable='scene_setup_node',
            name='scene_setup_node',
            output='screen',
        ),
    ])

    # 3. QR Test Publisher
    qr_publisher = TimerAction(period=6.0, actions=[
        Node(
            package='pick_scan_place',
            executable='qr_test_publisher',
            name='qr_test_publisher',
            output='screen',
            parameters=[{'camera_topic': '/camera/image_raw'}],
        ),
    ])

    # 4. QR Scanner Node
    qr_scanner = TimerAction(period=8.0, actions=[
        Node(
            package='pick_scan_place',
            executable='qr_scanner_node',
            name='qr_scanner_node',
            output='screen',
            parameters=[{'camera_topic': '/camera/image_raw'}],
        ),
    ])

    # 5. Main Pick-Scan-Place Node
    pick_place = TimerAction(period=12.0, actions=[
        Node(
            package='pick_scan_place',
            executable='pick_scan_place_node',
            name='pick_scan_place_node',
            output='screen',
            parameters=[{'planning_group': 'panda_arm'}],
        ),
    ])

    return LaunchDescription([
        moveit,
        scene_setup,
        qr_publisher,
        qr_scanner,
        pick_place,
    ])
