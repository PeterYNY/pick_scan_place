"""Launch file for Pick-Scan-Place system."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_panda = get_package_share_directory('moveit_resources_panda_moveit_config')

    qr_arg = DeclareLaunchArgument(
        'qr_data', default_value='category_a',
        description='QR code content: category_a, category_b, or category_c')

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_panda, 'launch', 'demo.launch.py')),
    )

    scene = TimerAction(period=5.0, actions=[
        Node(package='pick_scan_place', executable='scene_setup_node',
             name='scene_setup_node', output='screen'),
    ])

    qr_pub = TimerAction(period=6.0, actions=[
        Node(package='pick_scan_place', executable='qr_test_publisher',
             name='qr_test_publisher', output='screen',
             parameters=[{
                 'camera_topic': '/camera/image_raw',
                 'qr_data': LaunchConfiguration('qr_data'),
             }]),
    ])

    qr_scan = TimerAction(period=8.0, actions=[
        Node(package='pick_scan_place', executable='qr_scanner_node',
             name='qr_scanner_node', output='screen',
             parameters=[{'camera_topic': '/camera/image_raw'}]),
    ])

    pick_place = TimerAction(period=12.0, actions=[
        Node(package='pick_scan_place', executable='pick_scan_place_node',
             name='pick_scan_place_node', output='screen'),
    ])

    return LaunchDescription([
        qr_arg,
        moveit,
        scene,
        qr_pub,
        qr_scan,
        pick_place,
    ])
