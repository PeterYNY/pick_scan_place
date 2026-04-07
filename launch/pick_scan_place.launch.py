"""
Launch file for the complete Pick-Scan-Place system.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_pick = get_package_share_directory('pick_scan_place')
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    pkg_panda = get_package_share_directory('moveit_resources_panda_moveit_config')
    world_file = os.path.join(pkg_pick, 'worlds', 'pick_place.world')
    robot_urdf = os.path.join(pkg_pick, 'urdf', 'panda_arm_gazebo.urdf')

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )

    # 2. Spawn robot arm in Gazebo
    spawn_robot = TimerAction(period=3.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', robot_urdf,
                '-entity', 'panda_arm',
                '-x', '0', '-y', '0', '-z', '0.02',
            ],
            output='screen',
        ),
    ])

    # 3. MoveIt 2 + Panda + RViz
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_panda, 'launch', 'demo.launch.py')),
    )

    # 4. QR Test Publisher
    qr_publisher = TimerAction(period=6.0, actions=[
        Node(
            package='pick_scan_place',
            executable='qr_test_publisher',
            name='qr_test_publisher',
            output='screen',
            parameters=[{'camera_topic': '/camera/image_raw'}],
        ),
    ])

    # 5. QR Scanner Node
    qr_scanner = TimerAction(period=8.0, actions=[
        Node(
            package='pick_scan_place',
            executable='qr_scanner_node',
            name='qr_scanner_node',
            output='screen',
            parameters=[{'camera_topic': '/camera/image_raw'}],
        ),
    ])

    # 6. Main Pick-Scan-Place Node
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
        gazebo,
        spawn_robot,
        moveit,
        qr_publisher,
        qr_scanner,
        pick_place,
    ])
