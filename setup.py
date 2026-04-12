import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pick_scan_place'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='ROS 2 Pick-Scan-Place Robot System',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'pick_scan_place_node = pick_scan_place.pick_place_node:main',
            'qr_scanner_node = pick_scan_place.qr_scanner_node:main',
            'qr_test_publisher = pick_scan_place.qr_test_publisher:main',
            'scene_setup_node = pick_scan_place.scene_setup_node:main',
        ],
    },
)
