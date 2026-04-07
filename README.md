# Pick-Scan-Place: ROS 2 Robotic Manipulation System


A complete ROS 2-based robotic manipulation pipeline implementing an industrial-style pick-scan-place workflow. The Panda robot picks an object from a fixed station, moves it to a scanning pose, decodes its QR code, and places it in a designated bin based on the encoded data.

## System Architecture

- **Robot:** Franka Emika Panda (7 DOF)
- **Framework:** ROS 2 Humble
- **Motion Planning:** MoveIt 2 (OMPL RRTConnect planner)
- **QR Decoding:** pyzbar + cv_bridge
- **Simulation:** Gazebo + RViz

## Nodes

| Node | Description |
|------|-------------|
| `pick_scan_place_node` | Main orchestrator — executes pick, scan, place workflow |
| `qr_scanner_node` | Subscribes to camera images, decodes QR codes via pyzbar |
| `qr_test_publisher` | Publishes test QR code images for simulation testing |

## Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/barcode` | `std_msgs/String` | Decoded QR code data |
| `/camera/image_raw` | `sensor_msgs/Image` | Camera feed |
| `/move_action` | `MoveGroup` | MoveIt 2 motion commands |
| `/panda_hand_controller/gripper_cmd` | `GripperCommand` | Gripper open/close |

## Decision Logic

| QR Content | Bin Assignment |
|------------|---------------|
| Contains 'A' or 'category_a' | Bin A (Red) |
| Contains 'B' or 'category_b' | Bin B (Blue) |
| Default / timeout | Bin C (Green) |

## Installation

### Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2

### Install Dependencies
```bash
sudo apt install -y ros-humble-moveit \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-moveit-resources-panda-description \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-controllers \
  ros-humble-ros2-control \
  libzbar0

pip3 install pyzbar opencv-python "numpy<2" qrcode[pil]
```

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select pick_scan_place
source install/setup.bash
```

## Usage

### Launch entire system (one command)
```bash
ros2 launch pick_scan_place pick_scan_place.launch.py
```

This starts Gazebo, MoveIt 2, RViz, QR scanner, and the pick-place pipeline automatically.

### Run nodes individually
```bash
# Terminal 1: MoveIt 2 + Panda
ros2 launch moveit_resources_panda_moveit_config demo.launch.py

# Terminal 2: QR Scanner
ros2 run pick_scan_place qr_scanner_node

# Terminal 3: QR Test Publisher
ros2 run pick_scan_place qr_test_publisher

# Terminal 4: Main workflow
ros2 run pick_scan_place pick_scan_place_node
```

## Project Structure
pick_scan_place/
├── pick_scan_place/
│   ├── init.py
│   ├── pick_place_node.py       # Main pick-scan-place orchestrator
│   ├── qr_scanner_node.py       # QR code detection and decoding
│   └── qr_test_publisher.py     # Test QR image publisher
├── launch/
│   └── pick_scan_place.launch.py
├── worlds/
│   └── pick_place.world          # Gazebo world with table and bins
├── config/
├── urdf/
├── package.xml
├── setup.py
└── README.md

