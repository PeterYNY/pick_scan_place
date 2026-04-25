# Pick-Scan-Place: ROS 2 Robotic Manipulation System

## MAI605 — Robotic Systems (ROS2) | Course Project I

A complete ROS 2-based robotic manipulation pipeline implementing an industrial-style pick-scan-place workflow. The Panda robot picks an object, moves it to a scanning pose, decodes its QR code, and places it in a designated bin based on the encoded data.

## System Architecture

- **Robot:** Franka Emika Panda (7 DOF)
- **Framework:** ROS 2 Humble
- **Motion Planning:** MoveIt 2 (OMPL RRTConnect planner)
- **QR Decoding:** pyzbar + cv_bridge
- **Visualization:** RViz 2

## Quick Start

Launch the entire system with one command, choosing which bin to send the object to:

### Send object to Bin A (Red):
```bash
ros2 launch pick_scan_place pick_scan_place.launch.py qr_data:=category_a
```

### Send object to Bin B (Blue):
```bash
ros2 launch pick_scan_place pick_scan_place.launch.py qr_data:=category_b
```

### Send object to Bin C (Green):
```bash
ros2 launch pick_scan_place pick_scan_place.launch.py qr_data:=category_c
```

After launching, in RViz: click **Add** → **By topic** → `/visualization_marker_array` → **OK** to see the colored scene.

## Installation

### Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2

### Install Dependencies
```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-moveit-resources-panda-description \
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

## Decision Logic

| QR Content | Target Bin | Color |
|------------|-----------|-------|
| `category_a` | Bin A | 🔴 Red |
| `category_b` | Bin B | 🔵 Blue |
| `category_c` / unknown / no QR | Bin C | 🟢 Green |

## Project Structure
## Nodes

| Node | Description |
|------|-------------|
| `pick_scan_place_node` | Main orchestrator — executes pick, scan, place workflow |
| `qr_scanner_node` | Subscribes to camera images, decodes QR codes via pyzbar |
| `qr_test_publisher` | Publishes test QR code images for simulation |
| `scene_setup_node` | Adds colored markers (table, bins, scanner) to RViz |

## Author
- **Course:** MAI605 — Robotic Systems
- **University:** Ajman University
- **Instructor:** Dr. Omar Shalash
