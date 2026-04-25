# Pick-Scan-Place: ROS 2 Robotic Manipulation System

## MAI605 — Robotic Systems (ROS2) | Course Project I

A complete ROS 2-based robotic manipulation pipeline. The Panda robot picks an object, moves it to a scanning pose, decodes its QR code, and places it in the correct bin based on the QR data.

## System Architecture

- **Robot:** Franka Emika Panda (7 DOF)
- **Framework:** ROS 2 Humble
- **Motion Planning:** MoveIt 2 (OMPL RRTConnect planner)
- **QR Decoding:** pyzbar + cv_bridge
- **Visualization:** RViz 2

## Complete Installation Guide (Fresh Ubuntu 22.04)

Follow these steps **in order**. Skipping any step will cause failures.

### Step 1: Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools

# Source ROS 2 automatically in every terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install MoveIt 2 and System Dependencies

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-moveit-resources-panda-description \
  ros-humble-ros2-controllers \
  ros-humble-ros2-control \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rviz2 \
  python3-colcon-common-extensions \
  libzbar0
```

### Step 3: Install Python Dependencies (CRITICAL!)

```bash
pip3 install pyzbar opencv-python qrcode[pil] "numpy<2" --break-system-packages
```

> ⚠️ **WARNING:** Without these Python packages, the QR scanner will silently fail and ALL objects will be placed in Bin C regardless of QR content.

### Step 4: Clone and Build the Project

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PeterYNY/pick_scan_place.git
cd ~/ros2_ws
colcon build --packages-select pick_scan_place

# Source the workspace automatically in every terminal
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 5: Verify Installation

```bash
# Check that the package is found
ros2 pkg list | grep pick_scan_place

# Check that the executables are registered
ros2 pkg executables pick_scan_place
```

You should see:

pick_scan_place pick_place_node

pick_scan_place qr_scanner_node

pick_scan_place qr_test_publisher

pick_scan_place scene_setup_node

## Running the Project

Launch the system and choose which bin to send the object to. Use ONE of these commands:

```bash
# Send to Bin A (Red)
ros2 launch pick_scan_place pick_scan_place.launch.py qr_data:=category_a

# Send to Bin B (Blue)
ros2 launch pick_scan_place pick_scan_place.launch.py qr_data:=category_b

# Send to Bin C (Green)
ros2 launch pick_scan_place pick_scan_place.launch.py qr_data:=category_c
```

### Important: Configure RViz to See the Scene

When RViz opens, the bins and table won't be visible by default. Add them:

1. Click **Add** (bottom-left of RViz)
2. Click the **By topic** tab
3. Find `/visualization_marker_array` and select **MarkerArray**
4. Click **OK**

You should now see the brown table, white cube, three colored bins, and the QR scanner.

## Decision Logic

| QR Content | Target Bin | Color |
|------------|-----------|-------|
| `category_a` | Bin A | 🔴 Red |
| `category_b` | Bin B | 🔵 Blue |
| `category_c` / unknown / no QR | Bin C | 🟢 Green (default) |

## Troubleshooting

### Problem: All objects go to Bin C regardless of qr_data

**Cause:** Python QR libraries are missing.

```bash
# Check what's installed
pip3 list | grep -iE "pyzbar|opencv|qrcode|numpy"
```

If any of these are missing, install them:
```bash
pip3 install pyzbar opencv-python qrcode[pil] "numpy<2" --break-system-packages
sudo apt install -y libzbar0
```

Then **clean rebuild**:
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select pick_scan_place
source install/setup.bash
```

### Problem: "No executable found"

You forgot to source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

### Problem: "NumPy 2.x error" or "cv_bridge crash"

```bash
pip3 install "numpy<2" --break-system-packages
```

### Problem: Bins/table not visible in RViz

Add the MarkerArray display: **Add** → **By topic** → `/visualization_marker_array` → **OK**

### Problem: "Package 'moveit_resources_panda_moveit_config' not found"

```bash
sudo apt install -y ros-humble-moveit-resources-panda-moveit-config ros-humble-moveit-resources-panda-description
```

## Project Structure

pick_scan_place/

├── pick_scan_place/

│   ├── pick_place_node.py       # Main pick-scan-place orchestrator

│   ├── qr_scanner_node.py       # QR code detection and decoding

│   ├── qr_test_publisher.py     # Test QR image publisher

│   └── scene_setup_node.py      # Visual markers for RViz

├── launch/

│   └── pick_scan_place.launch.py # Single-command system launch

├── worlds/

│   └── pick_place.world

├── package.xml

├── setup.py

└── README.md


## Nodes Description

| Node | Description |
|------|-------------|
| `pick_scan_place_node` | Main orchestrator — executes pick, scan, place workflow |
| `qr_scanner_node` | Subscribes to camera images, decodes QR codes via pyzbar |
| `qr_test_publisher` | Publishes test QR code images for simulation |
| `scene_setup_node` | Adds colored markers (table, bins, scanner) to RViz |

## ROS 2 Topics & Actions

| Topic / Action | Type | Purpose |
|---------------|------|---------|
| `/move_action` | MoveGroup action | Send motion plans to MoveIt 2 |
| `/panda_hand_controller/gripper_cmd` | GripperCommand action | Open/close gripper |
| `/camera/image_raw` | sensor_msgs/Image | Camera feed for QR scanner |
| `/barcode` | std_msgs/String | Decoded QR data |
| `/visualization_marker_array` | MarkerArray | Visual scene markers |
| `/joint_states` | sensor_msgs/JointState | Robot joint positions |
| `/tf` | tf2_msgs/TFMessage | Coordinate transforms |

## Author
- **Course:** MAI605 — Robotic Systems
- **University:** Ajman University
- **Instructor:** Dr. Omar Shalash
