#!/usr/bin/env python3
"""
Pick-Scan-Place ROS 2 node.

This node controls a Franka Panda robot to perform a complete workflow:
1. Wait until the conveyor reports that an object is ready.
2. Pick the object from the conveyor/table area.
3. Move the object to the QR scanner and read its barcode.
4. Select the correct bin based on the QR result.
5. Place the object inside the selected bin and return to a safe home position.

The code uses MoveIt 2 for arm motion planning and the Panda gripper action
server for opening and closing the gripper.
"""

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


class PickScanPlaceNode(Node):
    """Main ROS 2 node that manages the pick, scan, and place sequence."""

    def __init__(self):
        """Initialize ROS clients, publishers, subscribers, bin positions, and timer."""
        super().__init__('pick_scan_place_node')

        # Reentrant callback group allows callbacks/actions to run without blocking each other.
        self.cb_group = ReentrantCallbackGroup()

        # Action client used to send motion planning goals to MoveIt 2.
        self.move_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cb_group)

        # Action client used to control the Panda gripper open/close movement.
        self.grip_client = ActionClient(
            self, GripperCommand, '/panda_hand_controller/gripper_cmd',
            callback_group=self.cb_group)

        # Stores the latest QR/barcode result received from the scanner node.
        self.qr_result = None

        # Becomes True when the conveyor publishes that the object is ready.
        self.conveyor_ready = False

        # Subscribe to QR scan results and conveyor status messages.
        self.create_subscription(String, '/barcode', self.qr_cb, 10)
        self.create_subscription(String, '/conveyor_state', self.conveyor_cb, 10)

        # Publishes the simulated object state so the visualization can update the cube position.
        self.object_state_pub = self.create_publisher(String, '/object_state', 10)

        # Publishes collision objects to MoveIt so the robot avoids tables, bins, and sensors.
        self.collision_pub = self.create_publisher(
            CollisionObject, '/collision_object', 100)

        # Bin target positions. The key is selected from the QR result.
        self.bins = {
            'A': {'x': -0.20, 'y': -0.35, 'z': 0.32, 'name': 'Bin A (Red)'},
            'B': {'x': -0.35, 'y': -0.35, 'z': 0.32, 'name': 'Bin B (Blue)'},
            'C': {'x': -0.50, 'y': -0.35, 'z': 0.32, 'name': 'Bin C (Green)'},
        }

        # Wait until MoveIt and gripper action servers are available before starting.
        self.get_logger().info('Waiting for servers...')
        self.move_client.wait_for_server()
        self.grip_client.wait_for_server()
        self.get_logger().info('Ready!')

        # Start the main workflow once after a short delay.
        self.create_timer(3.0, self.go)

    def qr_cb(self, msg):
        """Save the latest QR/barcode result received from the scanner topic."""
        self.qr_result = msg.data

    def conveyor_cb(self, msg):
        """Detect when the conveyor has stopped and the object is ready for pickup."""
        if msg.data == 'ready' and not self.conveyor_ready:
            self.conveyor_ready = True
            self.get_logger().info('Conveyor ready signal received')

    def publish_object_state(self, state):
        """
        Publish the current object state for visualization.

        Example states:
        - 'table': object is on the table/conveyor area.
        - 'attached': object should follow the gripper.
        - 'bin_A', 'bin_B', 'bin_C': object has been released in a bin.
        """
        msg = String()
        msg.data = state
        self.object_state_pub.publish(msg)
        self.get_logger().info(f'Object state: {state}')

    def add_collision_box(self, name, x, y, z, sx, sy, sz):
        """
        Add a box-shaped obstacle to the MoveIt collision scene.

        Parameters:
        - name: unique object name in the planning scene.
        - x, y, z: center position of the box in the panda_link0 frame.
        - sx, sy, sz: box size along x, y, and z.
        """
        obj = CollisionObject()
        obj.header.frame_id = 'panda_link0'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = name

        # Define the obstacle geometry as a simple box.
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(sx), float(sy), float(sz)]

        # Define where the box is placed in the robot base frame.
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0

        # Attach the shape and pose to the collision object message.
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        # Publish the object to MoveIt and wait briefly to let the scene update.
        self.collision_pub.publish(obj)
        time.sleep(0.05)
        self.get_logger().info(f'Added collision object: {name}')

    def add_collision_scene(self):
        """Create all fixed obstacles used by MoveIt during path planning."""

        # Conveyor components. These prevent the arm from planning through the conveyor model.
        self.add_collision_box('conveyor_feeder', 0.50, -0.42, 0.20, 0.28, 0.22, 0.40)
        self.add_collision_box('conveyor_belt', 0.50, -0.12, 0.20, 0.18, 0.55, 0.02)
        self.add_collision_box('conveyor_left_rail', 0.40, -0.12, 0.115, 0.02, 0.55, 0.23)
        self.add_collision_box('conveyor_right_rail', 0.60, -0.12, 0.115, 0.02, 0.55, 0.23)
        self.add_collision_box('conveyor_front_support', 0.50, -0.395, 0.115, 0.20, 0.025, 0.23)
        self.add_collision_box('conveyor_back_support', 0.50, 0.155, 0.115, 0.20, 0.025, 0.23)

        # Only the physical sensor parts are added as obstacles; the laser beam itself has no collision.
        self.add_collision_box('beam_sensor_left', 0.40, 0.00, 0.225, 0.02, 0.03, 0.03)
        self.add_collision_box('beam_sensor_right', 0.60, 0.00, 0.225, 0.02, 0.03, 0.03)

        # QR scanner structure.
        self.add_collision_box('scanner_pole', 0.3, 0.58, 0.30, 0.03, 0.03, 0.60)
        self.add_collision_box('scanner_arm', 0.3, 0.52, 0.55, 0.03, 0.14, 0.03)
        self.add_collision_box('scanner_head', 0.3, 0.45, 0.55, 0.07, 0.04, 0.07)

        # Second table that supports the bins.
        self.add_collision_box('second_table_top', -0.35, -0.35, 0.2, 0.55, 0.35, 0.02)
        self.add_collision_box('second_table_leg_1', -0.55, -0.48, 0.1, 0.04, 0.04, 0.2)
        self.add_collision_box('second_table_leg_2', -0.15, -0.48, 0.1, 0.04, 0.04, 0.2)
        self.add_collision_box('second_table_leg_3', -0.55, -0.22, 0.1, 0.04, 0.04, 0.2)
        self.add_collision_box('second_table_leg_4', -0.15, -0.22, 0.1, 0.04, 0.04, 0.2)

        # Add bin bases and walls as obstacles so the gripper avoids hitting the bin edges.
        for key, b in self.bins.items():
            x = b['x']
            y = b['y']

            self.add_collision_box(
                f'bin_{key}_base', x, y, 0.21, 0.15, 0.15, 0.01
            )
            self.add_collision_box(
                f'bin_{key}_left', x - 0.075, y, 0.25, 0.008, 0.15, 0.075
            )
            self.add_collision_box(
                f'bin_{key}_right', x + 0.075, y, 0.25, 0.008, 0.15, 0.075
            )
            self.add_collision_box(
                f'bin_{key}_front', x, y - 0.075, 0.25, 0.15, 0.008, 0.075
            )
            self.add_collision_box(
                f'bin_{key}_back', x, y + 0.075, 0.25, 0.15, 0.008, 0.075
            )

    def move(self, x, y, z):
        """
        Move the Panda end effector to a target Cartesian position.

        MoveIt receives a goal with:
        - a small spherical position constraint around the target point,
        - an orientation constraint to keep the gripper in a stable pose,
        - replanning enabled to improve the chance of finding a valid path.

        Returns True if MoveIt reports a successful execution, otherwise False.
        """
        req = MotionPlanRequest()
        req.group_name = 'panda_arm'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0

        # Position constraint: the end effector must reach this small target region.
        pos = PositionConstraint()
        pos.header.frame_id = 'panda_link0'
        pos.link_name = 'panda_link8'

        bv = BoundingVolume()
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.01]

        p = Pose()
        p.position = Point(x=float(x), y=float(y), z=float(z))
        p.orientation = Quaternion(x=0.9238795, y=-0.3826834, z=0.0, w=0.0)

        bv.primitives.append(s)
        bv.primitive_poses.append(p)
        pos.constraint_region = bv
        pos.weight = 1.0

        # Orientation constraint: keeps the gripper angle consistent during the motion.
        ori = OrientationConstraint()
        ori.header.frame_id = 'panda_link0'
        ori.link_name = 'panda_link8'
        ori.orientation = Quaternion(x=0.9238795, y=-0.3826834, z=0.0, w=0.0)
        ori.absolute_x_axis_tolerance = 0.5
        ori.absolute_y_axis_tolerance = 0.5
        ori.absolute_z_axis_tolerance = 3.14
        ori.weight = 1.0

        # Combine position and orientation constraints into one MoveIt goal.
        c = Constraints()
        c.position_constraints.append(pos)
        c.orientation_constraints.append(ori)
        req.goal_constraints.append(c)

        # Configure MoveIt to execute the path, not only plan it.
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        # Send the goal and wait until MoveIt accepts it.
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            return False

        # Wait for the motion execution result.
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        success = result_future.result().result.error_code.val == 1
        if success:
            self.get_logger().info(f'  Reached ({x:.2f}, {y:.2f}, {z:.2f})')
        return success

    def grip(self, close):
        """
        Open or close the Panda gripper.

        close=True  -> gripper closes to hold the cube.
        close=False -> gripper opens to release the cube.
        """
        goal = GripperCommand.Goal()
        goal.command.position = 0.0 if close else 0.04
        goal.command.max_effort = 50.0

        future = self.grip_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

    def go(self, _=None):
        """Run the full pick-scan-place sequence once."""

        # The timer calls this method repeatedly, so this flag makes sure it only runs once.
        if hasattr(self, '_done'):
            return
        self._done = True

        logger = self.get_logger()

        # Build the planning scene before any robot movement starts.
        logger.info('Adding MoveIt collision scene...')
        self.add_collision_scene()
        time.sleep(1.0)

        logger.info('')
        logger.info('====== PICK-SCAN-PLACE ======')

        # Wait until the conveyor node says the object is positioned for pickup.
        logger.info('Waiting for conveyor sensor...')
        while not self.conveyor_ready:
            rclpy.spin_once(self, timeout_sec=0.2)

        logger.info('Conveyor stopped. Object ready for pickup.')

        # ------------------------------------------------------------------
        # PICK PHASE
        # ------------------------------------------------------------------
        logger.info('')
        logger.info('-- PICK --')

        logger.info('Open gripper')
        self.publish_object_state('table')
        self.grip(False)

        # Move through safe approach points before reaching the pickup height.
        logger.info('Go above object')
        if not self.move(0.4, 0.0, 0.55):
            logger.error('Failed to reach safe transition before pickup')
            return

        if not self.move(0.5, 0.0, 0.38):
            logger.error('Failed to reach above object')
            return

        if not self.move(0.5, 0.0, 0.34):
            logger.error('Failed to reach pickup height')
            return

        # Close the gripper and mark the cube as attached in the visualization.
        logger.info('Grab object')
        self.grip(True)
        time.sleep(0.5)
        self.publish_object_state('attached')

        # Lift the object to a safe height before moving to the scanner.
        logger.info('Lift up')
        self.move(0.5, 0.0, 0.50)
        self.move(0.5, 0.0, 0.60)

        # ------------------------------------------------------------------
        # SCAN PHASE
        # ------------------------------------------------------------------
        logger.info('')
        logger.info('-- SCAN --')

        # Move to the scanner using intermediate points to avoid obstacles.
        logger.info('Go to scan station')
        self.move(0.4, 0.15, 0.5)
        self.move(0.3, 0.3, 0.5)
        self.move(0.3, 0.3, 0.66)

        # Give the scanner time to read the QR code.
        logger.info('Scanning QR...')
        logger.info('Scanning QR in 2 seconds...')
        time.sleep(2.0)

        # Wait up to 2 seconds for a QR result to arrive from /barcode.
        start_time = time.time()
        while self.qr_result is None and (time.time() - start_time) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.2)

        qr = self.qr_result
        if qr:
            logger.info(f'  Detected: {qr}')
        else:
            logger.warn('  No QR, default bin')

        # Select destination bin using the QR suffix.
        # Example: item_a -> Bin A, item_b -> Bin B, anything else -> Bin C.
        if qr and qr.strip().lower().endswith('_a'):
            key = 'A'
        elif qr and qr.strip().lower().endswith('_b'):
            key = 'B'
        else:
            key = 'C'

        bin_target = self.bins[key]
        logger.info(f'  -> {bin_target["name"]}')

        # Leave the scanner area from a higher position for safety.
        logger.info('Leave scan station')
        self.move(0.3, 0.3, 0.75)
        self.move(0.3, 0.0, 0.75)

        # ------------------------------------------------------------------
        # PLACE PHASE
        # ------------------------------------------------------------------
        logger.info('')
        logger.info(f'-- PLACE ({bin_target["name"]}) --')

        bx, by, _ = bin_target['x'], bin_target['y'], bin_target['z']

        # Move above the selected bin, lower into it, then release the object.
        logger.info('Move to safe height')
        if not self.move(0.30, 0.15, 0.75):
            logger.error('Failed to move to transition point')
            return

        logger.info('Go above table bin')
        if not self.move(bx, by, 0.75):
            logger.error('Failed to reach above bin')
            return

        logger.info('Lower into bin')
        if not self.move(bx, by, 0.42):
            logger.error('Failed to lower into bin')
            return

        # Open the gripper and update the visualization state after releasing the object.
        logger.info('Release object inside bin')
        self.grip(False)
        time.sleep(0.5)
        self.publish_object_state(f'bin_{key}')

        # Move upward before returning home to avoid collision with bin walls.
        logger.info('Retreat')
        if not self.move(bx, by, 0.75):
            logger.error('Failed to retreat from bin')
            return

        # Return to a safe resting position.
        logger.info('Return home')
        self.move(0.30, 0.15, 0.75)
        self.move(0.30, 0.00, 0.65)

        logger.info('')
        logger.info('====== DONE ======')
        logger.info(f'Object in {bin_target["name"]}')
        logger.info('==================')

        rclpy.shutdown()


def main(args=None):
    """Initialize ROS, start the node, and keep it alive until shutdown."""
    rclpy.init(args=args)
    node = PickScanPlaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
