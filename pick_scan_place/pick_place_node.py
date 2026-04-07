#!/usr/bin/env python3
"""
Pick-Scan-Place Node for Panda Robot
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
import time


class PickScanPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_scan_place_node')
        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter('planning_group', 'panda_arm')
        self.planning_group = self.get_parameter('planning_group').value

        # Define positions
        self.pick_pose = self._make_pose(0.4, 0.0, 0.3)
        self.scan_pose = self._make_pose(0.3, 0.2, 0.5)
        self.bin_poses = {
            'A': self._make_pose(0.4, -0.3, 0.3),
            'B': self._make_pose(-0.4, -0.3, 0.3),
            'C': self._make_pose(0.0, -0.4, 0.3),
        }

        # Action clients
        self.move_action_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cb_group)
        self.gripper_client = ActionClient(
            self, GripperCommand, '/panda_hand_controller/gripper_cmd',
            callback_group=self.cb_group)

        # QR subscriber
        self.qr_result = None
        self.qr_sub = self.create_subscription(
            String, '/barcode', self.qr_callback, 10)

        self.get_logger().info('=== Pick-Scan-Place Node Started ===')
        self.get_logger().info('Waiting for MoveIt 2...')
        self.move_action_client.wait_for_server()
        self.get_logger().info('MoveIt 2 is ready!')
        self.gripper_client.wait_for_server()
        self.get_logger().info('Gripper is ready!')
        self.timer = self.create_timer(3.0, self.run_workflow)

    def _make_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = 'panda_link0'
        pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        pose.pose.orientation = Quaternion(x=0.9238795, y=-0.3826834, z=0.0, w=0.0)
        return pose

    def qr_callback(self, msg):
        self.qr_result = msg.data
        self.get_logger().info(f'QR Code received: {self.qr_result}')

    def decide_bin(self, qr_data):
        if qr_data is None:
            self.get_logger().warn('No QR data - defaulting to Bin C')
            return 'C'
        data = qr_data.strip().upper()
        self.get_logger().info(f'Processing QR data: "{data}"')
        if 'A' in data or 'CATEGORY_A' in data:
            return 'A'
        elif 'B' in data or 'CATEGORY_B' in data:
            return 'B'
        else:
            return 'C'

    def send_move_goal(self, target_pose):
        motion_plan_req = MotionPlanRequest()
        motion_plan_req.group_name = self.planning_group
        motion_plan_req.num_planning_attempts = 10
        motion_plan_req.allowed_planning_time = 5.0

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target_pose.header.frame_id
        position_constraint.link_name = 'panda_link8'
        bounding_volume = BoundingVolume()
        solid = SolidPrimitive()
        solid.type = SolidPrimitive.SPHERE
        solid.dimensions = [0.01]
        bounding_volume.primitives.append(solid)
        bounding_volume.primitive_poses.append(target_pose.pose)
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target_pose.header.frame_id
        orientation_constraint.link_name = 'panda_link8'
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        motion_plan_req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = motion_plan_req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info('  Sending motion plan to MoveIt 2...')
        future = self.move_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('  Motion plan REJECTED!')
            return False
        self.get_logger().info('  Plan accepted, executing...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.result.error_code.val == 1:
            self.get_logger().info('  Motion completed successfully!')
            return True
        else:
            self.get_logger().error(f'  Motion failed! Code: {result.result.error_code.val}')
            return False

    def control_gripper(self, open_gripper=True):
        goal = GripperCommand.Goal()
        goal.command.position = 0.04 if open_gripper else 0.0
        goal.command.max_effort = 50.0
        action = 'Opening' if open_gripper else 'Closing'
        self.get_logger().info(f'  {action} gripper...')
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info('  Gripper command done!')

    def wait_for_qr(self, timeout=10.0):
        if self.qr_result is not None:
            self.get_logger().info(f'  QR already available: {self.qr_result}')
            return self.qr_result
        start = time.time()
        while self.qr_result is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().info(f'  Waiting for QR... ({time.time()-start:.1f}s)')
        return self.qr_result

    def run_workflow(self):
        self.timer.cancel()
        self.get_logger().info('')
        self.get_logger().info('=== STARTING PICK-SCAN-PLACE WORKFLOW ===')

        # PHASE 1: PICK
        self.get_logger().info('-- Phase 1: PICK --')
        self.control_gripper(open_gripper=True)
        time.sleep(1.0)

        approach = self._make_pose(0.4, 0.0, 0.45)
        self.get_logger().info('Moving above pick position...')
        self.send_move_goal(approach)
        time.sleep(0.5)

        self.get_logger().info('Moving down to grasp...')
        self.send_move_goal(self.pick_pose)
        time.sleep(0.5)

        self.get_logger().info('Grasping object...')
        self.control_gripper(open_gripper=False)
        time.sleep(1.0)

        self.get_logger().info('Lifting object...')
        self.send_move_goal(approach)
        time.sleep(0.5)

        # PHASE 2: SCAN
        self.get_logger().info('')
        self.get_logger().info('-- Phase 2: SCAN --')
        self.get_logger().info('Moving to scan position...')
        self.send_move_goal(self.scan_pose)
        time.sleep(1.0)

        self.get_logger().info('Scanning QR code...')
        qr_data = self.wait_for_qr(timeout=10.0)
        target_bin = self.decide_bin(qr_data)
        self.get_logger().info(f'Decision: Object goes to Bin {target_bin}')

        # PHASE 3: PLACE
        self.get_logger().info('')
        self.get_logger().info(f'-- Phase 3: PLACE (Bin {target_bin}) --')
        bin_pose = self.bin_poses[target_bin]
        above_bin = self._make_pose(
            bin_pose.pose.position.x,
            bin_pose.pose.position.y,
            bin_pose.pose.position.z + 0.15)

        self.get_logger().info(f'Moving above Bin {target_bin}...')
        self.send_move_goal(above_bin)
        time.sleep(0.5)

        self.get_logger().info('Lowering into bin...')
        self.send_move_goal(bin_pose)
        time.sleep(0.5)

        self.get_logger().info('Releasing object...')
        self.control_gripper(open_gripper=True)
        time.sleep(1.0)

        self.get_logger().info('Retreating...')
        self.send_move_goal(above_bin)

        self.get_logger().info('')
        self.get_logger().info('=== WORKFLOW COMPLETE! ===')


def main(args=None):
    rclpy.init(args=args)
    node = PickScanPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
