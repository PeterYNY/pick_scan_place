#!/usr/bin/env python3
"""Pick-Scan-Place - Simple realistic movement."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, PlanningOptions, Constraints,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Quaternion, Pose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
import time


class PickScanPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_scan_place_node')
        self.cb_group = ReentrantCallbackGroup()

        self.move_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cb_group)
        self.grip_client = ActionClient(
            self, GripperCommand, '/panda_hand_controller/gripper_cmd',
            callback_group=self.cb_group)

        self.qr_result = None
        self.create_subscription(String, '/barcode', self.qr_cb, 10)

        self.bins = {
            'A': {'x': 0.5, 'y': -0.4, 'z': 0.15, 'name': 'Bin A (Red)'},
            'B': {'x': 0.3, 'y': -0.4, 'z': 0.15, 'name': 'Bin B (Blue)'},
            'C': {'x': 0.1, 'y': -0.4, 'z': 0.15, 'name': 'Bin C (Green)'},
        }

        self.get_logger().info('Waiting for servers...')
        self.move_client.wait_for_server()
        self.grip_client.wait_for_server()
        self.get_logger().info('Ready!')
        self.create_timer(3.0, self.go)

    def qr_cb(self, msg):
        self.qr_result = msg.data

    def move(self, x, y, z):
        req = MotionPlanRequest()
        req.group_name = 'panda_arm'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0

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

        ori = OrientationConstraint()
        ori.header.frame_id = 'panda_link0'
        ori.link_name = 'panda_link8'
        ori.orientation = Quaternion(x=0.9238795, y=-0.3826834, z=0.0, w=0.0)
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        ori.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pos)
        c.orientation_constraints.append(ori)
        req.goal_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        f = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        ok = rf.result().result.error_code.val == 1
        if ok:
            self.get_logger().info(f'  Reached ({x:.2f}, {y:.2f}, {z:.2f})')
        return ok

    def grip(self, close):
        g = GripperCommand.Goal()
        g.command.position = 0.0 if close else 0.04
        g.command.max_effort = 50.0
        f = self.grip_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if gh.accepted:
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self, rf)

    def go(self, _=None):
        if hasattr(self, '_done'):
            return
        self._done = True
        L = self.get_logger()

        L.info('')
        L.info('====== PICK-SCAN-PLACE ======')

        # -- PICK --
        L.info('')
        L.info('-- PICK --')

        L.info('Open gripper')
        self.grip(False)

        # Start high, go above object, go down, grab, go back up
        L.info('Go above object')
        self.move(0.4, 0.0, 0.5)    # safe height
        self.move(0.5, 0.0, 0.4)    # above object
        self.move(0.5, 0.0, 0.30)   # at object (above table z=0.2)

        L.info('Grab object')
        self.grip(True)

        L.info('Lift up')
        self.move(0.5, 0.0, 0.4)    # lift
        self.move(0.5, 0.0, 0.5)    # safe height

        # -- SCAN --
        L.info('')
        L.info('-- SCAN --')

        # Move to scan station through safe height
        L.info('Go to scan station')
        self.move(0.4, 0.15, 0.5)   # transition
        self.move(0.3, 0.3, 0.5)    # above scan
        self.move(0.3, 0.3, 0.66)   # at scan station (aligned with scanner)

        L.info('Scanning QR...')
        # self.qr_result = None  # DO NOT reset - keep received QR
        t0 = time.time()
        while self.qr_result is None and (time.time() - t0) < 8.0:
            rclpy.spin_once(self, timeout_sec=0.5)
            L.info(f'  waiting... ({time.time()-t0:.0f}s)')

        qr = self.qr_result
        if qr:
            L.info(f'  Detected: {qr}')
        else:
            L.warn('  No QR, default bin')

        # Decide bin based on QR content
        if qr and qr.strip().lower().endswith('_a'):
            key = 'A'
        elif qr and qr.strip().lower().endswith('_b'):
            key = 'B'
        else:
            key = 'C'
        b = self.bins[key]
        L.info(f'  -> {b["name"]}')

        # Lift from scan
        L.info('Leave scan station')
        self.move(0.3, 0.3, 0.5)    # lift up
        self.move(0.3, 0.0, 0.5)    # back to center

        # -- PLACE --
        L.info('')
        L.info(f'-- PLACE ({b["name"]}) --')

        bx, by, bz = b['x'], b['y'], b['z']

        L.info('Go above bin')
        self.move(bx, 0.0, 0.5)     # transition above
        self.move(bx, by, 0.5)      # above bin
        self.move(bx, by, 0.35)     # lower toward bin
        self.move(bx, by, bz)       # in bin

        L.info('Release object')
        self.grip(False)

        L.info('Retreat')
        self.move(bx, by, 0.35)     # up from bin
        self.move(bx, by, 0.5)      # safe height

        L.info('Return home')
        self.move(0.3, 0.0, 0.5)
        self.move(0.3, 0.0, 0.6)

        L.info('')
        L.info('====== DONE ======')
        L.info(f'Object in {b["name"]}')
        L.info('==================')


def main(args=None):
    rclpy.init(args=args)
    n = PickScanPlaceNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
