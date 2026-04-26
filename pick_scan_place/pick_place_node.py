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
    CollisionObject,
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

        self.object_state_pub = self.create_publisher(String, '/object_state', 10)

        self.collision_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10)
        
        
        self.bins = {
            'A': {'x': -0.20, 'y': -0.35, 'z': 0.32, 'name': 'Bin A (Red)'},
            'B': {'x': -0.35, 'y': -0.35, 'z': 0.32, 'name': 'Bin B (Blue)'},
            'C': {'x': -0.50, 'y': -0.35, 'z': 0.32, 'name': 'Bin C (Green)'},
        }


        self.get_logger().info('Waiting for servers...')
        self.move_client.wait_for_server()
        self.grip_client.wait_for_server()
        self.get_logger().info('Ready!')
        self.create_timer(3.0, self.go)

    def qr_cb(self, msg):
        self.qr_result = msg.data

    def publish_object_state(self, state):
        msg = String()
        msg.data = state
        self.object_state_pub.publish(msg)
        self.get_logger().info(f'Object state: {state}')

    def publish_object_state(self, state):
        msg = String()
        msg.data = state
        self.object_state_pub.publish(msg)
        self.get_logger().info(f'Object state: {state}')

    def add_collision_box(self, name, x, y, z, sx, sy, sz):
        obj = CollisionObject()
        obj.header.frame_id = 'panda_link0'
        obj.id = name

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(sx), float(sy), float(sz)]

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        self.collision_pub.publish(obj)
        self.get_logger().info(f'Added collision object: {name}')

    def add_collision_scene(self):
        # Table top
        self.add_collision_box('table_top', 0.45, 0.0, 0.2, 0.5, 0.4, 0.02)

        # Table legs
        self.add_collision_box('table_leg_1', 0.30, -0.15, 0.1, 0.04, 0.04, 0.2)
        self.add_collision_box('table_leg_2', 0.60, -0.15, 0.1, 0.04, 0.04, 0.2)
        self.add_collision_box('table_leg_3', 0.30, 0.15, 0.1, 0.04, 0.04, 0.2)
        self.add_collision_box('table_leg_4', 0.60, 0.15, 0.1, 0.04, 0.04, 0.2)

        # Scanner pole/camera
        self.add_collision_box('scanner_pole', 0.3, 0.58, 0.30, 0.03, 0.03, 0.60)
        self.add_collision_box('scanner_arm', 0.3, 0.52, 0.55, 0.03, 0.14, 0.03)
        self.add_collision_box('scanner_head', 0.3, 0.45, 0.55, 0.07, 0.04, 0.07)

        # Second table collision
        self.add_collision_box('second_table_top', -0.35, -0.35, 0.2, 0.55, 0.35, 0.02)
        self.add_collision_box('second_table_leg_1', -0.55, -0.48, 0.1, 0.04, 0.04, 0.2)
        self.add_collision_box('second_table_leg_2', -0.15, -0.48, 0.1, 0.04, 0.04, 0.2)
        self.add_collision_box('second_table_leg_3', -0.55, -0.22, 0.1, 0.04, 0.04, 0.2)
        self.add_collision_box('second_table_leg_4', -0.15, -0.22, 0.1, 0.04, 0.04, 0.2)

        # Bin walls simplified as obstacles
        #for key, b in self.bins.items():
            #x = b['x']
            #y = b['y']
            #self.add_collision_box(f'bin_{key}_base', x, y, 0.005, 0.15, 0.15, 0.01)
            #self.add_collision_box(f'bin_{key}_left', x - 0.075, y, 0.04, 0.008, 0.15, 0.075)
            #self.add_collision_box(f'bin_{key}_right', x + 0.075, y, 0.04, 0.008, 0.15, 0.075)
            #self.add_collision_box(f'bin_{key}_front', x, y - 0.075, 0.04, 0.15, 0.008, 0.075)
            #self.add_collision_box(f'bin_{key}_back', x, y + 0.075, 0.04, 0.15, 0.008, 0.075)

        
        # Bin walls simplified as obstacles (with opening on top)
        for key, b in self.bins.items():
            x = b['x']
            y = b['y']

            # Base (on table)
            self.add_collision_box(
                f'bin_{key}_base', x, y, 0.21, 0.15, 0.15, 0.01
            )

            # Side walls (shorter so gripper can enter from above)
            self.add_collision_box(
                f'bin_{key}_left', x - 0.075, y, 0.28, 0.008, 0.15, 0.12
            )
            self.add_collision_box(
                f'bin_{key}_right', x + 0.075, y, 0.28, 0.008, 0.15, 0.12
            )
            self.add_collision_box(
                f'bin_{key}_front', x, y - 0.075, 0.28, 0.15, 0.008, 0.12
            )
            self.add_collision_box(
                f'bin_{key}_back', x, y + 0.075, 0.28, 0.15, 0.008, 0.12
            )


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
        L.info('Adding MoveIt collision scene...')
        self.add_collision_scene()
        time.sleep(1.0)

        L.info('')
        L.info('====== PICK-SCAN-PLACE ======')

        # -- PICK --
        L.info('')
        L.info('-- PICK --')

        L.info('Open gripper')
        self.publish_object_state('table')
        self.grip(False)

        L.info('Go above object')
        self.move(0.4, 0.0, 0.55)    # safe transition
        self.move(0.5, 0.0, 0.50)    # above cube
        self.move(0.5, 0.0, 0.42)    # close to cube

        L.info('Grab object')
        self.grip(True)
        time.sleep(0.5)
        self.publish_object_state('attached')

        L.info('Lift up')
        self.move(0.5, 0.0, 0.50)    # lift cube with claw
        self.move(0.5, 0.0, 0.60)    # safe height

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
        self.move(0.3, 0.3, 0.75)    # lift up
        self.move(0.3, 0.0, 0.75)    # back to center
        
        # -- PLACE --
        L.info('')
        L.info(f'-- PLACE ({b["name"]}) --')

        bx, by, bz = b['x'], b['y'], b['z']

        L.info('Move to safe height')
        if not self.move(0.30, 0.15, 0.75):
            L.error('Failed to move to transition point')
            return

        L.info('Go above second table bin')
        if not self.move(bx, by, 0.75):
            L.error('Failed to reach above bin')
            return

        L.info('Lower close to bin')
        if not self.move(bx, by, 0.45):
            L.error('Failed to lower close to bin')
            return

        L.info('Release object inside bin')
        self.grip(False)
        self.publish_object_state(f'bin_{key}')

        L.info('Retreat')
        if not self.move(bx, by, 0.75):
            L.error('Failed to retreat from bin')
            return

        L.info('Return home')
        self.move(0.30, 0.15, 0.75)
        self.move(0.30, 0.00, 0.65)

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
