#!/usr/bin/env python3
"""Scene Setup - Visual markers only."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
import time


class SceneSetupNode(Node):
    def __init__(self):
        super().__init__('scene_setup_node')

        self.marker_pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', 10)

        self.conveyor_state_pub = self.create_publisher(
            String, '/conveyor_state', 10)

        self.mid = 0
        self.markers = MarkerArray()

        # Conveyor simulation
        self.conveyor_x = 0.50
        self.conveyor_y = -0.32
        self.conveyor_speed = 0.01
        self.sensor_y = 0.00
        self.conveyor_running = True
        self.conveyor_ready_published = False

        # Object state (for moving cube)
        self.object_state = 'table'

        # Subscribe to object state
        self.create_subscription(
            String,
            '/object_state',
            self.object_state_cb,
            10
        )

        time.sleep(2.0)

        # First Table
        #self._box(0.45, 0.0, 0.2, 0.5, 0.4, 0.02, 0.55, 0.35, 0.17)
        # Table Legs
        #self._box(0.30, -0.15, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)
        #self._box(0.60, -0.15, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)
        #self._box(0.30, 0.15, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)
        #self._box(0.60, 0.15, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)

        # Conveyor system: feeder box + belt

        # Large feeder box / hopper reaching the floor
        self._box(0.50, -0.42, 0.20, 0.28, 0.22, 0.40, 0.35, 0.35, 0.35)
        self._label('Feeder', 0.50, -0.42, 0.43, 1.0, 1.0, 0.0)

        # Exit hole visual: starts from conveyor surface and goes upward
        self._box(0.50, -0.30, 0.255, 0.12, 0.01, 0.09, 0.0, 0.0, 0.0)

        # Conveyor belt surface
        self._box(0.50, -0.12, 0.20, 0.18, 0.55, 0.02, 0.03, 0.03, 0.03)

        # Low side rails: from slightly above belt down to ground
        self._box(0.40, -0.12, 0.115, 0.02, 0.55, 0.23, 0.45, 0.45, 0.45)
        self._box(0.60, -0.12, 0.115, 0.02, 0.55, 0.23, 0.45, 0.45, 0.45)

        # End roller/supports: at both ends of conveyor, from above belt to ground
        self._box(0.50, -0.395, 0.115, 0.20, 0.025, 0.23, 0.6, 0.6, 0.6)
        self._box(0.50, 0.155, 0.115, 0.20, 0.025, 0.23, 0.6, 0.6, 0.6)

        # Horizontal beam sensor embedded near the top of side rails

        # Left sensor emitter
        self._box(0.40, 0.00, 0.225, 0.02, 0.03, 0.03, 0.1, 0.1, 0.1)

        # Right sensor receiver
        self._box(0.60, 0.00, 0.225, 0.02, 0.03, 0.03, 0.1, 0.1, 0.1)

        # Laser beam slightly above conveyor belt surface
        self._box(0.50, 0.00, 0.225, 0.20, 0.004, 0.004, 1.0, 0.0, 0.0, 0.9)

        self._label('Beam Sensor', 0.50, 0.00, 0.30, 1.0, 0.0, 0.0)


        self._label('Conveyor Belt', 0.50, -0.12, 0.36, 1.0, 1.0, 0.0)

        # Second table on the opposite side of the robot
        self._box(-0.35, -0.35, 0.2, 0.55, 0.35, 0.02, 0.55, 0.35, 0.17)
        self._box(-0.55, -0.48, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)
        self._box(-0.15, -0.48, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)
        self._box(-0.55, -0.22, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)
        self._box(-0.15, -0.22, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)

        # Object
        #self._box(0.5, 0.0, 0.25, 0.04, 0.04, 0.04, 1.0, 1.0, 1.0)
        # Object marker will be updated dynamically
        self.object_marker_id = self.mid
        self._dynamic_object_marker()

        # Bins centered on second table
        self._bin(-0.20, -0.35, 1.0, 0.15, 0.15)
        self._label('Bin A', -0.20, -0.35, 0.35, 1.0, 0.3, 0.3)

        self._bin(-0.35, -0.35, 0.15, 0.15, 1.0)
        self._label('Bin B', -0.35, -0.35, 0.35, 0.3, 0.3, 1.0)

        self._bin(-0.50, -0.35, 0.15, 0.85, 0.15)
        self._label('Bin C', -0.50, -0.35, 0.35, 0.3, 1.0, 0.3)

        # Scan station - horizontal scanner facing the arm
        # Vertical pole
        self._box(0.3, 0.58, 0.30, 0.03, 0.03, 0.60, 0.4, 0.4, 0.4)
        # Horizontal arm connecting pole to camera
        self._box(0.3, 0.52, 0.55, 0.03, 0.14, 0.03, 0.4, 0.4, 0.4)
        # Camera housing
        self._box(0.3, 0.45, 0.55, 0.07, 0.04, 0.07, 0.2, 0.2, 0.2)
        # Camera lens (cyan, facing toward arm)
        self._box(0.3, 0.42, 0.55, 0.03, 0.01, 0.03, 0.0, 0.8, 1.0)
        # Small red laser dot
        self._box(0.3, 0.40, 0.55, 0.005, 0.005, 0.005, 1.0, 0.0, 0.0)
        # Base plate
        self._box(0.3, 0.58, 0.01, 0.12, 0.12, 0.02, 0.5, 0.5, 0.5)
        # Label
        self._label('QR Scanner', 0.3, 0.58, 0.65, 0.0, 0.8, 1.0)

        self.timer = self.create_timer(0.1, self._pub)
        self.get_logger().info('Scene ready!')

    def _pub(self):
        if self.object_state == 'table':
            self._dynamic_object_marker()

            if not self.conveyor_running and self.conveyor_ready_published:
                msg = String()
                msg.data = 'ready'
                self.conveyor_state_pub.publish(msg)

        self.marker_pub.publish(self.markers)


    def object_state_cb(self, msg):
        self.object_state = msg.data
        self._dynamic_object_marker()


    def _dynamic_object_marker(self):
        m = Marker()
        m.ns = 'dynamic_object'
        m.id = self.object_marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.scale = Vector3(x=0.04, y=0.04, z=0.04)
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        if self.object_state == 'attached':
            m.header.frame_id = 'panda_link8'
            m.pose.position = Point(x=0.0, y=0.0, z=0.08)

        elif self.object_state == 'bin_A':
            m.header.frame_id = 'panda_link0'
            m.pose.position = Point(x=-0.20, y=-0.35, z=0.28)

        elif self.object_state == 'bin_B':
            m.header.frame_id = 'panda_link0'
            m.pose.position = Point(x=-0.35, y=-0.35, z=0.28)

        elif self.object_state == 'bin_C':
            m.header.frame_id = 'panda_link0'
            m.pose.position = Point(x=-0.50, y=-0.35, z=0.28)

        else:
            # Conveyor movement along Y direction
            if self.conveyor_running:
                self.conveyor_y += self.conveyor_speed

                # Stop at sensor
                if self.conveyor_y >= self.sensor_y:
                    self.conveyor_y = self.sensor_y
                    self.conveyor_running = False

                    if not self.conveyor_ready_published:
                        msg = String()
                        msg.data = 'ready'
                        self.conveyor_state_pub.publish(msg)
                        self.get_logger().info('Conveyor sensor detected object: ready')
                        self.conveyor_ready_published = True

            m.header.frame_id = 'panda_link0'
            m.pose.position = Point(
                x=float(self.conveyor_x),
                y=float(self.conveyor_y),
                z=0.23
            )

        # Remove old marker
        self.markers.markers = [
            marker for marker in self.markers.markers
            if not (marker.ns == 'dynamic_object' and marker.id == self.object_marker_id)
        ]

        self.markers.markers.append(m)



    def _box(self, x, y, z, sx, sy, sz, r, g, b, a=1.0):
        m = Marker()
        m.header.frame_id = 'panda_link0'
        m.ns = 'scene'
        m.id = self.mid
        self.mid += 1
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position = Point(x=float(x), y=float(y), z=float(z))
        m.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        m.scale = Vector3(x=float(sx), y=float(sy), z=float(sz))
        m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
        self.markers.markers.append(m)

    def _cyl(self, x, y, z, d, h, r, g, b):
        m = Marker()
        m.header.frame_id = 'panda_link0'
        m.ns = 'scene'
        m.id = self.mid
        self.mid += 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position = Point(x=float(x), y=float(y), z=float(z))
        m.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        m.scale = Vector3(x=float(d), y=float(d), z=float(h))
        m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
        self.markers.markers.append(m)

    def _bin(self, x, y, r, g, b):
        s = 0.15
        t = 0.008
        table_top_z = 0.21
        self._box(x, y, table_top_z, s, s, 0.01, r, g, b, 0.9)
        self._box(x+s/2, y, table_top_z + 0.04, t, s, 0.075, r*0.7, g*0.7, b*0.7, 0.7)
        self._box(x-s/2, y, table_top_z + 0.04, t, s, 0.075, r*0.7, g*0.7, b*0.7, 0.7)
        self._box(x, y+s/2, table_top_z + 0.04, s, t, 0.075, r*0.7, g*0.7, b*0.7, 0.7)
        self._box(x, y-s/2, table_top_z + 0.04, s, t, 0.075, r*0.7, g*0.7, b*0.7, 0.7)

    def _label(self, text, x, y, z, r, g, b):
        m = Marker()
        m.header.frame_id = 'panda_link0'
        m.ns = 'labels'
        m.id = self.mid
        self.mid += 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position = Point(x=float(x), y=float(y), z=float(z))
        m.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        m.scale.z = 0.03
        m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
        m.text = text
        self.markers.markers.append(m)


def main(args=None):
    rclpy.init(args=args)
    node = SceneSetupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
