#!/usr/bin/env python3
"""Scene Setup - Visual markers only."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import time


class SceneSetupNode(Node):
    def __init__(self):
        super().__init__('scene_setup_node')
        self.marker_pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', 10)
        self.mid = 0
        self.markers = MarkerArray()
        time.sleep(2.0)

        # Table
        self._box(0.5, 0.0, 0.2, 0.5, 0.6, 0.02, 0.55, 0.35, 0.17)
        self._box(0.3, -0.25, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)
        self._box(0.7, -0.25, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)
        self._box(0.3, 0.25, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)
        self._box(0.7, 0.25, 0.1, 0.04, 0.04, 0.2, 0.45, 0.28, 0.12)

        # Object
        self._box(0.5, 0.0, 0.25, 0.04, 0.04, 0.04, 1.0, 1.0, 1.0)

        # Bins aligned in a row at y=-0.4
        self._bin(0.5, -0.4, 1.0, 0.15, 0.15)
        self._label('Bin A', 0.5, -0.4, 0.12, 1.0, 0.3, 0.3)

        self._bin(0.3, -0.4, 0.15, 0.15, 1.0)
        self._label('Bin B', 0.3, -0.4, 0.12, 0.3, 0.3, 1.0)

        self._bin(0.1, -0.4, 0.15, 0.85, 0.15)
        self._label('Bin C', 0.1, -0.4, 0.12, 0.3, 1.0, 0.3)

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

        self.timer = self.create_timer(1.0, self._pub)
        self.get_logger().info('Scene ready!')

    def _pub(self):
        self.marker_pub.publish(self.markers)

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
        self._box(x, y, 0.005, s, s, 0.01, r, g, b, 0.9)
        self._box(x+s/2, y, 0.04, t, s, 0.075, r*0.7, g*0.7, b*0.7, 0.7)
        self._box(x-s/2, y, 0.04, t, s, 0.075, r*0.7, g*0.7, b*0.7, 0.7)
        self._box(x, y+s/2, 0.04, s, t, 0.075, r*0.7, g*0.7, b*0.7, 0.7)
        self._box(x, y-s/2, 0.04, s, t, 0.075, r*0.7, g*0.7, b*0.7, 0.7)

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
