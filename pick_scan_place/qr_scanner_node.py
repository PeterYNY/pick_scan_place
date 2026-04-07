#!/usr/bin/env python3
"""
QR Scanner Node
===============
Subscribes to camera images from Gazebo, detects QR codes
using pyzbar, and publishes decoded data to /barcode topic.
Keeps publishing the last detected QR code periodically
so the pick-and-place node can catch it during scan phase.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
import cv2
import numpy as np


class QRScannerNode(Node):
    def __init__(self):
        super().__init__('qr_scanner_node')

        self.bridge = CvBridge()

        self.declare_parameter('camera_topic', '/camera/image_raw')
        camera_topic = self.get_parameter('camera_topic').value

        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        # Publish decoded QR data
        self.barcode_pub = self.create_publisher(
            String,
            '/barcode',
            10
        )

        # Store the latest detected QR code
        self.latest_qr = None
        self.scan_count = 0

        # Timer to republish latest QR every 0.5 seconds
        # This ensures the pick-and-place node catches it
        self.republish_timer = self.create_timer(0.5, self.republish_qr)

        self.get_logger().info(f'QR Scanner started, listening on {camera_topic}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            decoded_objects = decode(cv_image)

            for obj in decoded_objects:
                qr_data = obj.data.decode('utf-8')
                qr_type = obj.type

                if qr_data != self.latest_qr:
                    self.scan_count += 1
                    self.latest_qr = qr_data
                    self.get_logger().info(
                        f'[Scan #{self.scan_count}] Detected {qr_type}: "{qr_data}"'
                    )

                # Publish immediately on detection
                result_msg = String()
                result_msg.data = qr_data
                self.barcode_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def republish_qr(self):
        """
        Republish the latest QR code every 0.5 seconds.
        This way the pick-and-place node will receive it
        whenever it enters the scan phase.
        """
        if self.latest_qr is not None:
            result_msg = String()
            result_msg.data = self.latest_qr
            self.barcode_pub.publish(result_msg)


def main(args=None):
    rclpy.init(args=args)
    node = QRScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down QR Scanner...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
