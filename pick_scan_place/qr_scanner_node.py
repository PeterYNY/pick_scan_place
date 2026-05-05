#!/usr/bin/env python3
"""
QR Scanner Node
===============

This ROS 2 node receives camera images from Gazebo, searches each image for
QR codes using pyzbar, and publishes the decoded QR value on the `/barcode`
topic.

The node also republishes the last detected QR code every 0.5 seconds. This is
important because the pick-scan-place node may only start listening during the
scan phase, so repeating the last value makes the communication more reliable.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode

# cv2 and numpy are not directly used in this file, but they are commonly kept
# available for future image-processing/debugging extensions.
import cv2
import numpy as np


class QRScannerNode(Node):
    """ROS 2 node responsible for detecting and publishing QR code values."""

    def __init__(self):
        """Initialize the QR scanner node, subscribers, publishers, and timer."""
        super().__init__('qr_scanner_node')

        # CvBridge converts ROS Image messages into OpenCV images that pyzbar can read.
        self.bridge = CvBridge()

        # The camera topic can be changed from the launch file or command line.
        # By default, it listens to the Gazebo camera image topic.
        self.declare_parameter('camera_topic', '/camera/image_raw')
        camera_topic = self.get_parameter('camera_topic').value

        # Subscribe to the camera stream. Every received image is sent to image_callback().
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        # Publisher used to send the decoded QR text to the pick-scan-place node.
        self.barcode_pub = self.create_publisher(
            String,
            '/barcode',
            10
        )

        # latest_qr stores the last QR value detected by the camera.
        # scan_count is only used for clearer logging/debugging.
        self.latest_qr = None
        self.scan_count = 0

        # Republish the latest QR value periodically so that other nodes can still
        # receive it even if they were not ready at the exact detection moment.
        self.republish_timer = self.create_timer(0.5, self.republish_qr)

        self.get_logger().info(f'QR Scanner started, listening on {camera_topic}')

    def image_callback(self, msg):
        """
        Process each camera frame and publish any QR code detected.

        Steps:
        1. Convert the ROS Image message into an OpenCV image.
        2. Use pyzbar to detect and decode QR/barcode objects.
        3. Save and publish the decoded QR value.
        """
        try:
            # Convert from ROS image format to OpenCV BGR format.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Decode all QR/barcode objects visible in the current frame.
            decoded_objects = decode(cv_image)

            for obj in decoded_objects:
                # Convert QR bytes into a readable string.
                qr_data = obj.data.decode('utf-8')
                qr_type = obj.type

                # Log only when a new QR value is detected to avoid repeated messages.
                if qr_data != self.latest_qr:
                    self.scan_count += 1
                    self.latest_qr = qr_data
                    self.get_logger().info(
                        f'[Scan #{self.scan_count}] Detected {qr_type}: "{qr_data}"'
                    )

                # Publish the QR value immediately after detecting it.
                result_msg = String()
                result_msg.data = qr_data
                self.barcode_pub.publish(result_msg)

        except Exception as e:
            # Log image-processing errors without crashing the whole ROS node.
            self.get_logger().error(f'Error processing image: {str(e)}')

    def republish_qr(self):
        """
        Republish the last detected QR code every 0.5 seconds.

        This supports synchronization with the pick-scan-place node. If the robot
        reaches the scan phase after the QR was first detected, it can still
        receive the latest QR value from this repeated publication.
        """
        if self.latest_qr is not None:
            result_msg = String()
            result_msg.data = self.latest_qr
            self.barcode_pub.publish(result_msg)


def main(args=None):
    """Start the QR scanner node and keep it running until shutdown."""
    rclpy.init(args=args)
    node = QRScannerNode()

    try:
        # Keep the node alive so it can continue receiving camera frames.
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down QR Scanner...')
    finally:
        # Cleanly destroy the node and shut down ROS 2.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
