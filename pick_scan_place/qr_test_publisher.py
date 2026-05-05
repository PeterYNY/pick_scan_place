#!/usr/bin/env python3
"""
QR Test Publisher Node
======================
This node creates a fake QR code image and publishes it to a ROS 2 image topic.

It is mainly used for testing the QR scanner and pick-scan-place workflow without
needing a real camera or a physical QR code in the simulation.

The QR content can be changed using the `qr_data` parameter. For example:
- category_a -> should send the object to Bin A
- category_b -> should send the object to Bin B
- any other value -> usually sends the object to the default Bin C
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import qrcode


class QRTestPublisher(Node):
    """
    ROS 2 node responsible for generating and publishing a QR code image.

    The published image simulates the output of a camera. This allows the
    QR scanner node to receive an image, decode the QR content, and publish
    the detected value to the rest of the system.
    """

    def __init__(self):
        super().__init__('qr_test_publisher')

        # Declare configurable parameters so they can be changed from the launch file
        # or terminal without modifying the source code.
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('qr_data', 'category_a')

        # Read the parameter values.
        # camera_topic: topic where the generated QR image will be published.
        # qr_data: text encoded inside the QR code.
        topic = self.get_parameter('camera_topic').value
        self.qr_data = self.get_parameter('qr_data').value

        # Publisher that sends the generated QR image as a ROS Image message.
        self.publisher = self.create_publisher(Image, topic, 10)

        # CvBridge converts between OpenCV/numpy image format and ROS Image messages.
        self.bridge = CvBridge()

        # Publish the QR code repeatedly every 0.5 seconds.
        # Repeating the image makes testing easier because the scanner node can
        # detect the QR code whenever it starts or enters the scan phase.
        self.timer = self.create_timer(0.5, self.publish_qr)

        self.get_logger().info(f'Publishing QR "{self.qr_data}" to {topic}')

    def publish_qr(self):
        """
        Generate a QR code image and publish it to the camera topic.

        Steps:
        1. Create a QR code object.
        2. Encode the selected text inside the QR code.
        3. Convert the QR image to a numpy RGB image.
        4. Convert the numpy image to a ROS Image message.
        5. Publish it to the configured camera topic.
        """

        # Create the QR code structure.
        # version=1 creates a small QR code.
        # box_size controls the pixel size of each QR square.
        # border adds white padding around the QR code for easier detection.
        qr = qrcode.QRCode(version=1, box_size=10, border=5)

        # Add the configured text to the QR code.
        qr.add_data(self.qr_data)
        qr.make(fit=True)

        # Render the QR code as a black and white image.
        img = qr.make_image(fill_color="black", back_color="white")

        # Convert the PIL image to a numpy RGB array.
        # This format is suitable for conversion to a ROS Image message.
        img_array = np.array(img.convert('RGB'))

        # Convert the numpy image to a ROS Image message using CvBridge.
        ros_image = self.bridge.cv2_to_imgmsg(img_array, encoding='rgb8')

        # Publish the QR image so the scanner node can receive and decode it.
        self.publisher.publish(ros_image)


def main(args=None):
    """
    Initialize ROS 2, start the QR test publisher node, and keep it running.
    """
    rclpy.init(args=args)
    node = QRTestPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allows the node to close cleanly when the user presses Ctrl+C.
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
