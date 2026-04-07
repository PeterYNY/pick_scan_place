#!/usr/bin/env python3
"""
Publishes a QR code image to the camera topic for testing.
In a real system, the Gazebo camera would see an actual QR code on the object.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import qrcode


class QRTestPublisher(Node):
    def __init__(self):
        super().__init__('qr_test_publisher')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        topic = self.get_parameter('camera_topic').value
        self.publisher = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.5, self.publish_qr)
        self.get_logger().info(f'Publishing QR images to {topic}')

    def publish_qr(self):
        qr = qrcode.QRCode(version=1, box_size=10, border=5)
        qr.add_data('category_a')
        qr.make(fit=True)
        img = qr.make_image(fill_color="black", back_color="white")
        img_array = np.array(img.convert('RGB'))
        ros_image = self.bridge.cv2_to_imgmsg(img_array, encoding='rgb8')
        self.publisher.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)
    node = QRTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
