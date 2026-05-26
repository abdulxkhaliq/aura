#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('device', 0)
        self.declare_parameter('fps', 30)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        device = self.get_parameter('device').value
        fps    = self.get_parameter('fps').value
        w      = self.get_parameter('width').value
        h      = self.get_parameter('height').value

        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open /dev/video{device}')
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f'Camera /dev/video{device} → {actual_w}x{actual_h} @ {fps}fps')

        self.pub   = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / fps, self.capture)

    def capture(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Camera read failed', throttle_duration_sec=2.0)
            return

        msg          = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height   = frame.shape[0]
        msg.width    = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.step     = msg.width * 3
        msg.data     = frame.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
