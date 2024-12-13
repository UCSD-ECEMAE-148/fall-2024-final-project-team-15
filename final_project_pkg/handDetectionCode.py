#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import os
import time
from roboflowoak import RoboflowOak
import cv2
from cv_bridge import CvBridge, CvBridgeError


class HandDetectionNode(Node):
    def __init__(self):
        super().__init__('hand_detection_node')

        # Initialize RoboflowOak
        self.rf = RoboflowOak(
            model="rock-paper-scissors-sxsw",
            confidence=0.05,
            overlap=0.5,
            version="14",
            api_key="kDi4xMfGMvnsVqisDYa1",
            rgb=True,
            depth=True,
            device=None,
            blocking=True
        )

        # Publisher for /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to call the callback function periodically
        self.timer = self.create_timer(1.0, self.timer_callback)

        # State variables
        self.TakePicture = False
        self.picture_delay = 3.0  # Delay duration in seconds
        self.last_detect_time = None  # Time when "Rock" was detected

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

    # Robot Stop
    def stop(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

    # Save Pictures
    def save_picture(self, frame):
        save_folder = '/home/projects/ros2_ws/src/final_project/Upload_Folder'
        os.makedirs(save_folder, exist_ok=True)  # Ensure the directory exists
        filename = os.path.join(save_folder, f"image_{int(time.time())}.jpg")
        if not cv2.imwrite(filename, frame):
            self.get_logger().error("Failed to save picture")
        else:
            self.get_logger().info(f"Picture saved to {filename}")

    # Take pictures
    def take_picture(self):
        # Check if frame is available
        _, frame, _, _ = self.rf.detect()
        if frame is None:
            self.get_logger().warn("Failed to capture photo")
        else:
            self.save_picture(frame)
            
        # Reset state
        self.TakePicture = False

    def timer_callback(self):
        try:
            result, frame, raw_frame, depth = self.rf.detect()
            if frame is None or depth is None:
                self.get_logger().warn("No frame or depth data received.")
                return

            predictions = result.get("predictions", [])
            class_data = [p.class_name for p in predictions]
            self.get_logger().info(f"Class Data: {class_data}")

            if "Rock" in class_data and not self.TakePicture:
                self.TakePicture = True
                self.last_detect_time = time.time()  # Record detection time
                self.stop()  # Stop the robot

            # Check if delay has passed
            if self.TakePicture and time.time() - self.last_detect_time >= self.picture_delay:
                self.take_picture()

        except Exception as e:
            self.get_logger().error(f"Error during processing: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HandDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
