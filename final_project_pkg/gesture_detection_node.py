#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time
from roboflowoak import RoboflowOak
import cv2
import os


class GestureDetectionNode(Node):
    def __init__(self):
        super().__init__('gesture_detection_node')

        self.TakePicture = False
        self.frame = None

        # initialize Roboflowoak
        self.rf = RoboflowOak(
            model = 'gesture-detection-t96d8',
            confidence = 0.7, #confidence under 0.7 would be discarded
            overlap = 0.5,
            version = "yolov11",
            api_key = "REJbgYWyGLYclnhkavAS",
            rgb = True,
            depth = True,
            device = None,
            blocking = True
        )

        # timer callback
        self.timer = self.create_timer(1.0, self.timer_callback)

        # publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    # Robot Stop
    def stop(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.z = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.x = 0.0
        self.publisher_.publish(stop_msg)

    # take picture
    def take_picture(self):
        self.stop()
        time.sleep(3)
        if not self.rf.device:
            self.get_logger().error("OAK-D device not found in photoing process")
            return
        else:
            __, frame, __, __ = self.rf.detect()
        if frame is None:
            self.get_logger().warn("Failed Photoing")
        else:
            self.frame = frame
            self.save_picture()

    # save picture
    def save_picture(self):
        save_folder = "/home/username/pictures/" # path can be changed here
        os.makedirs(save_folder, exist_ok=True)

        filename = os.path.join(save_folder, "image_{}.jpg".format(int(time.time())))
        cv2.imwrite(filename, self.frame)
        self.get_logger().info("Picture saved successfully")

        self.TakePicture = False


    def timer_callback(self):
        try:
            result, frame, raw_frame, depth = self.rf.detect()
            predictions = result["predictions"]
            class_data = [p["class"] for p in predictions]

            if "thumbs up" in class_data and self.TakePicture == False:
                self.TakePicture = True
                self.take_picture()

        except Exception as e:
            self.get_logger().error(f"Error during processing: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = GestureDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
