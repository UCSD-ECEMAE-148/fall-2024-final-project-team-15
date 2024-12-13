
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from roboflowoak import RoboflowOak
from cv_bridge import CvBridge, CvBridgeError

class hand_detection_node(Node):
    def __init__(self):
        super().__init__('hand_dectection_node')

        # Initialize RoboflowOak
        self.rf = RoboflowOak(
            model="gesture-detection-t96d8/4",
            confidence=0.05,
            overlap=0.5,
            version="yolov8",
            api_key="REJbgYWyGLYclnhkavAS",
            rgb=True,
            depth=True,
            device=None,
            blocking=True
        )

        # Publisher for /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to call the callback function periodically
        self.timer = self.create_timer(1.0, self.timer_callback)  # Adjust the interval as needed

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

    def timer_callback(self):
        try:
            # Run model inference
            result, frame, raw_frame, depth = self.rf.detect()

            # Ensure depth and frame are valid
            if frame is None or depth is None:
                self.get_logger().warn("No frame or depth data received.")
                return

            predictions = result.get("predictions", [])

            # Extract class data from predictions
            class_data = [p.class_name for p in predictions]
            self.get_logger().info(f"Class Data: {class_data}")

            # Determine motion commands based on class data
            cmd_vel = Twist()

            if "dicham" in class_data:
                cmd_vel.linear.x = 0.3 
                cmd_vel.angular.z = 0.5 # Turn right
            elif "dilui" in class_data:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0  # Stop
            elif "dinhanh" in class_data:
                cmd_vel.linear.x = 0.3
                cmd_vel.angular.z = -0.5  # Turn left
            elif "dinhanh dunglai" in class_data:
                cmd_vel.linear.x = 0.5
                cmd_vel.angular.z = -0.5  # Forward
            elif "dunglai" in class_data:
                cmd_vel.linear.x = 0.5
                cmd_vel.angular.z = -0.5  # Forward
            elif "rephai" in class_data:
                cmd_vel.linear.x = -0.1
                cmd_vel.angular.z = 0.0  # Backward
            elif "retrai" in class_data:
                cmd_vel.linear.x = 0.9
                cmd_vel.angular.z = 0.0  # SEND IT
            elif "Rock" in class_data:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0  # rps left
            elif "Paper" in class_data:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0  # rps Straight
            elif "Scissors" in class_data:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0  # rps right
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0  # Stop


            # Publish the command
            self.cmd_vel_pub.publish(cmd_vel)

        except Exception as e:
            self.get_logger().error(f"Error during processing: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = hand_detection_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
