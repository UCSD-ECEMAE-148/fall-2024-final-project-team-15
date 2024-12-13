#!/usr/bin/env python3

import cv2
import depthai as dai
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from pathlib import Path

NODE_NAME = "person_follower_lidar_depthai"

# Path to MobileNet SSD blob
nn_path = str((Path(__file__).parent / "models/mobilenet-ssd_openvino_2021.4_6shave.blob").resolve())


class PersonFollower(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Variables for person detection and distance
        self.person_detected = False
        self.person_distance = None  # Distance to the detected person
        self.min_lidar_distance = None  # Closest LiDAR distance in the monitored indices
        self.follow_distance = 1.0  # Desired follow distance in meters
        self.detections = []  # List to store current detections

        # LiDAR monitoring indices
        self.monitored_indices = [8, 9, 10, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 448]

        # DepthAI setup
        self.pipeline = self.create_pipeline()
        self.bridge = CvBridge()
        self.device = dai.Device(self.pipeline)

        # Queues for RGB image and NN output
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.q_nn = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)

        # Timer to process frames
        self.create_timer(0.1, self.process_frames)

    def create_pipeline(self):
        pipeline = dai.Pipeline()

        # Define the RGB camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(300, 300)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)

        # Define the mono cameras and stereo depth
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        # Set resolutions to supported values
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)

        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # Neural network for person detection
        nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
        nn.setBlobPath(nn_path)
        nn.setConfidenceThreshold(0.5)
        nn.setBoundingBoxScaleFactor(0.5)
        nn.setDepthLowerThreshold(100)
        nn.setDepthUpperThreshold(10000)

        cam_rgb.preview.link(nn.input)
        stereo.depth.link(nn.inputDepth)

        # Outputs for RGB and NN
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        nn_out = pipeline.create(dai.node.XLinkOut)
        nn_out.setStreamName("nn")
        nn.out.link(nn_out.input)

        return pipeline

    def lidar_callback(self, msg):
        # Extract distances at the monitored indices
        monitored_distances = []
        for index in self.monitored_indices:
            if 0 <= index < len(msg.ranges):  # Ensure the index is valid
                distance = msg.ranges[index]
                if not (float('inf') == distance or float('nan') == distance):  # Exclude invalid readings
                    monitored_distances.append(distance)

        # Find the minimum distance in the monitored indices
        if monitored_distances:
            self.min_lidar_distance = min(monitored_distances)
            self.get_logger().info(f"Closest object in monitored indices at {self.min_lidar_distance:.2f} meters.")
        else:
            self.min_lidar_distance = None
            self.get_logger().warn("No valid object detected in monitored indices.")

    def process_frames(self):
        # Get RGB frame
        in_rgb = self.q_rgb.tryGet()
        if in_rgb:
            frame = in_rgb.getCvFrame()

            # Get detections
            in_nn = self.q_nn.tryGet()
            self.detections = in_nn.detections if in_nn else []  # Save detections as an instance variable

            self.person_detected = False
            self.person_distance = None

            # Process detections
            for detection in self.detections:
                label = detection.label
                if label == 15:  # Label 15 corresponds to 'person' in MobileNetSSD
                    self.person_detected = True

                    # Spatial coordinates for distance
                    if hasattr(detection, 'spatialCoordinates'):
                        self.person_distance = detection.spatialCoordinates.z / 1000.0  # Convert mm to meters

                    # Draw detection on the frame
                    bbox = self.denormalize_bbox(detection, frame.shape)
                    cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
                    cv2.putText(frame, "Person", (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Show the frame
            cv2.imshow("Person Follower", frame)
            if cv2.waitKey(1) == ord('q'):
                rclpy.shutdown()

            # Control robot based on person detection and LiDAR
            self.control_robot()

    def denormalize_bbox(self, detection, frame_shape):
        """Convert normalized bbox to pixel coordinates."""
        height, width = frame_shape[:2]
        x1 = int(detection.xmin * width)
        y1 = int(detection.ymin * height)
        x2 = int(detection.xmax * width)
        y2 = int(detection.ymax * height)
        return (x1, y1, x2, y2)

    def control_robot(self):
        """Control robot based on camera and LiDAR inputs."""
        twist = Twist()

        if self.person_detected and self.min_lidar_distance is not None:
            # Control distance using LiDAR
            distance_error = self.min_lidar_distance - self.follow_distance
            if abs(distance_error) < 0.1:
                twist.linear.x = 0.0  # Stop if within acceptable range
            elif distance_error > 0:
                twist.linear.x = min(0.5, distance_error * 0.5)  # Move forward
            else:
                twist.linear.x = max(-0.3, distance_error * 0.5)  # Move backward

            # Control orientation using camera
            if self.detections:
                bbox = self.denormalize_bbox(self.detections[0], (300, 300))  # Process the first detection
                person_center_x = (bbox[0] + bbox[2]) // 2
                frame_center_x = 150  # Half of 300px width
                error_x = frame_center_x - person_center_x
                twist.angular.z = -error_x * 0.005

            self.get_logger().info(f"Following person. Distance: {self.min_lidar_distance:.2f} m")
        else:
            # Stop if no person detected or LiDAR data is unavailable
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("No person detected or LiDAR data unavailable.")

        self.vel_pub.publish(twist)


def main():
    rclpy.init()
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
