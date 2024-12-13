import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarFollowTargetNode(Node):
    def __init__(self):
        super().__init__('lidar_follow_target_node')

        # Publisher for /cmd_vel to send motion commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LiDAR scan data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',  # LiDAR topic name (adjust based on your setup)
            self.lidar_callback,
            10
        )

        # Follow target parameters
        self.follow_distance = 1.0  # Desired distance to maintain (in meters)
        self.monitored_indices = [
            8, 9, 10, 437, 438, 439, 440,
            441, 442, 443, 444, 445, 446, 447, 448
        ]
        self.min_distance = 0.0  # Closest distance in the monitored indices

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
            self.min_distance = min(monitored_distances)
            self.get_logger().info(f"Closest object in monitored indices at {self.min_distance:.2f} meters.")
            self.adjust_speed()
        else:
            self.min_distance = None
            self.get_logger().warn("No valid object detected in monitored indices.")
            self.send_stop_command()

    def adjust_speed(self):
        """
        Adjust robot's speed based on the detected target distance.
        """
        if self.min_distance is None:
            # No valid target detected
            self.send_stop_command()
            return

        # Compute speed adjustment based on the distance error
        distance_error = self.min_distance - self.follow_distance
        cmd_vel = Twist()

        if abs(distance_error) < 0.1:  # Within acceptable range
            cmd_vel.linear.x = 0.0
            self.get_logger().info("Maintaining follow distance.")
        elif distance_error > 0:  # Target is farther away
            cmd_vel.linear.x = min(0.5, distance_error * 0.5)  # Proportional control
            self.get_logger().info(f"Target is far. Approaching: Speed {cmd_vel.linear.x:.2f} m/s")
        else:  # Target is too close
            cmd_vel.linear.x = max(-0.3, distance_error * 0.5)  # Proportional control (negative speed to back up)
            self.get_logger().info(f"Target is close. Backing up: Speed {cmd_vel.linear.x:.2f} m/s")

        cmd_vel.angular.z = 0.0  # No turning needed for now
        self.cmd_vel_pub.publish(cmd_vel)

    def send_stop_command(self):
        # Create a Twist message with zero velocity to stop the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info("Stopping the robot.")


def main(args=None):
    rclpy.init(args=args)
    node = LidarFollowTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



