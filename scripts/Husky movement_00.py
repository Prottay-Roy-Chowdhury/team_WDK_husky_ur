import rclpy
import json
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class HuskyMover(Node):
    def __init__(self):
        super().__init__("husky_mover")
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.send_twist)  # Send command every 0.5s
        self.target_pose = self.load_pose("registered_poses.json", index=0)  # Select the first pose

    def load_pose(self, file_path, index=0):
        """ Load a specific pose from file """
        try:
            with open(file_path, "r") as f:
                poses = json.load(f)
            if index < len(poses):
                self.get_logger().info(f"Loaded Pose {index}: {poses[index]}")
                return poses[index]
            else:
                self.get_logger().warn("Invalid index, using default pose.")
        except Exception as e:
            self.get_logger().error(f"Error loading pose: {e}")
        return None

    def send_twist(self):
        """ Publish a Twist message based on the selected pose """
        if not self.target_pose:
            return

        twist = Twist()

        # Example: Move forward if X > 1.0, turn if Y > 1.0
        x = self.target_pose["position"]["x"]
        y = self.target_pose["position"]["y"]

        twist.linear.x = 0.5 if x > 1.0 else 0.0  # Move forward if X > threshold
        twist.angular.z = 0.5 if y > 1.0 else 0.0  # Turn if Y > threshold

        self.publisher_.publish(twist)
        self.get_logger().info(f"Published Twist: linear={twist.linear.x}, angular={twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = HuskyMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Husky Mover")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
