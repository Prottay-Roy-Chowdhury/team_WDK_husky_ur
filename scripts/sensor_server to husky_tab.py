import socket
import json
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# TCP Server Configuration
SERVER_IP = "192.168.0.35"  # Replace with the actual IP of the tablet
TCP_PORT = 5000

class HuskyControl(Node):
    def __init__(self):
        super().__init__("husky_control")

        # ROS 2 Publisher for Husky movement
        self.cmd_pub = self.create_publisher(Twist, "/a200_0000/cmd_vel", 10)

        # ROS 2 Subscriber for ArUco STOP/GO signals
        self.create_subscription(String, "/aruco_status", self.aruco_callback, 10)

        self.stop_flag = False  # Flag to track STOP condition

        # Connect to tablet TCP server
        self.connect_to_server()

    def connect_to_server(self):
        """Establishes TCP connection with the tablet in a non-blocking way."""
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client_socket.connect((SERVER_IP, TCP_PORT))
            self.get_logger().info(f"✅ Connected to tablet at {SERVER_IP}:{TCP_PORT}")

            # Start a separate thread for listening
            thread = threading.Thread(target=self.listen_for_data, daemon=True)
            thread.start()

        except Exception as e:
            self.get_logger().error(f"❌ Connection failed: {e}")

    def listen_for_data(self):
        """Continuously listens for processed sensor data from the tablet in a separate thread."""
        try:
            while rclpy.ok():
                self.client_socket.send(b"GET_DATA")

                data = self.client_socket.recv(1024).decode()

                if not data:
                    self.get_logger().warning("⚠️ No data received. Retrying...")
                    continue

                try:
                    sensor_values = json.loads(data)
                    self.process_sensor_data(sensor_values)
                except json.JSONDecodeError:
                    self.get_logger().error("❌ Received invalid JSON data.")

        except (ConnectionResetError, BrokenPipeError):
            self.get_logger().error("🔴 Connection lost. Attempting reconnect...")
            self.connect_to_server()  # Try reconnecting

        except Exception as e:
            self.get_logger().error(f"❌ Error in listen_for_data: {e}")

        finally:
            self.client_socket.close()
            self.get_logger().info("🔴 Disconnected from tablet.")

    def process_sensor_data(self, sensor_values):
        """Processes the received average sensor data and controls Husky accordingly."""
        # Extract processed averages
        avg_x = sensor_values.get("avg_x", 0.0)
        avg_y = sensor_values.get("avg_y", 0.0)

        self.send_velocity_command(avg_x, avg_y)

    def send_velocity_command(self, avg_x, avg_y):
        """Converts sensor data into velocity commands and publishes them."""

        # print(self.stop_flag)
        if self.stop_flag == False:
            

            twist = Twist()

            # Linear movement: Forward (avg_y > 0) / Backward (avg_y < 0)
            twist.linear.x = 0.1 if avg_y > 4 else -0.1 if avg_y < -4 else 0.0

            # Angular movement: Right (avg_x > 0) / Left (avg_x < 0)
            twist.angular.z = -0.1 if avg_x > 4 else 0.1 if avg_x < -4 else 0.0

            self.cmd_pub.publish(twist)

    def aruco_callback(self, msg):
        """Stops Husky when an ArUco marker is detected (STOP message)."""

        if msg.data == "STOP":
            self.get_logger().info("🚨 STOP signal received. Stopping Husky.")
            self.stop_flag = True
        else:
            self.get_logger().info("✅ GO signal received. Resuming movement.")
            self.stop_flag = False

    # def stop_husky(self):
    #     """Immediately stops Husky's movement."""
    #     twist = Twist()
    #     # Linear movement: stop
    #     # twist.linear.x = 0.0

    #     # # Angular movement: stop
    #     # twist.angular.z = -0.0
        
    #     self.cmd_pub.publish(twist)
    #     self.get_logger().info("🛑 Husky Stopped")

def main():
    rclpy.init()
    husky_control = HuskyControl()
    rclpy.spin(husky_control)

    husky_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()