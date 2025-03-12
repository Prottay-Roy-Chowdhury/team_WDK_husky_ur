import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        
        # Camera parameters (should be calibrated for accuracy)
        self.focal_length = 750  # Adjust based on your camera calibration
        self.real_marker_size = 0.0675  # Real-world marker size in meters
        
        # ROS 2 Subscriber
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera_back/image_compressed',
            self.image_callback,
            10)
        if self.subscription is None:
            self.get_logger().error("Failed to subscribe to /camera_back/image_compressed")
        else:
            self.get_logger().info("Subscribed to /camera_back/image_compressed")
        
        self.bridge = CvBridge()
        self.get_logger().info("Aruco Detector Node Started")
    
    def image_callback(self, msg):
        # Convert compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        distances = []
        
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                corner = corners[i][0]
                marker_width = np.linalg.norm(corner[0] - corner[1])
                distance = (self.real_marker_size * self.focal_length) / marker_width
                distances.append(distance)
                
                # Display distance
                center = tuple(corner.mean(axis=0).astype(int))
                cv2.putText(frame, f"{distance:.2f}m", center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
            min_distance = min(distances)
            if min_distance <= 0.30:
                self.get_logger().info("STOP!!!")
            else:
                self.get_logger().info("GO!!!")
        else:
            self.get_logger().info("GO!!!")
        
        # Display the frame (optional, for debugging)
        cv2.imshow("ArUco Marker Detection", frame)
        cv2.waitKey(1)  # Necessary to update OpenCV window


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    
    aruco_detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()