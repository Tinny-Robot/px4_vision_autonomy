#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped
import cv2
import numpy as np

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8" and img_msg.encoding != "rgb8":
        # Fallback or error, but for now assume bgr8/rgb8
        pass
    
    dtype = np.uint8
    n_channels = 3
    
    img_buf = np.asarray(img_msg.data, dtype=dtype)
    image = np.reshape(img_buf, (img_msg.height, img_msg.width, n_channels))
    
    if img_msg.encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
    return image

def cv2_to_imgmsg(cv_image, encoding="bgr8"):
    img_msg = Image()
    img_msg.header.frame_id = "camera_link" # Set a default frame_id or pass it in
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = encoding
    img_msg.is_bigendian = 0
    img_msg.step = cv_image.shape[1] * 3
    img_msg.data = cv_image.tobytes()
    return img_msg

class ArucoDetector(Node):
    """
    Detects ArUco markers in the camera feed and publishes their position and pixel error.
    """
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('marker_size', 0.2)
        self.declare_parameter('marker_id', 0)
        
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.target_marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/detections', 10)
        self.error_pub = self.create_publisher(Point, '/aruco/center_error', 10)
        self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 10)

        # Subscribers
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)
        
        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        # Create detector parameters in a way that's compatible across OpenCV versions
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Check for OpenCV 4.7+ API
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        else:
            self.use_new_api = False
        
        # Camera Matrix (should be loaded from calibration, using defaults for simulation)
        # Assuming 640x480, 90 deg FOV approx
        self.camera_matrix = np.array([[320.0, 0.0, 320.0],
                                       [0.0, 320.0, 240.0],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros((5, 1))

        self.get_logger().info(f'Aruco Detector Started. Listening on {self.camera_topic}')

    def image_callback(self, msg):
        # self.get_logger().info("Received image")
        try:
            cv_image = imgmsg_to_cv2(msg)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            if self.use_new_api:
                corners, ids, rejected = self.detector.detectMarkers(gray)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None:
                self.get_logger().info(f"Detected markers: {ids.flatten()}")
                # Draw markers
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                for i, marker_id in enumerate(ids):
                    if marker_id[0] == self.target_marker_id:
                        # Estimate pose
                        # Define marker object points
                        marker_points = np.array([[-self.marker_size / 2, self.marker_size / 2, 0],
                                                  [self.marker_size / 2, self.marker_size / 2, 0],
                                                  [self.marker_size / 2, -self.marker_size / 2, 0],
                                                  [-self.marker_size / 2, -self.marker_size / 2, 0]], dtype=np.float32)
                        
                        success, rvec, tvec = cv2.solvePnP(marker_points, corners[i], self.camera_matrix, self.dist_coeffs)
                        
                        if success:
                            # Draw axis
                            cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                            
                            # Publish Pose
                            pose_msg = PoseStamped()
                            pose_msg.header = msg.header
                            pose_msg.pose.position.x = float(tvec[0][0])
                            pose_msg.pose.position.y = float(tvec[1][0])
                            pose_msg.pose.position.z = float(tvec[2][0])
                            # Rotation is not fully converted to quaternion here for brevity, 
                            # but position is enough for centering.
                            self.pose_pub.publish(pose_msg)
                        else:
                            self.get_logger().warn("PnP Failed", throttle_duration_sec=2)
                            
                        # Calculate pixel error from center
                        # Center of the image
                        h, w = cv_image.shape[:2]
                        center_x, center_y = w / 2, h / 2
                        
                        # Center of the marker
                        marker_corners = corners[i][0]
                        marker_center_x = sum([c[0] for c in marker_corners]) / 4
                        marker_center_y = sum([c[1] for c in marker_corners]) / 4

                        # Draw a BIG GREEN BOX around the marker
                        # Convert corners to int format for drawing
                        pts = marker_corners.astype(int)
                        # Draw the polygon (more robust than boundingRect for rotated markers)
                        cv2.polylines(cv_image, [pts], isClosed=True, color=(0, 255, 0), thickness=4)
                        # Also draw a circle at the calculated center
                        cv2.circle(cv_image, (int(marker_center_x), int(marker_center_y)), 5, (0, 0, 255), -1)

                        error_x = center_x - marker_center_x
                        error_y = center_y - marker_center_y
                        
                        error_msg = Point()
                        error_msg.x = float(error_x)
                        error_msg.y = float(error_y)
                        error_msg.z = 0.0 # Depth error could be encoded here if needed
                        self.error_pub.publish(error_msg)
                        
                        self.get_logger().info(f"Marker detected! Error: ({error_x:.1f}, {error_y:.1f})", throttle_duration_sec=1)

            # Publish debug image (Resized for performance)
            # Resize for compact bandwidth usage if source is high-res
            small_debug = cv2.resize(cv_image, (320, 240))
            debug_msg = cv2_to_imgmsg(small_debug, "bgr8")
            debug_msg.header = msg.header # Copy header from input image
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')
            import traceback
            traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
