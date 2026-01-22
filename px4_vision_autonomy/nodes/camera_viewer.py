#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
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

class CameraViewer(Node):
    """
    A simple node to view the camera feed from Gazebo.
    """
    def __init__(self):
        super().__init__('camera_viewer')
        self.subscription = self.create_subscription(
            Image,
            '/aruco/debug_image',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        # self.bridge = CvBridge()
        self.get_logger().info('Camera Viewer Node Started')

    def image_callback(self, msg):
        """
        Callback function for the camera image topic.
        """
        try:
            cv_image = imgmsg_to_cv2(msg)
            # Resize for compact viewing
            cv_image = cv2.resize(cv_image, (320, 240))
            cv2.imshow("Drone Camera View", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    camera_viewer = CameraViewer()
    try:
        rclpy.spin(camera_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        camera_viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
