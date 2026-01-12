#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String

class VisionController(Node):
    """
    Subscribes to vision error and publishes velocity commands to center the drone.
    """
    def __init__(self):
        super().__init__('vision_controller')
        
        # Parameters
        self.declare_parameter('kp_x', 0.001) # Pixel error to velocity gain
        self.declare_parameter('kp_y', 0.001)
        self.declare_parameter('kp_z', 0.5)
        self.declare_parameter('target_altitude', 2.0)
        self.declare_parameter('descent_speed', 0.2)
        self.declare_parameter('landing_threshold', 0.2) # meters
        self.declare_parameter('center_deadband', 20) # pixels

        self.kp_x = self.get_parameter('kp_x').get_parameter_value().double_value
        self.kp_y = self.get_parameter('kp_y').get_parameter_value().double_value
        self.kp_z = self.get_parameter('kp_z').get_parameter_value().double_value
        self.center_deadband = self.get_parameter('center_deadband').get_parameter_value().integer_value
        
        # Subscribers
        self.error_sub = self.create_subscription(
            Point,
            '/aruco/center_error',
            self.error_callback,
            10)
            
        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/vision/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/vision/status', 10)
        
        self.last_error_time = self.get_clock().now()
        self.is_centered = False
        
        self.get_logger().info('Vision Controller Started')

    def error_callback(self, msg):
        self.last_error_time = self.get_clock().now()
        
        cmd_vel = Twist()
        
        # Coordinate frame: 
        # Image X (right) -> Drone Y (right) (Body frame)
        # Image Y (down) -> Drone X (back) (Body frame) - Wait, usually camera is forward facing?
        # If camera is down-facing (which is typical for landing on markers):
        # Image X (right) -> Drone Y (right)
        # Image Y (down) -> Drone X (forward) ? No.
        # Let's assume standard NED body frame for drone: X forward, Y right, Z down.
        # Camera mounted looking down, top of image is forward (X).
        # Image X (right) corresponds to Drone Y (right).
        # Image Y (down) corresponds to Drone -X (backward).
        
        # Let's assume:
        # Error X > 0 means marker is to the right of center -> Drone needs to move RIGHT (Positive Y)
        # Error Y > 0 means marker is below center -> Drone needs to move BACK (Negative X)
        
        # Simple P controller
        vel_y = msg.x * self.kp_x
        vel_x = -msg.y * self.kp_y # Note the negative sign
        
        # Deadband
        if abs(msg.x) < self.center_deadband: vel_y = 0.0
        if abs(msg.y) < self.center_deadband: vel_x = 0.0
        
        cmd_vel.linear.x = float(vel_x)
        cmd_vel.linear.y = float(vel_y)
        cmd_vel.linear.z = 0.0 # Maintain altitude for now, or descend if centered
        
        # Check if centered to start descent
        if abs(msg.x) < self.center_deadband and abs(msg.y) < self.center_deadband:
            self.is_centered = True
            cmd_vel.linear.z = 0.2 # Descent speed (positive is down in NED? MAVSDK uses NED usually, but let's check bridge)
            # Actually, MAVSDK Offboard velocity body is: X forward, Y right, Z down.
            # So positive Z is down.
            self.status_pub.publish(String(data="Centering complete. Descending."))
        else:
            self.is_centered = False
            self.status_pub.publish(String(data="Centering..."))

        # Safety limits
        cmd_vel.linear.x = max(min(cmd_vel.linear.x, 1.0), -1.0)
        cmd_vel.linear.y = max(min(cmd_vel.linear.y, 1.0), -1.0)

        self.vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = VisionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
