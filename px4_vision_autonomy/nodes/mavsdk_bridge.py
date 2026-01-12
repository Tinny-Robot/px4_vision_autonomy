#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)
import threading

class MavsdkBridge(Node):
    """
    Bridges ROS2 velocity commands to MAVSDK offboard control.
    """
    def __init__(self):
        super().__init__('mavsdk_bridge')
        
        self.declare_parameter('system_address', 'udp://:14540')
        self.system_address = self.get_parameter('system_address').get_parameter_value().string_value
        
        self.drone = System()
        self.loop = asyncio.new_event_loop()
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/vision/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.current_vel = Twist()
        self.offboard_active = False
        
        # Start the asyncio loop in a separate thread
        self.thread = threading.Thread(target=self.start_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info(f'MAVSDK Bridge Started. Connecting to {self.system_address}')

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run())

    async def run(self):
        await self.drone.connect(system_address=self.system_address)

        self.get_logger().info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Drone discovered!")
                break

        # Start offboard task
        asyncio.ensure_future(self.offboard_control_loop())
        
        # Keep the loop running
        while True:
            await asyncio.sleep(1)

    async def offboard_control_loop(self):
        """
        Continuously sends velocity setpoints to the drone.
        """
        self.get_logger().info("Starting offboard control loop")
        
        # Initial setpoint
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        try:
            await self.drone.offboard.start()
            self.offboard_active = True
            self.get_logger().info("Offboard mode started")
        except OffboardError as error:
            self.get_logger().error(f"Starting offboard mode failed: {error}")
            return

        while True:
            # Send the current velocity command
            # MAVSDK uses m/s. Twist uses m/s.
            # Coordinate frame: Forward (x), Right (y), Down (z)
            
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    self.current_vel.linear.x,
                    self.current_vel.linear.y,
                    self.current_vel.linear.z,
                    self.current_vel.angular.z # Yaw speed
                )
            )
            await asyncio.sleep(0.1) # 10Hz

    def cmd_vel_callback(self, msg):
        self.current_vel = msg
        # self.get_logger().info(f"Received cmd_vel: {msg.linear.x}, {msg.linear.y}, {msg.linear.z}")

def main(args=None):
    rclpy.init(args=args)
    node = MavsdkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
