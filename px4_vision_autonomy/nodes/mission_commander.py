#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityBodyYawspeed)
import threading
import math
import time

# Mission States
STATE_IDLE = "IDLE"
STATE_ARM = "ARM"
STATE_TAKEOFF = "TAKEOFF"
STATE_NAVIGATE = "NAVIGATE"
STATE_SCAN = "SCAN"
STATE_ALIGN = "ALIGN"
STATE_DESCEND = "DESCEND"
STATE_LAND = "LAND"
STATE_DONE = "DONE"

class MissionCommander(Node):
    def __init__(self):
        super().__init__('mission_commander')
        
        self.declare_parameter('system_address', 'udp://:14540')
        self.system_address = self.get_parameter('system_address').get_parameter_value().string_value
        
        # Vision Parameters
        self.declare_parameter('kp_x', 0.002)
        self.declare_parameter('kp_y', 0.002)
        self.declare_parameter('descent_speed', 0.2)
        self.declare_parameter('pixel_error_threshold', 20.0)

        # Mapping helpers for quick iteration
        self.declare_parameter('swap_axes', False)
        self.declare_parameter('flip_x', False)
        self.declare_parameter('flip_y', False)

        self.kp_x = self.get_parameter('kp_x').get_parameter_value().double_value
        self.kp_y = self.get_parameter('kp_y').get_parameter_value().double_value
        self.descent_speed = self.get_parameter('descent_speed').get_parameter_value().double_value
        self.pixel_error_threshold = self.get_parameter('pixel_error_threshold').get_parameter_value().double_value

        self.swap_axes = self.get_parameter('swap_axes').get_parameter_value().bool_value
        self.flip_x = self.get_parameter('flip_x').get_parameter_value().bool_value
        self.flip_y = self.get_parameter('flip_y').get_parameter_value().bool_value

        # State
        self.state = STATE_IDLE
        self.marker_visible = False
        self.last_marker_time = 0
        self.current_error = Point()
        
        # Subscribers
        self.error_sub = self.create_subscription(
            Point,
            '/aruco/center_error',
            self.error_callback,
            10)
            
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # MAVSDK
        self.drone = System()
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.start_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info('Mission Commander Started')

    def error_callback(self, msg):
        self.current_error = msg
        self.last_marker_time = time.time()
        self.marker_visible = True

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run_mission())

    async def run_mission(self):
        await self.drone.connect(system_address=self.system_address)
        
        self.get_logger().info("Waiting for drone connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Drone connected!")
                break
                
        self.get_logger().info("Waiting for global position...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("Global position OK")
                break
        
        # Wait until the drone is ready to be armed
        self.get_logger().info("Waiting for drone to be ready to arm...")
        async for health in self.drone.telemetry.health():
            if health.is_armable:
                self.get_logger().info("Drone is ready to arm")
                break

        # Start Mission Loop
        self.state = STATE_ARM
        
        while self.state != STATE_DONE:
            self.status_pub.publish(String(data=f"State: {self.state}"))
            self.get_logger().info(f"Current State: {self.state}")
            
            if self.state == STATE_ARM:
                await self.drone.action.arm()
                self.state = STATE_TAKEOFF
                
            elif self.state == STATE_TAKEOFF:
                await self.drone.action.takeoff()
                await asyncio.sleep(10) # Wait for takeoff
                try:
                    await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3.0, 0.0))
                    await self.drone.offboard.start()
                    await asyncio.sleep(5)
                except OffboardError as e:
                    self.get_logger().error(f"Offboard failed: {e}")
                
                self.state = STATE_NAVIGATE
                
            elif self.state == STATE_NAVIGATE:
                self.get_logger().info("Navigating to Inspection Point (0, 5, 3)")
                await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 5.0, -3.0, 0.0))
                await asyncio.sleep(10)
                self.state = STATE_SCAN
                
            elif self.state == STATE_SCAN:
                self.get_logger().info("Scanning for marker...")
                # Yaw sweep
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 15.0))
                
                start_scan = time.time()
                while time.time() - start_scan < 24: # 360 degrees at 15 deg/s = 24s
                    if time.time() - self.last_marker_time < 0.5:
                        self.get_logger().info("Marker Found!")
                        self.state = STATE_ALIGN
                        break
                    await asyncio.sleep(0.1)
                
                if self.state != STATE_ALIGN:
                    self.get_logger().warn("Scan complete, marker not found. Retrying scan...")
                    
            elif self.state == STATE_ALIGN:
                # Vision Control Loop
                if time.time() - self.last_marker_time > 1.0:
                    self.get_logger().warn("Marker lost during alignment!")
                    self.state = STATE_SCAN # Go back to scan
                    continue
                    
                err_x = self.current_error.x
                err_y = self.current_error.y

                # Compute mapping from image error to body velocities
                if self.swap_axes:
                    vel_x = err_x * self.kp_x
                    vel_y = err_y * self.kp_y
                else:
                    vel_y = err_x * self.kp_x
                    vel_x = -err_y * self.kp_y

                # Apply flips if requested
                if self.flip_x:
                    vel_x = -vel_x
                if self.flip_y:
                    vel_y = -vel_y

                # Clamp
                vel_x = max(min(vel_x, 1.0), -1.0)
                vel_y = max(min(vel_y, 1.0), -1.0)
                
                # Debug log to help tune mapping
                self.get_logger().info(f'ALIGN: err_x={err_x:.1f} err_y={err_y:.1f} -> vel_x={vel_x:.3f} vel_y={vel_y:.3f} swap={self.swap_axes} flip_x={self.flip_x} flip_y={self.flip_y}')

                # Check if centered
                if abs(err_x) < self.pixel_error_threshold and abs(err_y) < self.pixel_error_threshold:
                    self.get_logger().info("Centered! Starting Descent.")
                    self.state = STATE_DESCEND
                    
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(vel_x, vel_y, 0.0, 0.0))
                await asyncio.sleep(0.1)
                
            elif self.state == STATE_DESCEND:
                if time.time() - self.last_marker_time > 0.5:
                    self.get_logger().warn("Marker lost during descent! Stopping.")
                    await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -0.5, 0.0))
                    await asyncio.sleep(2)
                    self.state = STATE_ALIGN
                    continue
                
                err_x = self.current_error.x
                err_y = self.current_error.y

                if self.swap_axes:
                    vel_x = err_x * self.kp_x
                    vel_y = err_y * self.kp_y
                else:
                    vel_y = err_x * self.kp_x
                    vel_x = -err_y * self.kp_y

                if self.flip_x:
                    vel_x = -vel_x
                if self.flip_y:
                    vel_y = -vel_y

                vel_z = self.descent_speed

                # Altitude check
                async for position in self.drone.telemetry.position():
                    current_alt = position.relative_altitude_m
                    break

                self.get_logger().info(f'DESCEND: alt={current_alt:.2f} vel_x={vel_x:.3f} vel_y={vel_y:.3f} vel_z={vel_z:.3f}')

                if current_alt < 0.3:
                    self.get_logger().info("Low altitude reached. Landing.")
                    self.state = STATE_LAND

                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(vel_x, vel_y, vel_z, 0.0))
                await asyncio.sleep(0.1)
                
            elif self.state == STATE_LAND:
                try:
                    await self.drone.action.land()
                except Exception as e:
                    self.get_logger().error(f"Landing failed: {e}")
                
                async for is_armed in self.drone.telemetry.armed():
                    if not is_armed:
                        self.get_logger().info("Disarmed. Mission Complete.")
                        break
                
                self.state = STATE_DONE
                
            await asyncio.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = MissionCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
