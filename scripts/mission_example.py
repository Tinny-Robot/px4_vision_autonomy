#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("-- Starting Offboard Mode (Vision Control should take over if running)")
    # Note: This script just demonstrates sequencing. 
    # The actual vision control is handled by the ROS2 node 'mavsdk_bridge' 
    # which also tries to set offboard mode. 
    # If you run this script, it might conflict with mavsdk_bridge if both try to control.
    # This script is a standalone example of how to sequence actions.
    
    try:
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed: {error}")
        print("Disarming")
        await drone.action.disarm()
        return

    print("-- Hovering for 5 seconds")
    await asyncio.sleep(5)
    
    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
