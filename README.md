# PX4 Vision Autonomy

A production-quality ROS2 Humble package for simulated drone perception and offboard control using PX4 SITL and Gazebo.

## Overview

This package implements a complete vision-based precision landing stack. It includes:
- **ArUco Marker Detection**: Detects markers from the simulated camera feed.
- **Vision Controller**: A proportional controller that centers the drone over the marker.
- **MAVSDK Bridge**: Interfaces with PX4 to send velocity commands.
- **Simulation Launch**: One-command launch for PX4 SITL, Gazebo, and the ROS2 stack.

## Prerequisites

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **PX4 Autopilot**: v1.14 or main branch
- **Gazebo**: Gazebo Classic (usually included with PX4 setup) or Gazebo Garden/Harmonic depending on PX4 version. This guide assumes standard PX4 SITL with Gazebo Classic.
- **Python**: 3.10

### System Dependencies

```bash
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-vision-opencv python3-opencv python3-pip
pip3 install mavsdk aioconsole
```

## Installation

1. **Clone PX4-Autopilot** (if not already done):
   ```bash
   cd ~
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   cd PX4-Autopilot
   bash ./Tools/setup/ubuntu.sh
   # Reboot your computer after the script finishes
   ```

2. **Create Workspace and Clone Package**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   # Copy this package here or clone it
   # git clone https://www.github.com/Tinny-Robot/px4_vision_autonomy
   ```

3. **Build**:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

## Running the Simulation

### 1. Full End-to-End Simulation

This launch file attempts to start PX4 SITL and the ROS2 nodes.

```bash
ros2 launch px4_vision_autonomy sim_x500_vision.launch.py px4_dir:=$HOME/PX4-Autopilot
```

**Expected Result**:
- Gazebo window opens with an X500 drone.
- A camera viewer window opens showing the drone's view.
- The drone should be on the ground.
- To start the mission, you can use the MAVSDK shell or QGroundControl to Arm and Takeoff, or run the example mission script.

### 2. Modular Execution (Recommended)

It is often more robust to run PX4 in one terminal and ROS2 in another.

**Terminal 1: PX4 SITL**
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth
```
*Wait for Gazebo to load and the drone to be ready.*

**Terminal 2: ROS2 Perception Stack**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch px4_vision_autonomy perception_only.launch.py
```

### 3. Triggering the Behavior

The `mavsdk_bridge` node waits for the drone to be armed and in offboard mode, or it can be used to command the drone.

To see the precision landing in action:
1. Takeoff manually using QGroundControl or:
   ```bash
   # In a new terminal
   python3 src/px4_vision_autonomy/scripts/mission_example.py
   ```
   *Note: The example script is a standalone sequencer. For the closed-loop vision control, the `mavsdk_bridge` needs to be running.*

### 4. Mission: Inspection & Landing

This mission demonstrates a complete autonomous workflow in a rich simulation environment.

**Launch the Mission:**
```bash
ros2 launch px4_vision_autonomy mission_inspection.launch.py
```

**The Environment:**
The simulation spawns a populated "Inspection Site" world containing:
- **Urban Elements**: Houses, post office, gas station, and fast food restaurant.
- **Props**: Trees, street lights, dumpsters, and playground equipment.
- **Obstacles**: A box obstacle and parked vehicles (Ambulance, SUV).
- **Target**: An ArUco landing pad located at (x=5.8, y=0).

**Mission Sequence:**
1.  **Startup**: PX4 SITL launches with the `gz_x500_mono_cam_down` model (drone with downward-facing camera).
2.  **Takeoff**: The drone arms and takes off to an altitude of 3 meters.
3.  **Navigation**: It flies autonomously to the inspection area near `x=5.0`.
4.  **Search**: The drone yaws to scan for the ArUco marker.
5.  **Precision Landing**: Once the marker is detected, the drone aligns itself using visual feedback and descends for a landing.

**Note on Models:**
The launch file automatically selects the `gz_x500_mono_cam_down` model which allows the drone to see the ground target. No manual model selection is required.

## Node Details

### `aruco_detector`
- **Subscribes**: `/camera/image_raw`
- **Publishes**: 
  - `/aruco/detections` (PoseStamped)
  - `/aruco/center_error` (Point: x, y pixel error)
  - `/aruco/debug_image` (Image with overlays)
- **Config**: `marker_size`, `marker_id` in `params.yaml`

### `vision_controller`
- **Subscribes**: `/aruco/center_error`
- **Publishes**: `/vision/cmd_vel` (Twist)
- **Logic**: Simple P-controller. If error is within deadband, it commands a descent (positive Z velocity).

### `mavsdk_bridge`
- **Subscribes**: `/vision/cmd_vel`
- **Action**: Connects to PX4 via MAVSDK (UDP 14540). Sends `set_velocity_body` commands.

## Configuration

Edit `config/params.yaml` to tune gains:

```yaml
vision_controller:
  ros__parameters:
    kp_x: 0.001 # Increase for more aggressive centering
    descent_speed: 0.2 # m/s
```

## Troubleshooting

- **"ImportError: No module named mavsdk"**: Run `pip3 install mavsdk`.
- **Camera topic not found**: Check `ros2 topic list`. If using a different model, update `camera_topic` in `params.yaml`.
- **Drone not moving**: Ensure PX4 is in Offboard mode. The `mavsdk_bridge` attempts to switch to Offboard, but safety checks (RC loss, no GPS) might prevent it. Ensure you have a GPS lock in simulation (usually automatic).

## License

MIT
