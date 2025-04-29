# Diff base

[![Multi-Architecture Docker Build](https://github.com/joseluiz/r2m_diff_bot/actions/workflows/docker-push.yml/badge.svg)](https://github.com/joseluiz/r2m_diff_bot/actions/workflows/docker-push.yml)
[![AMD64 Build Status](https://github.com/joseluiz/r2m_diff_bot/actions/workflows/docker-push.yml/badge.svg?job=build-push-amd64)](https://github.com/joseluiz/r2m_diff_bot/actions/workflows/docker-push.yml)
[![ARM64 Build Status](https://github.com/joseluiz/r2m_diff_bot/actions/workflows/docker-push.yml/badge.svg?job=build-push-arm64)](https://github.com/joseluiz/r2m_diff_bot/actions/workflows/docker-push.yml)
[![Multi-Arch Manifest](https://github.com/joseluiz/r2m_diff_bot/actions/workflows/docker-push.yml/badge.svg?job=create-manifests)](https://github.com/joseluiz/r2m_diff_bot/actions/workflows/docker-push.yml)

Current setup is just the stripped demo-example from ros2 control for the diff-controler.

## Setup Instructions

```bash
git clone git@gitlab.rwu.de:prj-iki-ros2/robots/iki_diff_base.git && cd iki_diff_base
```

```bash
git submodule update --init
```

```bash
docker compose -f docker/build.yml build
```

## Docker Compose Files

This project uses several docker-compose files for different purposes:

- `docker/build.yml`: Used to build the Docker images
- `docker/docker-compose.yml`: Launches the system with fake hardware interfaces for simulation
- `docker/docker-compose-hardware.yml`: Launches the system with real hardware interfaces connected to physical motors
- `docker/docker-compose-amd64.yml`: Configuration specific for AMD64 architecture
- `docker/docker-compose-arm64.yml`: Configuration specific for ARM64 architecture

## Start without real hardware:

This mode uses a fake hardware interface that simulates the robot's behavior without requiring physical hardware.

```bash
docker compose -f docker/docker-compose.yml up
```

## Start with real hardware:

This mode connects to the actual physical hardware (motors and sensors) of the diff-base.

### Hardware Setup
1. **Connect CAN interface**: Connect your laptop to the diff-base via the CAN2USB adapter
2. **Set up CAN interface**: The CAN interface needs to be configured before running the robot
   ```bash
   sudo ip link set can0 up type can bitrate 250000
   ```
   If the above command fails, you may need to first create the CAN interface:
   ```bash
   sudo ip link add dev can0 type can bitrate 250000
   sudo ip link set can0 up
   ```

3. **Power up the diff-base**: 
   - Ensure the battery (bike cell) is properly connected
   - Turn on the main power switch located on the side of the robot
   - Verify the status LEDs on the ODrive controllers are lit

4. **Launch the hardware container**:
   ```bash
   docker compose -f docker/docker-compose-hardware.yml up
   ```

### Sending movement commands

To send a simple movement command to the robot:

```bash
ros2 topic pub --rate 30 /cmd_vel geometry_msgs/msg/TwistStamped "
       twist:
         linear:
           x: 0.2
           y: 0.0
           z: 0.0
         angular:
           x: 0.0
           y: 0.0
           z: 0.3"
```

Alternatively, for a one-time command:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}}"
```

## Fake vs Real Hardware Interfaces

### Fake Hardware Interface
- Used for simulation and testing without physical hardware
- Provides simulated feedback and accepts commands just like the real hardware
- Useful for development and testing of algorithms before deploying to the real robot
- No risk of damaging physical equipment during testing

### Real Hardware Interface
- Connects to the actual motor controllers (ODrives) via CAN bus
- Reads real sensor data and sends commands to real motors
- Requires proper hardware setup and configuration
- Used for actual deployment and real-world testing

## Common Issues and Troubleshooting

### No Odometry or Incorrect Movement

**Symptom**: No odometry data is published, or motors turn in unexpected ways.

**Possible Cause**: ODrive controllers have incorrect CAN IDs

**Solution**: 
- Check if the ODrives are configured with CAN ID 0 (left motor) and 1 (right motor)
- You can verify the CAN IDs using the ODrive Tool or by checking the configuration parameters

### Motors Turning in the Same Direction

**Symptom**: Both wheels turn in the same direction when sending rotation commands.

**Solution**: 
1. Check the motor configuration in your hardware YAML file (typically in `bringup/config/`):

2. Rebuild and restart the container after modifications

### Robot Not Responding to Commands

**Symptom**: Robot doesn't move when sending velocity commands.

**Possible Causes**:
1. **Wrong topic**: Ensure you're publishing to `/cmd_vel` with the correct message type
2. **Controller not running**: Check if the controller is loaded with:
   ```bash
   ros2 control list_controllers
   ```
3. **Hardware not ready**: Verify ODrive status LEDs and CAN communication

### CAN Interface Issues

**Symptom**: Communication errors or no response from hardware.

**Solution**:
- Check CAN interface status:
  ```bash
  ip -details link show can0
  ```
- Monitor CAN traffic:
  ```bash
  candump can0
  ```
- If needed, reset the interface:
  ```bash
  sudo ip link set can0 down
  sudo ip link set can0 up type can bitrate 250000
  ```

## Important links
https://github.com/ros-controls/ros2_control_demos

https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-real/