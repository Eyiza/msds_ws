# MSDS Robot – Non-Technical User Guide (MVP)

## Introduction
The **MSDS Robot** is an autonomous medical supply delivery robot designed to assist healthcare personnel by reducing their physical workload and automating routine delivery tasks.

This guide explains how to **operate the robot’s MVP (Minimum Viable Product)** that currently supports odometry, mapping (SLAM), and autonomous navigation using Nav2.

---

## Current Capabilities (MVP)
The robot can currently:
1. **Map its environment (SLAM)** using LiDAR.  
2. **Navigate autonomously (Nav2)** within a mapped area while avoiding obstacles.  
3. **Track motion (Odometry)** to display movement and position in real time.  
4. **Be visualized in RViz2** for mapping and navigation.  

---

## What You Need
- The **Raspberry Pi** running the robot’s ROS 2 Jazzy workspace (`msds_ws`).
- A **Ubuntu system (laptop/PC)** connected to the same network as the robot.
- **RViz2** installed on your Ubuntu system for visualization.
- The robot **powered ON** and placed on a flat, obstacle-free surface.

---

## Getting Started

### Step 1 – Power On the Robot
Turn on the robot’s **main switch**.

### Step 2 – SSH into the Raspberry Pi
On your Ubuntu system, open a terminal and connect to the Raspberry Pi:

```bash
ssh <username>@<raspberry_pi_ip_address>
```
Replace <username> and <raspberry_pi_ip_address> with Raspberry Pi’s actual credentials (e.g. ssh ubuntu@192.168.1.102).

### Step 3 - Launch the Robot
Once connected, launch the robot’s ROS 2 workspace:
```bash
ros2 launch msds_bringup real_robot.launch.py use_slam:=true
```
This command starts the robot in SLAM mode, enabling mapping and navigation. <br>
If a Joystick is connected and powered on before this command, it will be automatically enabled for manual movement.

### Step 4 - Expose the Robot’s ROS 2 Topics
In another terminal on your Ubuntu system, set the ROS 2 environment variables to connect to the robot:
```bash
export ROS_DOMAIN_ID=2
```
This makes the robot’s topics visible to your computer for visualization and control through RViz2.

### Step 5 - Launch and Control in RViz2
In the same terminal, launch RViz2 with the robot’s configuration:
```bash
rviz2 
```
Add the following displays in RViz2:
- **RobotModel**: To visualize the robot.
- **LaserScan**: To see the LiDAR data.
- **Map**: To view the SLAM-generated map.
- **Path**: To visualize the planned navigation path.
- **Odometry**: To track the robot’s movement.
- **TF**: To visualize the robot’s coordinate frames.
- **2D Nav Goal**: To set navigation goals by clicking on the map.
- **2D Pose Estimate**: To set the robot’s initial position on the map.

You can now interact with the robot by setting navigation goals and observing its movement in RViz2.

## Manual Control
### Using Keyboard
For manual movement, on your Ubuntu system or directly on the Raspberry Pi, run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use the following keys to control the robot:
- `i`: Move forward
- `,`: Move backward
- `j`: Turn left
- `l`: Turn right
- `k`: Stop
- `q`: Quit the teleop program

### Using Joystick
If a joystick is connected, you can control the robot using the joystick axes and buttons. The joystick will automatically be enabled if connected before launching the robot.

## Standby Mode
To stop all active processes and put the robot in standby mode, press `Ctrl + C` in the terminal where the robot was launched. This will safely shut down the robot’s operations.

## Troubleshooting
- **Robot not moving**: Ensure the robot is powered on and the ROS 2 nodes are running.
- **No map displayed**: Make sure SLAM mode is enabled and the LiDAR is functioning.
- **Cannot connect via SSH**: Check the network connection and IP address.
- **Joystick not working**: Ensure the joystick is connected before launch and recognized by the system.