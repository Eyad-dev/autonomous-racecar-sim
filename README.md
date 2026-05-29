# 🏎️ Digital Twin ROS2

A closed-loop, high-performance digital twin simulation bridging mechanical URDF models with autonomous pathfinding. This project runs a custom C++ AI stack against a real-time MuJoCo physics engine.

## Project Overview

This repository contains a decoupled autonomous simulation pipeline that:
- Translates a static chassis URDF into dynamic MuJoCo MJCF for high-fidelity physics (tire friction, torque vectoring)
- Streams simulated 360-degree LiDAR and Odometry data to a C++ navigation node via ROS 2
- Executes autonomous decision-making in real-time

## Core Features

- **Custom ROS 2/MuJoCo Bridge**: Real-time translation of `/cmd_vel` to independent wheel torques
- **Simulated Perception**: 360-degree LiDAR raycasting with hardware limitations and blind-spot modeling
- **C++ Autonomous Brain**:
  - Proportional-Integral-Derivative (PID) steering controller
  - Configuration Space (C-Space) dynamic obstacle inflation for chassis width
  - Reactive proximity forcefields for emergency collision avoidance
- **Parallel Visualization**: RViz2 integration with dynamic TF broadcasting for real-time RobotModel and LaserScan mapping

## Development Environment

### Hardware Specifications
- **CPU**: Intel i7-11850H
- **GPU**: NVIDIA RTX 3070 Mobile
- **RAM**: 32GB

### Software Stack
- **Operating System**: Ubuntu Linux 22.04 LTS
- **Middleware**: ROS 2
- **Physics Engine**: MuJoCo
- **Languages**:
  - C++ (Pathfinding & Control Node)
  - Python (Bridge & Procedural Track Generation)

## 🏗️ System Architecture

The system consists of four interconnected components:

1. **URDF**: Physical dimensions and sensor transforms of the car
2. **MuJoCo**: Calculates high-speed tire friction, collisions, and raycasting
3. **ROS 2**: Streams Odometry and LaserScan data; receives Twist velocity commands
4. **C++**: Calculates gaps, inflates obstacles, computes PID error, and commands steering angle

## 🛠️ Installation & Build

### Prerequisites
- ROS 2 environment sourced and configured

### Setup Instructions

```bash
# Navigate to your workspace
cd ~/your_workspace/src

# Clone the repository
git clone https://github.com/yourusername/digital-twin-ros2.git

# Build the packages
cd ~/your_workspace
colcon build --packages-select twin_simulation twin_control

# Source the newly built workspace
source install/setup.bash
```

## 🏁 Usage

The simulation utilizes a multi-terminal launch to decouple the physics engine, visualization, and autonomous brain.

### Terminal 1: Launch the Digital Twin (Physics Engine & Bridge)

```bash
ros2 run twin_simulation mujoco_ros_bridge
```

### Terminal 2: Launch the Autonomous Brain (C++ Node)

```bash
ros2 run twin_control lidar_racer
```

### Terminal 3: Launch Visualization (RViz2)

```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat src/digital-twin-ros2/twin_simulation/urdf/simple_fs_car.urdf)"

rviz2
```

**RViz2 Configuration**: Configure RViz2 to display:
- RobotModel
- `/scan` (LaserScan)
- `/odom` (Odometry)
- Use **world** as the fixed frame

## 👨‍💻 Author

**Eyad Ahmed Habib**  
Computer Engineering Student  
Ain Shams University (ASU)

---

*Last Updated: 29-5-2026*
