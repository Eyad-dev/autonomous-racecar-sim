digital-twin-ros2 🏎️
A closed-loop, high-performance digital twin simulation. This project bridges the gap between mechanical URDF models and autonomous pathfinding by running a custom C++ AI stack against a real-time MuJoCo physics engine.

--- Project Overview ---
This repository contains a decoupled autonomous simulation pipeline. It utilizes a custom Python bridge to translate a static chassis URDF into dynamic MuJoCo MJCF for high-fidelity physics (tire friction, torque vectoring), while simultaneously streaming simulated 360-degree LiDAR and Odometry data to a C++ navigation node via ROS 2.

Core Features
Custom ROS 2/MuJoCo Bridge: Real-time translation of /cmd_vel to independent wheel torques.

Simulated Perception: Custom 360-degree LiDAR raycasting with hardware limitations and blind-spot modeling.

C++ Autonomous Brain: * Proportional-Integral-Derivative (PID) steering controller.

Configuration Space (C-Space) dynamic obstacle inflation to account for chassis width.

Reactive proximity forcefields for emergency collision avoidance.

Parallel Visualization: RViz2 integration with dynamic TF broadcasting for real-time RobotModel and LaserScan mapping.

💻 Development Environment
This stack was developed and tested on the following hardware and software configuration:

Hardware Specifications
CPU: i7-11850H
GPU: RTX 3070 mobile
RAM: 32GB RAM 

Software Stack
Operating System: Ubuntu Linux 22.04 LTS

Middleware: ROS 2

Physics Engine: MuJoCo

Languages: * C++ (Pathfinding & Control Node)

Standard Python (Native Linux installation, Bridge & Procedural Track Generation)

🏗️ System Architecture
The Blueprint (URDF): The physical dimensions and sensor transforms of the car.

The Physics (MuJoCo): Processes the MJCF translation to calculate high-speed tire friction, collisions, and raycasting.

The Middleware (ROS 2): Passes Odometry and LaserScan data out, and receives Twist velocity commands in.

The Brain (C++): Calculates gaps, inflates obstacles, calculates PID error, and commands the steering angle.

🛠️ Installation & Build
Ensure your ROS 2 environment is sourced, then clone and build the workspace:

Bash
# Navigate to your workspace
cd ~/your_workspace/src

# Clone the repository (Replace with your actual repo link)
git clone https://github.com/yourusername/fs-digital-twin-ros2.git

# Build the packages
cd ~/your_workspace
colcon build --packages-select twin_simulation twin_control

# Source the newly built workspace
source install/setup.bash
🏁 Usage
The simulation utilizes a multi-terminal launch to decouple the physics engine, the visualization, and the autonomous brain.

1. Launch the Digital Twin (Physics Engine & Bridge)

Bash
ros2 run twin_simulation mujoco_ros_bridge
2. Launch the Autonomous Brain (C++ Node)

Bash
ros2 run twin_control lidar_racer
3. Launch the Visualization (RViz2)

Bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat src/fs-digital-twin-ros2/twin_simulation/urdf/simple_fs_car.urdf)"
rviz2
(Configure RViz2 to display the RobotModel, /scan, and /odom on the world fixed frame).

👨‍💻 Author
Eyad Ahmed Habib Computer Engineering Student, Ain Shams University (ASU)