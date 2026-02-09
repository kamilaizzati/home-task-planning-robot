# PlanSys2 Home Service Robot

## Overview
This repository contains the core implementation of my undergraduate thesis project on  
**task planning for a mobile service robot using ROS 2 and PlanSys2**.

The system uses **PDDL-based planning** to generate and execute task sequences such as
navigation and service actions in a simulated home environment.

A custom Python node is implemented as an interface to PlanSys2 services for:
- submitting PDDL problems
- requesting plans
- triggering plan execution

This project focuses on **planning system integration**, not hardware deployment or
low-level control optimization.

---

## Key Technologies
- ROS 2 Humble
- PlanSys2
- PDDL (Domain & Problem)
- Python (rclpy)
- Nav2
- Gazebo
- TurtleBot3 simulation

---

## System Architecture
1. PDDL domain and problem define robot capabilities and goals
2. Problem is sent to PlanSys2 Problem Expert
3. Planner generates a plan
4. PlanSys2 Executor executes the plan
5. Navigation actions are handled by Nav2

---

## Repository Structure
```text
plansys2-home-service-robot/
├── pddl/            PDDL domain and problem files
├── scripts/         Python nodes interfacing with PlanSys2
├── maps/            Navigation maps
├── launch/          ROS 2 launch files
├── requirements.txt Python dependencies
└── README.md        Project documentation

```
---
## Requirements

### Operating System
- Ubuntu 22.04 (recommended)

### ROS 2
Install ROS 2 Humble:
```bash
sudo apt update
sudo apt install ros-humble-desktop
```
### PlanSys2
```bash
mkdir -p ~/plansys2_ws/src
cd ~/plansys2_ws/src
git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system.git

# Optional
git clone https://github.com/IntelligentRoboticsLabs/plansys2_tfd_plan_solver.git
git clone https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples.git

cd ~/plansys2_ws
rosdep install -y -r -q --from-paths src --ignore-src --roshumble <ros2-humble>
colcon build --symlink-install
```
### Nav2 & TurtleBot3
```bash
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3*
```
### Python Dependencies
Install Dependencies including rclpy, plansys2_msgs
---
### How to Run the Project
1. Source ROS 2
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Launch Simulation and PlanSys2
   ```bash
   ros2 launch launch/plansys_bringup.launch.py
   ```
3. Run PlanSys2 Interface Node
   ```bash
   ros2 run <your package name> plansys_exec.py
   ```
   This node will sends the PDDL problem, retrieves generated plan, and triggers plan execution

### Notes
- The python node acts as a custom interface to PlanSys2 services, all planning logic is handled internally by PlanSys2
- This repository is intended for educational and portfolio purposes. For further discussions, contact the author through mila9255@gmail.com
