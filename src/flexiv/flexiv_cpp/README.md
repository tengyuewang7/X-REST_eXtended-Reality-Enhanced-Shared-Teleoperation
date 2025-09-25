# Flexiv Robot Control Programs

This repository provides a collection of **Flexiv Rizon robot** control programs built on **ROS 2**, covering teleoperation, motion planning, inverse kinematics, 3D reconstruction, Unity-based decision making, and hardware I/O integration.  

---

## File Overview

### 1. `keyboard_control.cpp`
- Keyboard- and gamepad-based teleoperation of the Flexiv robot.  
- Publishes robot states to ROS 2 topics.  
- Supports manual position/velocity control and gripper actuation.  
- Includes safety mechanisms such as fault monitoring and reset.  

### 2. `robot_test.cpp`
- ROS 2 node `robot_test` that subscribes to robot states and publishes control commands.  
- Demonstrates mode switching (joint position, joint torque, Cartesian modes).  
- Example of impedance parameter tuning and mode transitions.  

### 3. `test_ros2.cpp`
- ROS 2 node `TestROS2` showcasing basic robot state acquisition and publishing.  
- `FlexivRobot` class encapsulates initialization, homing, and joint control.  
- Provides digital I/O operations and scheduler integration.  

### 4. `unity_decision_making.cpp`
- ROS 2 node for robot–Unity interactive decision making.  
- Demonstrates human–robot shared control with task-level reasoning (e.g., Tower of Hanoi).  
- Supports both manual and shared control modes.  

### 5. `flexiv_reconstruction.cpp`
- ROS 2 node `Flexiv3dReconstruction` for 3D point cloud reconstruction.  
- Subscribes to RGB/depth images and camera poses, integrates data using Open3D.  
- Publishes reconstructed point clouds for environment perception.  

### 6. `flexiv_robot.cpp`
- ROS 2 node `FlexivRobot`: a complete robot control framework.  
- Publishes robot states and subscribes to various command topics (`ModeController`, `RigidBody`, etc.).  
- Provides `set_mode` service and digital I/O handling.  
- Implements real-time joint streaming and fault recovery.  

### 7. `flexiv_test.cpp`
- ROS 2 node `FlexivTest`, integrating **TRAC-IK** and a custom solver.  
- Tracks target trajectories with obstacle avoidance.  
- Loads obstacle point clouds and switches between manual and shared control.  
- Records experimental data upon shutdown.  

### 8. `flexiv_trac_ik.cpp`
- ROS 2 node `FlexivTracIK` for inverse kinematics based on TRAC-IK.  
- Subscribes to end-effector poses, computes joint solutions, and commands the robot.  
- Supports suction-based end-effector control.  
- Logs robot state and IK results for later analysis.  

### 9. `flexiv_unity.cpp` & `flexiv_unity_nrt.cpp`
- Interfaces between the robot and Unity for interactive control.  
- `flexiv_unity_nrt.cpp` implements **non-real-time (NRT)** control.  
- Integrates **Nlopt IK** solver for smooth tracking, obstacle avoidance, and manual/shared modes.  
- Includes digital I/O control and kinematic initialization routines. 