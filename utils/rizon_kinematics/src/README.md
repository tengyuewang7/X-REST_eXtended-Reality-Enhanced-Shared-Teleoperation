# 📖 Rizon Kinematics Module

This module provides a full forward and inverse kinematics (IK) implementation for the **Flexiv Rizon** 7-DOF robot, with support for Jacobian computation, obstacle avoidance, and mesh-based collision checking.

---

## File Overview

### 1. `rizon_kinematics.cpp` / `rizon_kinematics.hpp`
- Implements the `RizonKinematics` class.
- **Core functionality:**
  - Maintains homogeneous transformations between joints and the base.
  - Computes joint transforms, end-effector pose, and Jacobians.
  - Provides multiple IK solvers:
    - **Direct IK**: single-step update from pose error.
    - **Control IK**: incremental update with obstacle avoidance.
    - **Iterative IK**: solves until target pose is reached within tolerance.
  - Supports both `std::vector<double>` and `std::array<double, 7>` joint/pose inputs.
- **Collision & obstacle handling:**
  - Loads robot link meshes via **Assimp** for precise collision geometry.
  - Updates transformed link meshes in real-time for collision checking.
  - Computes minimum distances between robot links and obstacles.
  - Returns Jacobian-based repulsion terms for obstacle avoidance.
- **Utilities:**
  - Get current flange (end-effector) pose in `[px, py, pz, rx, ry, rz, rw]`.
  - Get current joint vector.
  - Compute manipulability measure (Jacobian determinant).

### 2. `tensor_utils.hpp`
- Helper utilities for converting between containers (`std::vector`, `std::array`) and LibTorch tensors.
- Includes quaternion operations, tensor-to-vector conversions, and matrix reshaping.

### 3. `main.cpp`
- Example program demonstrating the kinematics library.
- Features:
  - Initializes the robot in home or random configurations.
  - Solves IK for a target pose.
  - Prints updated joint states and flange pose.
  - Demonstrates obstacle update with random obstacle points. 