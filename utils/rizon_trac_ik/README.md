# Rizon TRAC-IK Module

This module provides a **TRAC-IK based inverse kinematics (IK) solver** for the **Flexiv Rizon** 7-DOF robot. It integrates the KDL chain extracted from the URDF, forward kinematics utilities, and TRAC-IK solver for robust IK computation.

---

## File Overview

### 1. `urdf_xml.txt`
- Contains the URDF description of the **Rizon** robot, including:
  - Joint definitions (limits, axes, origins).
  - Link inertial, visual, and collision properties.
  - Mesh references for both visualization and collision checking:contentReference[oaicite:0]{index=0}.
- Used by TRAC-IK to construct the kinematic chain.

### 2. `rizon_trac_ik.cpp` / `rizon_trac_ik.hpp`
- Implements the `RizonTracIK` class:
  - Loads robot URDF from `urdf_xml.txt`.
  - Initializes the **KDL chain**, joint limits, and the TRAC-IK solver:contentReference[oaicite:1]{index=1}.
  - Provides:
    - `get_ik(current_q, desired_pose, return_type)`  
      Computes IK solution for a target pose given the current joint state.  
      - Supports returning either absolute joint values (`'q'`) or relative increments (`'u'`).  
    - `get_flange_pose(current_q)`  
      Computes the forward kinematics (end-effector pose) from the current joint configuration.
- Includes helper function `PrintChainDetails` for debugging the KDL chain.

### 3. `main.cpp`
- Example program demonstrating the TRAC-IK usage:
  - Initializes the solver with the URDF description.  
  - Defines test joint configurations and target poses.  
  - Runs both **forward kinematics** and **inverse kinematics** queries.  
  - Prints resulting joint solutions and end-effector poses to console:contentReference[oaicite:2]{index=2}.