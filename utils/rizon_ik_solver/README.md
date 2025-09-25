# Rizon IK Solver

This module implements **inverse kinematics (IK) solvers** for the **Flexiv Rizon** robot.  
It integrates numerical optimization, control-based IK, and TRAC-IK to provide multiple approaches for pose tracking and obstacle avoidance.

---

## File Overview

### 1. `rizon_ik_solver.cpp` / `rizon_ik_solver.hpp`
- Defines the `RizonIKSolver` class, which combines:
  - **Analytical kinematics model** (`RizonKinematics`)
  - **TRAC-IK solver** (`RizonTracIK`)
  - **NLOpt-based numerical optimization** (using MMA optimizer).
- Implements multiple IK methods:
  - `nlopt_ik()` → solves IK via nonlinear optimization, minimizing position/orientation error while considering collision constraints:contentReference[oaicite:0]{index=0}.
  - `control_ik()` → direct control-based IK update using the kinematics model.
  - `trac_ik()` → TRAC-IK solver for robust inverse kinematics.
- Supports gradient computation for optimization using **LibTorch tensors**.

### 2. `main.cpp`
- Provides benchmarking of IK solvers.
- Generates **random target poses** (random positions + unit quaternions):contentReference[oaicite:1]{index=1}.
- Compares:
  - `nlopt_ik` (optimization-based)
  - `control_ik` (control update-based)
- Measures average execution time, computes **performance improvement ratio**, and reports **mean and standard deviation** across runs.

### 3. `nlopt_ik.cpp`
- Implements a standalone **nonlinear optimization-based IK example** using **NLOpt + LibTorch**:contentReference[oaicite:2]{index=2}.
- Defines an `inverse_kinematics_objective` function that minimizes:
  1. End-effector pose error
  2. Joint increment regularization
  3. Collision-avoidance penalty from closest-point Jacobians.
- Demonstrates:
  - Updating robot kinematics
  - Iteratively applying IK increments
  - Comparing runtime performance
  - Testing alternative IK strategies (direct IK, control IK).
- Useful for validating optimization setups and experimenting with solver parameters.
