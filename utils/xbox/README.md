# Rizon Xbox Control & Multithreading Tests

This module provides an **Xbox-based teleoperation interface** for the Flexiv Rizon robot, along with a **multithreading test program** to validate concurrency and memory management in C++.

---

## File Overview

### 1. `main.cpp`
- Demonstrates the use of `RizonXbox` and `XboxMonitor` classes.  
- Workflow:
  - Initializes an `RizonXbox` object and starts the interface.
  - Runs a loop at **100 Hz** (10 ms cycle).
  - Retrieves:
    - **Robot pose** (`get_pose()`).
    - **Digital/IO states** (`get_io()`).
    - **Button/axis mapping** (`get_map()`).
  - Prints IO states and control mapping to the console:contentReference[oaicite:0]{index=0}.
- Purpose:  
  Serves as the main entry point for Xbox teleoperation control of the Rizon robot.

### 2. `RizonXbox.hpp`
- Declares the `RizonXbox` class (implementation not shown in the snippet).  
- Likely responsibilities:
  - Handling Xbox controller input.
  - Converting joystick/buttons into robot commands.
  - Providing getter functions for pose, I/O, and mapping.

### 3. `XboxMonitor.hpp`
- Declares the `XboxMonitor` utility class.  
- Likely responsibilities:
  - Listening to Xbox controller events.
  - Providing raw access to button/axis states.
  - Supporting asynchronous monitoring of inputs.

### 4. `test.cpp`
- A **multithreading and shared memory test** program.  
- Features:
  - Uses `std::shared_ptr<std::vector<int>>` protected by a `std::mutex`.  
  - Spawns two threads that repeatedly push and pop values in the vector:contentReference[oaicite:1]{index=1}.
  - Main thread:
    - Prints system time and current vector size every second.
    - Demonstrates thread-safe access with `std::lock_guard`.
- Purpose:  
  Validates **concurrent access**, **mutex locking**, and **memory safety** before deploying similar mechanisms in robot/Xbox integrations.