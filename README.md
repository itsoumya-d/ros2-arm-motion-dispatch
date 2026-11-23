# ROS 2 Robotic Arm Motion Dispatcher

A high-performance kinematic trajectory dispatcher and limit validation engine for multi-axis robotic arm systems.

## Features
- **Kinematic Bound Verification**: Validates joint positions, velocities, accelerations, and jerk limits prior to motion execution.
- **Trajectory Interpolation**: Evaluates polynomial waypoints and validates timing constraints.
- **Execution Controller**: State machine handling trajectory dispatch, progress feedback, emergency stop, and trajectory cancellation.
- **Dual C++ / Python Architecture**: Core safety and numerical validation implemented in C++17 with Python bindings / high-level ROS 2 controller interface.

## Quick Start

### Build & Run C++ Tests
```bash
mkdir build && cd build
cmake ..
cmake --build .
ctest --output-on-failure
```

### Run Python Tests
```bash
pip install -r requirements.txt
pytest -v
```
