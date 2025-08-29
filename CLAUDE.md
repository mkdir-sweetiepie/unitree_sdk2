# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System & Commands

### Building Examples
```bash
mkdir build
cd build
cmake ..
make
```

### Installing SDK
```bash
# System-wide installation
mkdir build
cd build
cmake ..
sudo make install

# Custom installation path
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

### Running Examples
Examples are built to `build/bin/` directory. Most require network interface parameter:
```bash
# Example usage patterns
./go2_sport_client <network_interface> <mode>
./h1_2_ankle_track <network_interface>
./g1_loco_client_example <network_interface>
```

## Architecture Overview

### Core Components
- **DDS Communication**: Built on Eclipse CycloneDX for robot communication
- **Robot APIs**: Separate client libraries for each robot model (GO2, H1, G1, B2)
- **Low-Level Control**: Direct motor control via DDS messages
- **High-Level Control**: Sport mode, locomotion, and specialized APIs

### Robot Support Matrix
- **GO2**: Sport mode, low-level control, video, audio, state monitoring
- **H1**: Locomotion control, parallel mechanism (ankle) control, arm control
- **G1**: Arm control, locomotion, audio, dex hand control
- **B2/B2W**: Sport mode control similar to GO2

### Directory Structure
- `include/unitree/`: SDK headers organized by functionality
  - `common/`: Core utilities (DDS, logging, threading, JSON)
  - `robot/`: Robot-specific APIs and clients
  - `idl/`: DDS message definitions for each robot
- `example/`: Robot-specific example implementations
- `lib/`: Pre-compiled SDK libraries (aarch64/x86_64)
- `thirdparty/`: CyclonDX DDS dependencies

### Communication Architecture
The SDK uses DDS (Data Distribution Service) for all robot communication:
- Each robot API uses specialized DDS topics
- Messages defined in IDL format under `include/unitree/idl/`
- Client-server pattern with request/response and publish/subscribe models
- Network interface must be specified for robot connection

### Key Programming Patterns
- **Client Initialization**: Create client → Initialize DDS → Connect to robot
- **State Management**: Subscribe to robot state topics for monitoring
- **Command Sending**: Publish command messages at regular intervals
- **Error Handling**: Check response status and connection state
- **Resource Management**: Proper cleanup of DDS entities

### Special Features
- **H1 Parallel Mechanism**: PR Mode for ankle joint control using parallel kinematics
- **Multi-DOF Control**: Support for various robot configurations (27-DOF, dual-arm)
- **Real-time Control**: Precise timing control for motor commands
- **State Machine Examples**: Advanced control logic implementations

## Development Notes

### Prerequisites
- Ubuntu 20.04 LTS (recommended)
- GCC 9.4.0 or compatible
- CMake 3.5+
- C++17 support

### Network Configuration
Most examples require network interface parameter for robot communication. Use `ip addr` to find your interface (e.g., `enp44s0`, `eth0`).

### Testing Robot Control
Always ensure robots are properly suspended or in safe position before running control examples, especially low-level motor control.

### Example Integration
Reference `example/cmake_sample` for integrating unitree_sdk2 into external CMake projects using `find_package(unitree_sdk2)`.