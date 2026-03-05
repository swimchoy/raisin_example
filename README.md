# Raisin SDK

SDK for communicating with Raisin robots from external applications.

## Overview

This SDK provides a simple C++ interface for:
- Robot status monitoring (locomotion state, battery, actuators)
- Real-time sensor data subscription (Odometry, PointCloud)
- Autonomous navigation control (waypoints, patrol routes)
- Locomotion control (stand up, sit down)
- Manual/autonomous control mode switching

## Requirements

### System
- Ubuntu 22.04+
- CMake 3.16+
- C++20 compatible compiler

### Dependencies
```bash
sudo apt-get install libeigen3-dev libssl-dev libpcl-dev
```

### Raisin SDK Libraries
This SDK requires the Raisin SDK libraries package provided by Raion Robotics.
The package should contain:
- Headers: `include/` directory
- Libraries: `lib/` directory (libraisin_network.so, libraisim.so, etc.)

## Build

```bash
mkdir build && cd build
cmake .. -DRAISIN_MASTER_PATH=/path/to/raisin_sdk_package
make -j$(nproc)
```

Where `RAISIN_MASTER_PATH` points to the directory containing `install/` folder with SDK libraries.

## Examples

### Monitoring Examples

| Example | Description |
|---------|-------------|
| `example_robot_state` | Locomotion state monitoring |
| `example_battery` | Battery voltage/current |
| `example_actuator_status` | Motor fault status |
| `example_odometry` | Current position |
| `example_pointcloud` | Map/LiDAR data |
| `example_ffmpeg_camera` | Camera stream (`FfmpegPacket`) subscribe (must be decoded for visualization) |

### Control Examples

| Example | Description |
|---------|-------------|
| `example_joy_control` | Manual/autonomous mode switch, stand up/sit down |

### Usage

`robot_id` can be the robot's network name (e.g., `railab_raibo-xxx`) or IP address (e.g., `10.42.0.1`).

```bash
./example_robot_state <robot_id>
./example_battery <robot_id>
./example_actuator_status <robot_id>
./example_odometry <robot_id>
./example_pointcloud <robot_id>
./example_ffmpeg_camera <robot_id>
./example_joy_control <robot_id>
```

### example_joy_control

Interactive tool for control mode switching and locomotion commands.

```bash
./example_joy_control <robot_id>
```

Commands:
- `m` - Set Manual control (gamepad)
- `a` - Set Autonomous control (patrol mode)
- `r` - Release control
- `u` - Stand Up
- `d` - Sit Down
- `s` - Show current state
- `q` - Quit

## SDK Usage

```cpp
#include "raisin_sdk/raisin_client.hpp"

int main() {
    raisin_sdk::RaisinClient client("my_app");

    // Connect
    if (!client.connect("ROBOT_ID")) {
        return 1;
    }

    // Subscribe to robot state (locomotion, battery, actuators)
    client.subscribeRobotState([](const raisin_sdk::ExtendedRobotState& state) {
        std::cout << "State: " << state.getLocomotionStateName() << std::endl;
        std::cout << "Battery: " << state.voltage << "V" << std::endl;
    });

    // Subscribe to odometry (current position)
    client.subscribeOdometry([](const raisin_sdk::RobotState& state) {
        std::cout << "Position: " << state.x << ", " << state.y << std::endl;
    });

    // Subscribe to pointcloud (map data)
    client.subscribePointCloud([](const std::vector<raisin_sdk::Point3D>& points) {
        std::cout << "Points: " << points.size() << std::endl;
    });

    // Set waypoints for navigation
    std::vector<raisin_sdk::Waypoint> waypoints = {
        raisin_sdk::Waypoint("map_name", 5.0, 0.0),
        raisin_sdk::Waypoint("map_name", 5.0, 5.0),
    };
    client.setWaypoints(waypoints, 1);  // 1 lap

    // Cleanup
    client.disconnect();
    return 0;
}
```

### Control Mode API

```cpp
// Set manual control (gamepad)
client.setManualControl();

// Set autonomous control (patrol mode)
client.setAutonomousControl();

// Release control
client.releaseControl("joy/gui");
client.releaseControl("vel_cmd/autonomy");
```

### Locomotion Control API

```cpp
// Stand up
client.standUp();

// Sit down
client.sitDown();

// Check locomotion state
auto state = client.getExtendedRobotState();
std::cout << "State: " << state.getLocomotionStateName() << std::endl;
// States: SITDOWN_MODE(8), STANDING_MODE(6), IN_CONTROL(7)
```

### Patrol Route API

```cpp
// List saved patrol routes
auto files = client.listWaypointsFiles();
for (const auto& file : files.files) {
    std::cout << "Route: " << file << std::endl;
}

// Load and resume patrol from nearest waypoint
client.loadWaypointsFile("office_patrol");
auto result = client.resumePatrol();
std::cout << "Starting from waypoint " << (int)result.waypoint_index << std::endl;
```

### Mission Status API

```cpp
// Check current mission status (for arrival notification)
auto status = client.getMissionStatus();
if (status.valid) {
    std::cout << "Current waypoint: " << (int)status.current_index << std::endl;
}
```

### Actuator Status API

```cpp
auto state = client.getExtendedRobotState();

// Check for motor errors
if (state.hasActuatorError()) {
    auto errors = state.getActuatorsWithErrors();
    for (const auto& err : errors) {
        std::cerr << "Error: " << err << std::endl;
    }
}
```

## Troubleshooting

### Connection Failed
- Verify raisin_master is running on the robot
- Check network connectivity (same network/subnet)
- Verify robot ID is correct

### SetWaypoints Failed
- Ensure Autonomy plugin is loaded
- Load map first using `setMap()`
- Waypoint frame must match map name

### StandUp/SitDown Failed
- Check current locomotion state
- Ensure robot is in a safe position

### Service Timeout
- Check robot logs for errors

## License

Copyright (c) Raion Robotics. All rights reserved.
