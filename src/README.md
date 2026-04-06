# ROS 2 Workspace `src/` Overview

This workspace currently keeps the UAV control path: mapping/localization, waypoint publishing, aircraft PID, the STM32 serial bridge, and `drone_camera_pkg`.

## Quick Start

Run from the workspace root:

```bash
colcon build --symlink-install
# Windows PowerShell
.\install\setup.ps1
# Linux/macOS
source install/setup.bash
```

## Main Data Flow

- `bluesea2` publishes `/scan`
- `activity_control_pkg` publishes `/target_position` and `/active_controller`
- `drone_camera_pkg` publishes `/fine_data` and `/apriltag_code`
- `activity_control_pkg` enters visual takeover for selected waypoints, republishes the target with a configured visual-alignment height, and publishes `/visual_takeover_active`
- `pid_control_pkg` subscribes to `/target_position`, `/height`, `/visual_takeover_active`, and `/fine_data`, then publishes `/target_velocity`
- An external ROS 2 node publishes `/is_fly` as the remote-control enable signal
- `activity_control_pkg` publishes `/visual_aligned_apriltag_code` after visual alignment succeeds
- `uart_to_stm32` listens to `/is_fly` and forwards `/target_velocity` to the flight controller only after receiving `/is_fly=true`
- `uart_to_stm32` sends `/visual_aligned_apriltag_code` as serial frame `0x11`
- `activity_control_pkg` publishes `/mission_complete` after all targets complete
- `uart_to_stm32` sends `/mission_complete` as serial frame `0x66` with payload `0x06`
- `uart_to_stm32` also publishes `/height`, `/is_st_ready`, and `/mission_step`

## Packages

### `activity_control_pkg`

Maintains the waypoint queue, checks whether the current target is reached, and triggers visual takeover when `Target.is_takeover` is `true`.

Visual takeover behavior:

- XY alignment still comes from `/fine_data`
- Z is switched to the configurable `visual_takeover_target_height_cm`
- `height_tolerance_cm` is shared by normal waypoint reach checks and visual-alignment height checks
- `/visual_aligned_apriltag_code` is published only after XY stays aligned for the required frames and height is also within tolerance

Key files:

- `activity_control_pkg/include/activity_control_pkg/route_target_publisher.hpp`
- `activity_control_pkg/src/route_target_publisher.cpp`
- `activity_control_pkg/src/route_target_publisher_main.cpp`
- `activity_control_pkg/src/route_test_node.cpp`

Visual takeover topics:

- `/visual_takeover_active`
- `/visual_aligned_apriltag_code`
- `/mission_complete`

### `drone_camera_pkg`

Runs camera preview and AprilTag detection only, then publishes:

- `/fine_data`: pixel error `[x_px, y_px]`
- `/apriltag_code`: current tag code

### `my_carto_pkg`

Launches lidar, URDF, Cartographer, and RViz together.

Key file:

- `my_carto_pkg/launch/fly_carto.launch.py`

### `my_launch`

Launches the complete demo flow.

Key file:

- `my_launch/launch/demo1.launch.py`

### `pid_control_pkg`

The only PID package left in the workspace. It converts target position and current pose into `/target_velocity`.

Control behavior:

- Normal mode: XY / Z / Yaw use waypoint PID
- Visual takeover mode: XY uses visual PID from `/fine_data`, while Z / Yaw keep using the original PID

Key files:

- `pid_control_pkg/include/pid_control_pkg/pid_controller.hpp`
- `pid_control_pkg/src/pid_controller.cpp`
- `pid_control_pkg/launch/position_pid_controller.launch.py`

### `serial_comm`

Reusable serial communication library used by `uart_to_stm32`.

### `uart_to_stm32`

Bridges ROS topics and the STM32/flight-controller serial protocol.

Remote-control gating:

- `/is_fly` is published by an external ROS 2 node
- `uart_to_stm32` subscribes to `/is_fly` using `std_msgs/msg/Bool`
- after the first `/is_fly=true`, target velocity forwarding stays enabled and is not turned off again
- before that first `true`, `/target_velocity` messages are ignored and not sent to STM32

Key files:

- `uart_to_stm32/src/uart_to_stm32_node.cpp`
- `uart_to_stm32/src/uart_to_stm32.cpp`
- `uart_to_stm32/launch/uart_to_stm32.launch.py`

Current serial frame usage:

- `0x31`: target velocity
- `0x32`: velocity/pose related data
- `0x11`: aligned AprilTag code, payload length `1`, sent `3` times
- `0x66`: mission complete, payload `0x06`, sent `3` times

## Common Launch Commands

```bash
ros2 launch pid_control_pkg position_pid_controller.launch.py
ros2 launch uart_to_stm32 uart_to_stm32.launch.py
ros2 launch my_launch demo1.launch.py
```
