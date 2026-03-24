# ROS 2 工作空间 `src/` 目录说明

这是当前工作空间 `src/` 目录的简要说明，重点描述仓库里现在仍然存在、并且实际接入主链路的功能包。

## 快速开始

在工作空间根目录执行：

```bash
colcon build --symlink-install
# Windows PowerShell
.\install\setup.ps1
# Linux/macOS
source install/setup.bash
```

## 当前主链路数据流

- 雷达数据：`bluesea2` 发布 `/scan`。
- 任务目标：`activity_control_pkg` 发布 `/target_position`，并通过 `/active_controller` 指定当前由飞控执行。
- 速度生成：`pid_control_pkg` 订阅 `/target_position`，发布 `/target_velocity`。
- 串口桥接：`uart_to_stm32` 与 STM32/飞控交互，并发布 `/height`、`/is_st_ready`、`/mission_step`。
- 相机节点：`drone_camera_pkg` 订阅 `/height`，发布 `/fine_data` 与 `/apriltag_code`，并在下降越过阈值时保存照片。

## 功能包说明

## `activity_control_pkg`（路线目标发布）

用途概述：管理一串目标点，根据当前位置与高度判断是否到达，并按顺序推进后续目标。

关键文件：

- `activity_control_pkg/include/activity_control_pkg/route_target_publisher.hpp`：定义 `Target` 结构体，以及目标发布节点和测试节点的接口。
- `activity_control_pkg/src/route_target_publisher.cpp`：维护目标点队列 `targets_`，通过 TF 查询 `map -> laser_link` 获取当前位姿（高度来自 `/height`），发布 `/target_position` 与 `/active_controller`，并在达到当前目标后切换到下一个目标。
- `activity_control_pkg/src/route_target_publisher_main.cpp`：`route_target_publisher_node` 的标准入口。
- `activity_control_pkg/src/route_test_node.cpp`：测试入口，使用 `MultiThreadedExecutor` 同时运行目标发布节点与测试节点，并自动添加一组预设目标点。

Launch：

- `activity_control_pkg/launch/route_target_publisher.launch.py`
- `activity_control_pkg/launch/route_test.launch.py`

---

## `bluesea2` / `base_lidar`（蓝海雷达驱动）

用途概述：对接蓝海雷达 SDK，发布 `LaserScan`（以及可选点云），并提供电机控制服务接口。

常用启动文件位于 `bluesea2/src/bluesea-ros2/launch/`，包括 `uart_lidar.launch`、`udp_lidar.launch` 等。

---

## `drone_camera_pkg`（相机预览、AprilTag 检测与阈值拍照）

用途概述：打开相机做实时预览，检测 AprilTag 并发布简化结果，同时根据高度阈值在下降过程中抓拍一张照片。

关键文件：

- `drone_camera_pkg/src/drone_camera_node.cpp`：核心节点，订阅 `/height`，发布 `/fine_data` 与 `/apriltag_code`，并在高度从阈值上方下降穿越阈值时触发一次抓拍保存。

通常通过 `my_launch/launch/demo1.launch.py` 一起启动。

---

## `my_carto_pkg`（建图/定位组合启动）

用途概述：把雷达、URDF、Cartographer 组织成一个启动序列。

关键文件：

- `my_carto_pkg/launch/fly_carto.launch.py`：组合 launch，先启动雷达，再启动 `robot_state_publisher`、Cartographer 和 RViz。

---

## `my_launch`（总控 launch）

用途概述：把建图、串口桥、位置 PID、路线测试和相机节点拼成一套演示流程。

关键文件：

- `my_launch/launch/demo1.launch.py`：启动 `fly_carto`、`uart_to_stm32`、`position_pid_controller`、`route_test_node` 和 `drone_camera_node`。

---

## `pid_control_pkg`（位置 PID -> 速度指令）

用途概述：把“目标位置 + 当前位姿/高度”转换为 `/target_velocity` 速度指令。

关键文件：

- `pid_control_pkg/include/pid_control_pkg/pid_controller.hpp`：定义 PID 控制器与位置控制节点的结构和参数。
- `pid_control_pkg/src/pid_controller.cpp`：订阅 `/target_position`，通过 TF 和 `/height` 计算误差后发布 `/target_velocity`。
- `pid_control_pkg/launch/position_pid_controller.launch.py`：标准启动入口。

---

## `pid_controller`（车体 PID）

用途概述：面向车体底盘的 PID 与轮速计算。当前仍保留在仓库中，但不在 `demo1` 主链路里。

---

## `serial_comm`（串口通信基础库）

用途概述：可复用的 C++ 串口通信库，提供同步/异步读写、超时处理与简单协议帧解析能力。

---

## `uart_to_stm32`（STM32/飞控协议桥）

用途概述：把 ROS 侧速度与状态转换为下位机串口协议，同时把下位机状态转换回 ROS 话题。

关键文件：

- `uart_to_stm32/src/uart_to_stm32_node.cpp`：节点入口。
- `uart_to_stm32/src/uart_to_stm32.cpp`：桥接主逻辑，发布 `/height`、`/mission_step` 等话题，并下发速度指令。
- `uart_to_stm32/launch/uart_to_stm32.launch.py`：标准启动入口。

## 常用启动命令

```bash
# 位置 PID 控制
ros2 launch pid_control_pkg position_pid_controller.launch.py

# 串口桥（高度/任务状态/速度下发）
ros2 launch uart_to_stm32 uart_to_stm32.launch.py

# 组合演示
ros2 launch my_launch demo1.launch.py
```
