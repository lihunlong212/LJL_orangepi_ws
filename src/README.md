# ROS 2 工作空间 `src/` 目录说明

这是一个 ROS 2 工作空间的 `src/` 目录，包含驱动、通信、控制、视觉识别与组合启动脚本等多个功能包。本文档面向公开阅读，重点说明“每个关键文件在做什么”。

## 快速开始

在工作空间根目录（`src/` 的上一级目录）执行：

```bash
colcon build --symlink-install
# Windows PowerShell
.\install\setup.ps1
# Linux/macOS
source install/setup.bash
```

## 关键数据流（便于快速理解系统）

- 雷达数据：`bluesea2` 发布 `/scan`（以及可选点云）。
- 任务目标：`activity_control_pkg` 发布 `/target_position`，并通过 `/active_controller` 指定当前由“车/飞控”谁来执行。
- 视觉辅助：`opencv01` 发布 `/qr_left/*` 与 `/qr_right/*` 话题，提供“是否对准”和“微调偏移量”。
- 速度生成：`pid_control_pkg` 订阅 `/target_position`，发布 `/target_velocity`（速度指令）。
- 串口桥接：`uart_to_stm32` 将速度与状态通过串口协议与 STM32/飞控交互，并发布 `/height`、`/is_st_ready`、`/mission_step`。
- OpenMV -> 蓝牙链路：`openmv_bridge` 发布 `/openmv_data`，`bluetooth` 处理后发布 `/bluetooth_data` 并回传预测位置。
- 车体驱动：`pid_controller`（车体 PID）发布 `/wheel_speeds`，`car_driver` 下发到电机驱动板。

## 功能包与文件逻辑说明

下面按包列出“关键文件 -> 逻辑效果”。若没有 launch，会明确标注“Launch：无”。

---

## `activity_control_pkg`（路线目标发布与视觉接管）

用途概述：管理一串目标点（航点/路径点），根据当前位置判断是否到达；在接近目标点时可切换到视觉对准与微调逻辑。

关键文件：

- `activity_control_pkg/include/activity_control_pkg/route_target_publisher.hpp`：定义 `Target` 结构体（含 `require_visual_align` 和 `camera_side`），以及两个节点类的接口与关键状态变量。
- `activity_control_pkg/src/route_target_publisher.cpp`：核心逻辑文件，维护目标点队列 `targets_`，通过 TF 查询 `map -> laser_link` 获取当前位姿（高度来自 `/height`），发布 `/target_position`、`/active_controller`（固定为 2，表示 Drone/飞控）与 `/current_target_camera`，并在接近视觉目标时根据 `/qr_left/*` 或 `/qr_right/*` 的对准状态与微调偏移动态发布“微调后的目标点”，对准成功后立即切换到下一个目标。
- `activity_control_pkg/src/route_target_publisher_main.cpp`：`route_target_publisher_node` 的标准入口（初始化并 spin）。
- `activity_control_pkg/src/route_test_node.cpp`：测试/演示入口，使用 `MultiThreadedExecutor` 同时运行目标发布节点与测试节点，并自动按顺序添加一系列目标点（部分目标点要求视觉对准并指定相机侧别）。

Launch：

- `activity_control_pkg/launch/route_target_publisher.launch.py`：启动 `route_target_publisher_node` 并设置容差、坐标系与输出话题参数。
- `activity_control_pkg/launch/route_test.launch.py`：启动 `route_test_node`（用于自动投喂测试航点）。

---

## `bluesea2` / `base_lidar`（蓝海雷达驱动与控制服务）

用途概述：对接蓝海雷达 SDK，发布 `LaserScan`（以及可选点云），并提供电机控制服务接口。

关键文件：

- `bluesea2/src/base_lidar/srv/Control.srv`：定义控制服务格式（请求包含 `topic/func/flag/params`，响应包含 `code/value`）。
- `bluesea2/src/bluesea-ros2/src/node.cpp`：雷达主节点逻辑，读取连接与过滤相关参数，从 SDK 获取数据扇区（fan）并拼接成整圈数据后发布 `/scan`（或点云），同时暴露 `*/_motor` 控制服务（使用 `base_lidar/srv/Control`）。
- `bluesea2/src/bluesea-ros2/src/client.cpp`：命令行客户端，通过调用 `/_motor` 服务实现 start/stop/switchZone/rpm/check 等操作。
- `bluesea2/src/bluesea-ros2/src/heart_check.cpp`：心跳/状态监听节点，提供 `/heart_motor` 服务用于开启/关闭心跳监听线程，并通过 UDP 组播接收心跳帧和打印设备状态信息。

Launch：

- 本包使用 XML/非 Python launch，位于 `bluesea2/src/bluesea-ros2/launch/`。
- 常见文件包括：`udp_lidar.launch`、`uart_lidar.launch`、`heart_check.launch`、`dual_udp_lidar.launch` 等。

---

## `bluetooth`（蓝牙串口读写与目标预测回传）

用途概述：从蓝牙串口读取简单帧并发布；同时订阅 OpenMV 识别结果，在 TF 下估计短时未来位置后回传给下位机。

关键文件：

- `bluetooth/src/bluetooth.cpp`：唯一核心文件，负责打开串口并用 10ms 定时器轮询读取，解析格式为 `[0xAA][data][checksum]` 的简易帧并发布 `/bluetooth_data`，订阅 `/openmv_data` 后要求同一动物类型连续出现 3 次才触发发送，随后通过 TF 估计 `map -> base_link` 的速度并对 0.5s 内位置做多点平均预测，最后将“动物类型 + 预测位置(cm)”打包为自定义帧回写串口。

Launch：无。

---

## `car_driver`（电机驱动板串口适配）

用途概述：把 ROS 中的轮速指令转成驱动板串口协议，并把驱动板返回数据发布回 ROS。

关键文件：

- `car_driver/src/car_driver.cpp`：主驱动节点，通过 Boost.Asio 打开串口并异步读取以 `#` 结尾的报文，解析 `$MAll/$MTEP/$MSPD` 等格式后发布 `motor_speed`，订阅 `/wheel_speeds` 并在 `/active_controller == 1`（Car 模式）时才真正下发速度（否则下发全 0），启动时会根据 `motor_type` 自动下发一组电机参数配置命令。
- `car_driver/src/wheel_speeds_pub.cpp`：测试用轮速发布器，按参数每 500ms 发布一次固定轮速到 `/wheel_speeds`。
- `car_driver/src/USART.py`：独立串口调试脚本，不依赖 ROS，直接用 Python 串口与驱动板通信和打印返回值。

Launch：

- `car_driver/launch/car_drive.launch.py`：同时启动 `car_driver` 与 `wheel_speeds_pub`，并设置串口与电机类型参数。

---

## `my_carto_pkg`（建图/定位组合启动）

用途概述：把雷达、URDF、Cartographer 组织成一个启动序列。

关键文件：

- `my_carto_pkg/launch/fly_carto.launch.py`：组合 launch，先包含雷达 launch（`bluesea2` 的 `udp_lidar.launch`），再读取 `urdf/fly.urdf` 启动 `robot_state_publisher`，最后延时启动 `cartographer_ros/cartographer_node` 并加载 `configuration_files/amphi.lua`。

Launch：

- `my_carto_pkg/launch/fly_carto.launch.py`。

---

## `my_launch`（演示/总控 launch 集合）

用途概述：用几个“场景化 launch 文件”把系统按不同需求拼起来。

关键文件（全部是 launch）：

- `my_launch/launch/camera_only.launch.py`：只启动左右相机二维码识别与微调节点。
- `my_launch/launch/demo2.launch.py`：启动建图（`fly_carto`）+ 串口桥（`uart_to_stm32`）。
- `my_launch/launch/demo1.launch.py`：在 demo2 的基础上再启动 PID 控制与路线测试（部分车体/蓝牙节点已注释）。
- `my_launch/launch/demo3.launch.py`：更完整的任务流，启动建图、串口桥、位置 PID、路线测试节点（并通过参数开启视觉接管），再延时 1 秒启动左右相机识别与微调节点以减少 TF/串口初始化竞争。

Launch：

- `my_launch/launch/camera_only.launch.py`
- `my_launch/launch/demo1.launch.py`
- `my_launch/launch/demo2.launch.py`
- `my_launch/launch/demo3.launch.py`

---

## `opencv01`（二维码识别与视觉微调）

用途概述：识别二维码，给出“是否对准 + 偏移量”，并把偏移量转换成控制友好的“机体系微调量（cm）”。

关键文件：

- `opencv01/opencv01/decoder_common.py`：二维码识别基类（核心），打开摄像头并按 `decode_interval` 降频解码，发布 `{prefix}/id`、`{prefix}/offset_norm`、`{prefix}/aligned`、`{prefix}/debug_image`，订阅 `/current_target_camera` 以只在匹配的相机侧别上工作，支持 GPIO 激光控制（二维码 ID 变化且进入对准窗口时触发一次激光脉冲线程），并支持运行时动态开关 GUI 窗口显示。
- `opencv01/opencv01/decoder_right.py`：右相机入口，使用前缀 `/qr_right`，默认设备 `/dev/video0`，激光引脚设为 10。
- `opencv01/opencv01/decoder_left.py`：左相机入口，使用前缀 `/qr_left`，默认设备 `/dev/video2`，激光引脚设为 13。
- `opencv01/opencv01/qr_fine_tune.py`：视觉微调节点，订阅 `{input_prefix}/offset_norm` 与 `{input_prefix}/aligned`，对偏移做死区、EMA 滤波、步长限制与限幅后输出 `{output_topic}`（机体系 cm 级微调量），且在已对准时可持续输出 0 并重置内部状态以避免历史残留。
- `opencv01/opencv01/decoder.py`：较早期的单节点识别脚本，直接读取摄像头并发布 `/qr_processed_image`、`/qr_code_data`、`/qr_code_position`。

Launch：无（通常通过 `my_launch` 来启动）。

---

## `openmv_bridge`（OpenMV 串口数据桥）

用途概述：从 OpenMV 串口读取识别结果，校验后发布到 ROS。

关键文件：

- `openmv_bridge/src/openmv_bridge.cpp`：唯一核心文件，使用底层 POSIX 串口读取文本行，期望行格式形如 `0xaa 0x5 0xaf`（header payload checksum），校验通过后发布 `/openmv_data`（`UInt8MultiArray`，通常只放一个识别类别编号）。

Launch：无。

---

## `pid_control_pkg`（位置 PID -> 速度指令）

用途概述：把“目标位置（cm/deg）+ 当前位姿（TF + 高度）”转换为速度指令 `/target_velocity`。

关键文件：

- `pid_control_pkg/include/pid_control_pkg/pid_controller.hpp`：定义 PID 控制器与位置控制节点的结构、状态与参数。
- `pid_control_pkg/src/pid_controller.cpp`：核心控制逻辑，订阅 `/target_position`（4 元组：x_cm, y_cm, z_cm, yaw_deg），通过 TF 获取 `map -> laser_link` 的当前 XY 与 yaw（高度来自 `/height`），XY 方向采用“距离 -> 速度”的策略再按方向分解成 vx/vy，yaw 与 z 分别用独立 PID 控制，发布 `/target_velocity`（4 元组：vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s），并在目标高度非 0 但尚未收到高度数据时抑制 z 速度并打印节流警告。
- `pid_control_pkg/launch/position_pid_controller.launch.py`：标准启动入口与参数模板。

Launch：

- `pid_control_pkg/launch/position_pid_controller.launch.py`。

---

## `pid_controller`（车体 PID 与麦克纳姆轮速计算）

用途概述：面向车体底盘的 PID 控制与轮速计算（与 `car_driver` 对接）。

关键文件：

- `pid_controller/src/pid_controller.cpp`：通用 PID 类，提供死区与输出限幅。
- `pid_controller/src/control_node_lifecycle.cpp`：当前真正有逻辑的控制节点，这是一个 Lifecycle 节点，会在启动后自动 `configure()` 再 `activate()`，订阅 `/target_position` 与 `/active_controller` 且仅当 `active_controller == 1` 时输出控制，使用 TF `odom -> base_link` 获取当前位姿，先算全局误差再转到机体系误差并用 PID 计算机体系速度，最终发布 `target_velocity`（Twist）与 `/wheel_speeds`（四轮速度）。
- `pid_controller/src/control_node.cpp`：当前为占位文件（主逻辑已注释，`main()` 直接返回 0）。
- `pid_controller/src/target_pub.cpp`：当前为占位文件（主逻辑已注释，`main()` 直接返回 0）。

Launch：无。

---

## `serial_comm`（串口通信基础库与协议解析）

用途概述：一个可复用的 C++ 串口通信库，带同步/异步读写、超时处理与简单协议帧解析能力。

关键文件：

- `serial_comm/src/serial_comm.cpp`：库实现，封装 Boost.Asio 串口初始化与参数配置，提供同步 read/write、带超时 read/read_line、异步读写接口，并提供“协议帧”能力（构建带 `header/address/id/len/data/sum/add` 的帧并在接收侧做校验与回调分发），`uart_to_stm32` 直接依赖该库进行协议通信。

Launch：无（这是库，不是节点）。

---

## `uart_to_stm32`（与 STM32/飞控的协议桥接）

用途概述：对接下位机串口协议的核心桥接节点，负责把 ROS 侧速度与状态转换为协议帧，同时把下位机状态转换回 ROS 话题。

关键文件：

- `uart_to_stm32/src/uart_to_stm32_node.cpp`：节点入口，声明并读取 `update_rate/source_frame/target_frame` 参数，创建 `UartToStm32` 实例并初始化。
- `uart_to_stm32/src/uart_to_stm32.cpp`：桥接主逻辑，使用 `serial_comm` 打开 `/dev/ttyS6`（921600），订阅 `/velocity_map` 与 `/target_velocity`，通过 TF 获取 yaw 并把速度从全局系旋转到机体系后再发送，解析下位机协议帧（`0xF1` 发布 `/mission_step` 且在满足条件时发布一次 `/is_st_ready=1`，`0x05` 发布 `/height`），并在收到 `/active_controller == 2`（Drone 模式）时连续发送 3 次 A2 ready 响应帧。
- `uart_to_stm32/launch/uart_to_stm32.launch.py`：标准启动入口。
- `uart_to_stm32/launch/total.launch.py`：一键组合启动（会包含雷达、Cartographer、串口桥、PID 控制）。

Launch：

- `uart_to_stm32/launch/uart_to_stm32.launch.py`
- `uart_to_stm32/launch/total.launch.py`

---

## 常用启动命令（建议从这里开始）

```bash
# 仅视觉识别链路
ros2 launch my_launch camera_only.launch.py

# 位置 PID 控制
ros2 launch pid_control_pkg position_pid_controller.launch.py

# 串口桥（高度/任务状态/速度下发）
ros2 launch uart_to_stm32 uart_to_stm32.launch.py

# 组合演示（更完整的链路）
ros2 launch my_launch demo3.launch.py
```

## 公开发布前的建议检查项

- 补全多个包 `package.xml` 中的 `description / maintainer / license`（目前不少仍是 TODO）。
- 明确每个包依赖的硬件设备名（例如 `/dev/ttyS6`、`/dev/video0`、GPIO 引脚号），并在 README 中标注“需要根据设备实际修改”。
- 如果要面向外部使用者，建议为关键话题补一张“话题-消息-来源/去向”表格。
