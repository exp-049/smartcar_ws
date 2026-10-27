# ESP32 micro-ROS USB 联调记录

- 日期：2026-10-27
- 底盘：ESP32 V1，USB 串口 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
- 传输：UART0 / micro-ROS XRCE serial，460800 baud，8N1
- 主机：ROS 2 Jazzy 官方 `micro_ros_agent`
- 状态：上下位机话题已连通；本次车辆运动验证未做（电机动力电源已断开）。

## 当前运行方式

Agent 当前在主机运行，USB 线需要保持连接。Wi-Fi 切换不影响这条 USB 串口链路。

如果 Agent 以后退出，在终端按下面顺序加载环境并启动。已有 Agent 运行时不要再启动第二个进程占用同一串口。

```bash
source /opt/ros/jazzy/setup.bash
source /home/doc049/Documents/Codex/2026-10-27/kan/work/micro_ros_agent_ws/install/local_setup.bash
source /home/doc049/dev/smartcar_ws/install/local_setup.bash
ros2 launch smartcar_bringup agent.launch.py \
  agent_transport:=serial \
  agent_device:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  agent_baudrate:=460800
```

## 话题接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/car/chassis/cmd_vel` | `geometry_msgs/msg/Twist` | 主机 → ESP32 | 使用 `linear.x` 和 `angular.z`；首次收到有效消息会自动使能底盘。 |
| `/car/chassis/odom` | `nav_msgs/msg/Odometry` | ESP32 → 主机 | 约 5 Hz，`odom` → `base_link`，包含 ROS epoch 时间戳。 |
| `/car/chassis/odom_valid` | `std_msgs/msg/Bool` | ESP32 → 主机 | 当前里程计是否可信。 |
| `/car/chassis/status` | `std_msgs/msg/UInt64` | ESP32 → 主机 | 状态位、故障、计数和最近速度命令处理结果。 |
| `/car/chassis/phase_edges` | `std_msgs/msg/UInt64` | ESP32 → 主机 | 四个编码器相位边沿计数，每 16 位一个计数器。 |
| `/car/chassis/maintenance` | `std_msgs/msg/UInt8` | 主机 → ESP32 | 仅未使能时处理：`1` 清故障，`2` 清编码器/里程计计数状态。 |

`/car/chassis/status` 解码：bit 0 里程计有效；bit 1/2 左右编码器有效；bit 3 已使能；bit 4 Agent 链路就绪；bit 5 里程计不确定；bits 8–11 故障码；bits 12–13 模式（`3` 为 ROS）；bits 16–31/32–47 左右硬件计数；bits 48–59 收到的速度命令计数；bits 60–63 最近命令结果（`0` 未收到、`1` 字段无效、`2` 使能命令入队失败、`3` 速度命令入队失败、`4` 成功入队）。

## 本次实测

- 串口 Agent 成功发现 ESP32 节点；速度订阅和里程计发布均可见，节点名为 `smartcar_chassis`。
- `/car/chassis/odom` 以约 5.0 Hz 连续发布。115200 baud 时发布和命令处理会被串口带宽限制，因此固件和 Agent 统一调整为 460800 baud。
- 主机发送的 Twist 到达 ESP32：状态累计收到 21 条，最近命令结果为 `4`。这验证了主机到 MCU 的订阅回路；MCU 到主机的 odometry/status 也持续可收。
- 最终状态：odom 有效，左右编码器有效，Agent 链路就绪，模式为 ROS，`armed=0`，故障码 `0`，里程计不确定标志为 `0`。
- 电机动力电源断开时曾发过速度消息，固件因没有检测到车轮运动触发堵转保护。随后在未使能状态执行了清故障和里程计归零；最终状态已恢复正常。电机断电期间没有做实际轮速或 PID 响应验收。
- 新固件保留原 NVS 配置，没有擦除保存的 PID/编码器标定数据。

## 后续实际车辆验证

恢复电机动力前，应先确认车轮安全架空并清出周围区域。`cmd_vel` 首条有效消息会自动使能底盘；重新接入动力后不要用任意非零命令试探。先从很短的中位速度脉冲开始，观察 `/car/chassis/odom`、`/car/chassis/status`、左右轮方向和急停路径，再逐步验证速度闭环。若电机动力仍关闭，保持底盘未使能，不要继续发 Twist，否则无轮速反馈时会再次进入堵转保护。
