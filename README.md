# 小车建图导航演示

## 功能包

当前演示共 **10 个功能包**：

| 功能包 | 用途 |
|---|---|
| `smartcar_description` | 小车模型和 RViz 配置 |
| `smartcar_gazebo` | Gazebo 仿真世界、车辆生成和传感器桥接 |
| `smartcar_sim_io` | 里程计、IMU 数据适配和速度指令看门狗 |
| `smartcar_cartographer` | Cartographer 二维建图与定位 |
| `smartcar_costmap_plugins` | 未知区域代价插件 |
| `smartcar_navigation` | Nav2 路径规划、跟踪与避障 |
| `smartcar_bringup` | 整体系统启动入口 |
| `smartcar_tools` | 目标位姿发送等辅助工具 |
| `smartcar_hal` | 实机 HAL 启动与参数接口 |
| `smartcar_sensor_hal` | ESP32-S3 雷达/IMU UDP 驱动 |

## 编译并启动仿真

在第一个终端执行以下整段命令，完成编译并启动 Gazebo、RViz、在线建图和导航：

```bash
cd /home/doc049/dev/smartcar_ws && \
source /opt/ros/jazzy/setup.bash && \
colcon build --symlink-install --packages-up-to smartcar_bringup && \
source install/setup.bash && \
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
ros2 launch smartcar_bringup mapping_nav.launch.py
```

## 迷宫仿真

Gazebo 迷宫为 20×20 单元，通道净宽按名义轮距 `0.133 m` 的 6 倍生成，即 `0.798 m`；隔墙厚 `0.081 m`。
机器人从中心单元 `(0, 0)` 米出生，四个外围角点由相交墙段填满。预设终点为 `(7.911, 7.911)` 米；重生成迷宫世界和配套静态栅格图：

```bash
python3 scripts/generate_smartcar_maze.py
```

生成的占据地图分辨率为 `0.003 m/像素`，尺寸为 `5861×5861`；栅格会按该分辨率包络连续 Gazebo 墙体，几何通道宽度保持精确。可用 `--cells N` 指定其他边长。

启动迷宫世界并在线建图/导航：

```bash
ros2 launch smartcar_bringup mapping_nav.launch.py runtime:=sim world:=smartcar_maze.sdf
```

世界文件安装在 `smartcar_gazebo/worlds/smartcar_maze.sdf`，配套占据地图位于
`smartcar_navigation/maps/smartcar_maze.yaml`。静态地图可与该世界生成并保存的 Cartographer `.pbstream`
一起用于定位；在线建图模式会直接使用 Cartographer 发布的 `/map`。

## 实机纯雷达建图

先连接雷达和运行支持发现/订阅协议的 ESP32-S3，并让电脑和 S3 处于同一网络。然后运行：

```bash
ros2 launch smartcar_bringup lidar_mapping.launch.py
```

该入口启动 UDP 传感器接收、Cartographer 和 RViz，以 `laser_link` 为跟踪坐标系，
关闭 Cartographer 的里程计和 IMU 融合；不启动 Agent、Nav2，也不要求实车标定文件。
缓慢平稳地手推雷达移动，观察 RViz 中的 `/map`。这是雷达坐标系下的建图试验，
不提供车辆的运动控制；缺少里程计和 IMU 时，快速移动或结构重复的环境可能产生较大漂移。
UDP 驱动仍使用其默认传感器配置；若驱动已经运行，可传 `start_sensor:=false` 避免重复占用端口。
可传 `device_ip:=实际S3地址` 或 `broadcast_address:=子网广播地址` 调整发现方式。
如果没有扫描数据，检查 `/car/scan` 和 `/diagnostics`；驱动需要发现设备、取得租约并完成时间同步才会发布数据。

## Cartographer 传感器融合开关

`use_odom` 和 `use_imu` 分别控制 Cartographer 是否融合里程计和 IMU，默认均为 `true`。
建图和定位入口均支持独立关闭，例如：

```bash
ros2 launch smartcar_bringup mapping_nav.launch.py use_odom:=false use_imu:=false
# 也可以直接启动 Cartographer
ros2 launch smartcar_cartographer cartographer.launch.py use_odom:=true use_imu:=false
```

这两个开关只影响 Cartographer 的传感器输入；完整导航系统的启动健康检查和运行保护仍要求 IMU、
底盘里程计等数据及相关 TF。关闭融合后仍需提供这些数据，才能启动导航并解除保护。

## 发布目标位姿

另开一个终端，执行以下整段命令。示例目标位于 `map` 坐标系的 `x=1.0 m、y=0.8 m`，航向为 `0.0 rad`；修改最后一行的数值即可更换目标。

```bash
cd /home/doc049/dev/smartcar_ws && \
source /opt/ros/jazzy/setup.bash && \
source install/setup.bash && \
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
ros2 run smartcar_tools navigate_to_pose 1.0 0.8 --yaw 0.0
```

## 同一入口运行仿真与实机

仿真沿用默认入口；切换运行环境只选择启动参数，不更换 Cartographer/Nav2 公共话题和 TF：

```bash
# Gazebo 建图和导航
ros2 launch smartcar_bringup mapping_nav.launch.py runtime:=sim

# 实机建图和导航；必须提供 status: measured 的车辆标定文件
# 默认启动本工作区集成的 smartcar_sensor_hal；如需改网络，另传 hal_params_file
ros2 launch smartcar_bringup mapping_nav.launch.py runtime:=real \
  calibration_file:=/path/to/car01.yaml

# 只启动实机 Agent、HAL 和机器人模型，不启动建图/导航
ros2 launch smartcar_bringup real.launch.py \
  calibration_file:=/path/to/car01.yaml
```

`thirdparty/smartcar_udp_heartbeat/` 保存 UDP 固件、协议、主机工具和验证资料的源码快照；
`src/smartcar_sensor_hal` 是加入本工作区的 ROS 2 驱动，默认经 `smartcar_hal` 接口启动并发布雷达、IMU 和诊断话题。
默认传感器配置面向 `192.168.0.0/24`；其他网络请复制 `src/smartcar_sensor_hal/config/sensors.yaml`，调整 `broadcast_address`，
并向 bringup 传入 `hal_params_file:=/绝对路径/配置.yaml`。本机没有 ROS 包形式的官方 `micro_ros_agent` 时，实机入口会明确提示安装或外部运行后关闭 `start_agent`；
系统中的 `MicroXRCEAgent` 不会被自动当作 ROS 节点替代。

在 Ubuntu 24.04 / ROS 2 Jazzy 上，可使用官方 `micro_ros_setup` 分支创建 Agent 工作区：

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p "$HOME/micro_ros_agent_ws/src"
cd "$HOME/micro_ros_agent_ws"
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update --rosdistro=jazzy
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
ros2 pkg prefix micro_ros_agent
```

该流程对应 [官方 micro-ROS Jazzy 构建说明](https://github.com/micro-ROS/micro_ros_setup/tree/jazzy)。底盘 Agent 默认使用 UDPv4 端口 8891，并由主机在 8892 端口主动发现底盘；这与传感器使用的 8888–8890 分开。可用 `agent_transport`、`agent_port` 和 `chassis_discovery_port` 覆盖，多网卡时可指定 `chassis_interface`。UDP 固件目前已编译，尚待刷写及实机联调，网络配置见[准备记录](docs/dev_log/2026-10-27-microros-udp-preparation.md)。如使用设备上原有的串口固件，指定 `agent_transport:=serial agent_device:=/dev/ttyUSB0 agent_baudrate:=460800`。

实机标定模板在 `src/smartcar_description/config/real.template.yaml`。填写机器人实测外参、方法和来源，记录底盘/IMU
固件及标定版本后，再以 `status: measured` 保存为车辆配置。名义几何和导航 footprint 有范围校验；轮式里程计的有效轮径、
轮距由底盘固件负责，IMU 零偏/轴向由 ESP32-S3 标定模块处理，HAL 不重复应用补偿。模型中的 `xyz/rpy` 使用米和弧度。

导航启动前检查时钟、`/car/scan`、`/car/imu/data_raw`、`/car/chassis/odom`、地图和动态/静态 TF，并保持有效 0.5 秒；超时由 `readiness_timeout` 控制。健康状态发布到 `/diagnostics`。

Nav2、`/car/cmd_vel_checked` 和实车 `/car/chassis/cmd_vel` 使用标准车体速度单位（m/s、rad/s）。实车 `command_guard` 用唯一的 `speed_per_pwm` 按轮距换算为归一化左右轮指令，并发布到内部 `/car/chassis/cmd_pwm`（`linear.x` 左轮、`angular.z` 右轮，范围 `[-1, 1]`）；仿真继续向 Gazebo 输出物理车体速度。底盘固件对归一化轮指令使用正反向 ReLU 开环映射，不运行速度 PID。手动发布 `/car/chassis/cmd_vel` 时发车体速度，不要直接填左右轮值。
