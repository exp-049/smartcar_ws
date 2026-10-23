# 小车建图导航演示

## 功能包

当前演示共 **8 个功能包**：

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

## 发布目标位姿

另开一个终端，执行以下整段命令。示例目标位于 `map` 坐标系的 `x=1.0 m、y=0.8 m`，航向为 `0.0 rad`；修改最后一行的数值即可更换目标。

```bash
cd /home/doc049/dev/smartcar_ws && \
source /opt/ros/jazzy/setup.bash && \
source install/setup.bash && \
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
ros2 run smartcar_tools navigate_to_pose 1.0 0.8 --yaw 0.0
```
