# 开发日志：仿真与实机统一接入（首轮）

- 日期：2026-10-26
- 分支：`main`
- 基线：`61e7cd2`
- 状态：本日志和实现均属于未提交的工作树改动；尚无本轮 commit hash。
- 设计依据：[《五人分工与验收方案》](../latex/五人分工与验收方案.pdf)

## 目标

为同一套 Cartographer＋Nav2 栈增加仿真/实机运行切换，预留 ESP32-S3 HAL 和 micro-ROS Agent 接入位置，保持公共传感器话题及 `map → chassis_odom → base_link` TF 链路。本轮不实现未冻结的 UDP 字节协议，也不修改 ESP32 固件。

## 改动内容

### HAL、Agent 与启动入口

- 新增 `src/smartcar_hal` ROS 包，包含 HAL 参数模板、启动入口、接入说明和启动前配置检查。未提供实际驱动时报告“HAL 尚未配置”，不生成或发布模拟传感器数据。
- HAL 接口约定输出 `/car/scan`（`LaserScan`，`laser_link`）和 `/car/imu/data_raw`（`Imu`，`imu_link`）；安装 TF 仍由机器人描述发布。
- 新增 Agent 和实机启动入口。Agent 默认使用 `udp4:8888`，可配置传输方式和端口；仅解析官方 `micro_ros_agent` ROS 包，不把系统中的通用 `MicroXRCEAgent` 当作替代品。`start_agent`、`start_hal` 可关闭，以接入外部已运行进程。
- 建图和定位导航入口增加 `runtime:=sim|real`，默认 `sim`。仿真入口继续启动 Gazebo；实机入口启动模型、HAL、Agent 和公共算法。
- README 记录 ROS 2 Jazzy Agent 的安装流程，见[官方 `micro_ros_setup` Jazzy 分支](https://github.com/micro-ROS/micro_ros_setup/tree/jazzy)。

### 模型、标定与公共配置

- 新增名义模型配置和实机标定模板。保留现有几何数值作为仿真名义值；实机模板标为未测量，需填写来源、测量方法、底盘/IMU 固件与标定版本后才可启动实机运行。
- Xacro 按标定配置读取底盘、雷达和 IMU 的完整安装外参；`use_gazebo` 控制 Gazebo 插件和传感器是否加入模型。
- Cartographer 保留公共估计配置，并为实机定位配置少量覆盖项。Nav2 统一应用 `use_sim_time` 和机器人 footprint 覆盖到内嵌 costmap。
- 标定职责：安装外参进入机器人模型；轮径、轮距等有效底盘参数由固件负责；IMU 补偿由第 3 人的 MCU 标定模块完成，HAL 不重复扣除。

### 就绪检查与速度安全链路

- 以数据、采集时间、ROS 时钟、地图和动态/静态 TF 检查替代固定等待。就绪状态稳定 0.5 秒后才启动 Nav2；超时由 `readiness_timeout` 控制。
- 增加共享健康检查和 `wait_for_ready`，校验传感器内容、帧名、时间戳、数据新鲜度、地图和 TF。
- 将 `/car/chassis/cmd_vel` 设为唯一公共速度输出，由 `command_guard` 控制；输入经碰撞监控后进入保护节点。输入超时、传感器/TF 故障或无效命令时发布零速，健康故障会锁定输出并尝试取消活动目标。
- 数据恢复后须停车并调用 `/car/resume_commands`，然后重新提交目标。保护节点在 SIGINT 时先发零速再退出。
- HAL 接口文档要求底盘固件独立订阅 `/car/chassis/cmd_vel`，并实现不大于 0.5 秒的指令看门狗，覆盖 ROS 进程退出或 Agent 链路中断。

## 验证记录

- `colcon build --symlink-install --packages-up-to smartcar_bringup`：9 个包编译完成。
- `python3 -m pytest -q src/smartcar_hal/test src/smartcar_bringup/test src/smartcar_tools/test`：11 项通过。
- `python3 scripts/validate_system.py`、全包 Python `compileall`、`git diff --check`：通过。
- Xacro 展开检查：仿真模型包含 2 个 Gazebo 插件，实机模型包含 0 个。
- 实机预检查：分别验证缺少 HAL、缺少官方 Agent、标定文件路径错误都会在启动时明确失败；用仅供测试的合成标定及 `start_hal:=false start_agent:=false` 验证机器人模型可启动且不会拉起内部 HAL/Agent。
- 隔离 ROS domain 的测试发布者验证命令保护流程：健康时转发速度；传感器中断后输出零速；数据恢复后继续锁定；停车并调用恢复服务后接受新命令。流程通过。
- `command_guard` 收到 SIGINT 后以退出码 0 退出；零速清理路径通过。
- Gazebo 集成回归：就绪检查通过，Nav2 生命周期节点激活；旋转、导航均通过。

| 回归 | 结果 |
|---|---|
| 旋转 | IMU/编码器角速度中位比 `0.9991`；最大定位航向误差 `0.01447 rad`（约 `0.83°`）；最大平移漂移 `0.00030 m` |
| 导航到 `(1.0, 0.8, 0.0)` | 位置误差 `0.0731 m`；航向误差 `0.0306 rad`；最终速度为零；目标容差通过 |

详细结果文件：[`rotation/summary.json`](../../work/validation/rotation/summary.json)、[`navigation/summary.json`](../../work/validation/navigation/summary.json)。

## 当前交付边界

- 尚无 ESP32-S3 UDP 协议、真实 HAL 驱动、官方 `micro_ros_agent` 安装和实测车辆标定。因此本轮不宣称实机运行已经验收；实机入口会在缺少这些条件时阻止启动。
- 协议交付后需继续验证传感器时间同步、QoS、标定版本、单一 TF 链路和底盘超时停车，再做低速闭环导航。
- 仿真旋转和导航结果分别保存在 `work/validation/rotation/` 与 `work/validation/navigation/`，供后续对比使用。

## 工作区附带差异

当前 `git diff` 还包含 `src/smartcar_description/rviz/smartcar_nav.rviz` 的完整界面状态序列化差异（文件时间为 2026-10-26 15:46）。它不改变本文描述的运行时接口；纳入正式提交前应由维护者确认是否保留。

## 后续补充：Nav2 规划可视化（2026-10-26）

- 保留全局路径 Path 显示 `/plan`，新增全局代价地图 Map 显示 `/global_costmap/costmap`。
- 将原来无对应发布者的 `/local_plan` 显示改为 MPPI 最优局部轨迹 `/optimal_trajectory`。启用 `FollowPath.visualize` 后，MPPI 才会发布这个 Path；RViz 不订阅候选 MarkerArray。
- 更新 `scripts/validate_system.py`，检查 MPPI 可视化参数和三个 RViz 话题。
- 隔离仿真 ROS domain 实测发现话题名为 `/optimal_trajectory`（消息 frame 为 `chassis_odom`），`/plan` 为 `nav_msgs/Path`，`/global_costmap/costmap` 为 `nav_msgs/OccupancyGrid`；确认显示项使用实际发布的话题。

## 后续补充：三倍轮距迷宫（2026-10-26）

- 按名义配置轮距 `0.133 m` 生成迷宫，通道净宽为 `0.399 m`（精确三倍轮距），墙厚 `0.081 m`。
- 迷宫现为 `20×20` 单元，Gazebo 世界为 `smartcar_maze.sdf`；机器人起点 `(0,0)`，预设终点 `(4.32,4.32)`，生成路线共 174 个单元步。
- 四边围墙在四角相交并填满外围角点；静态占据地图贴合外围墙体边界，四角像素标为占据。
- 同步生成 Nav2 静态地图 `smartcar_maze.yaml/.pgm`，分辨率 `0.003 m/像素`，尺寸 `3200×3200`；地图与 SDF 墙体位置由同一生成器产出。
- 使用 `python3 scripts/generate_smartcar_maze.py` 可重生成世界与地图，`--cells N` 可指定其他边长；仿真启动命令：`ros2 launch smartcar_bringup mapping_nav.launch.py runtime:=sim world:=smartcar_maze.sdf`。
- 本次已重新生成 SDF 和静态地图；源码契约检查、SDFormat 解析和 9 个包构建通过。独立 map_server 实测发布尺寸 `3200×3200`、分辨率 `0.003 m`；20×20 版本尚未重新执行整套 Gazebo/Nav2 运行回归。

## 后续补充：迷宫通道扩至四倍轮距（2026-10-26）

- 通道净宽由三倍轮距调整为四倍：`4 × 0.133 m = 0.532 m`；墙厚维持 `0.081 m`，20×20 布局与四角填充保留。
- 预设终点随单元间距调整到 `(5.517,5.517)` 米，静态地图更新为 `4088×4088`，分辨率仍为 `0.003 m/像素`。
- Gazebo SDF 中的通道几何保持精确 `0.532 m`；栅格地图按 3 mm 分辨率包络连续几何。
- 源码契约检查、SDFormat 解析、9 个包构建通过；独立 map_server 实测发布尺寸 `4088×4088`、分辨率 `0.003 m`。

## 后续补充：迷宫通道扩至六倍轮距（2026-10-26）

- 通道净宽调整为六倍轮距：`6 × 0.133 m = 0.798 m`；墙厚保持 `0.081 m`，20×20 单元和四角填充保持不变。
- 预设终点更新为 `(7.911,7.911)` 米，静态地图更新为 `5861×5861`，分辨率为 `0.003 m/像素`。
- 源码契约检查、SDFormat 解析、9 个包构建通过；独立 map_server 实测发布尺寸 `5861×5861`、分辨率 `0.003 m`。
