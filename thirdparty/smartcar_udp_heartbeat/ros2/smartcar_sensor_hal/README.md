# smartcar_sensor_hal

Ubuntu 24.04 / ROS 2 Jazzy 的 `ament_python` 包。一个进程接收 ESP32-S3 的 UDP 传感器包，发布：

| 接口 | ROS 2 类型 | frame_id | QoS |
|---|---|---|---|
| `/car/scan` | `sensor_msgs/msg/LaserScan` | `laser_link` | SensorData |
| `/car/imu/data_raw` | `sensor_msgs/msg/Imu` | `imu_link` | SensorData |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 无 TF | 默认深度 10 |

HAL 不发布 `/tf` 或 `/tf_static`。固定安装关系仍由第三个人维护的唯一 `robot_state_publisher` 发布，动态定位 TF 由 Cartographer 发布。

## 前提

- S3 使用本资料包 v0.6.0 或之后支持主动发现/订阅的固件。默认使用数据 UDP 8888、对时 UDP 8889、发现/租约 UDP 8890；对时端口由所选 S3 通告。
- S3、上位机处于可互通的局域网，允许 UDP 广播与上述端口。默认自动发现并订阅；S3 从订阅数据报获知上位机 IP 和数据端口，双方都不需要填写对方 IP。
- 8888 只能由一个接收程序占用；一台 S3 同时只接受一个有租约的订阅者。切换 Mac/Ubuntu 前先退出原接收程序以释放订阅，或等待其约 6 s 租约到期；不要同时启动多个接收程序。
- 运行环境已经安装 ROS 2 Jazzy、colcon 及对应消息包。
- 本包包含 `lidar_protocol.py`、`sensor_protocol.py`、`clock_sync.py`、`discovery.py` 四个与固件协议对应的纯 Python 模块；升级协议时与本资料包工具版本一起更新。旧版无发现服务的固件需要先升级，本 HAL 不提供旧版固定目的 IP 模式。

## 编译与启动

将整个 `smartcar_sensor_hal` 文件夹放入团队工作区 `src/`，然后：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/smartcar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select smartcar_sensor_hal
source install/setup.bash
ros2 launch smartcar_sensor_hal sensors.launch.py
```

默认配置位于 `config/sensors.yaml`，也可传 `config_file:=/绝对路径/配置.yaml`。不填任何 IP 即可启动并等待 S3。发现多台车时不会随意选择，查看 `/diagnostics` 的 `discovery.candidates`，然后按稳定的 12 位十六进制设备 ID 启动：

```bash
ros2 launch smartcar_sensor_hal sensors.launch.py device_id:=0123456789ab
```

如网络屏蔽全局广播但允许子网广播，可显式传 `broadcast_address:=子网广播地址`；知道 S3 IP 时也可传 `device_ip:=实际S3地址` 使用单播发现。手动 IP 仍要完成订阅，不会绕过租约。非空 launch 参数覆盖 YAML，留空时保留 YAML 的对应设置。自动选择后客户端会记住所选设备 ID；其 IP 改变时重新发现同一设备，不会自动换到另一台车。

参数在启动时读取。修改 YAML 后重启节点；运行时修改参数会被明确拒绝，`use_sim_time=false` 除外。启动和运行时都拒绝 `use_sim_time=true`，避免实时采集被仿真时钟误导。

节点启动后先发现、取得订阅租约，再创建时间同步客户端。未发现、发现多台、被其他订阅者占用或续租失效时，节点保留运行并在 `/diagnostics` 解释等待原因，不发布传感器消息。客户端约每 2 s 续租，6 s 租约过期立即停止消费数据；退出时尽力发送 release，不阻塞等待回复。

租约有效后仍需完成时间同步。在同步未锁定、同步过期、启动标识不匹配或同步不确定度大于 `max_sync_uncertainty_ms`（默认 10 ms）时，不发布传感器消息。数据和心跳只接受当前有效租约的来源 IP 与 boot ID；心跳不要求对时已锁定。IP、boot、session 或对时端口变化会清空对时模型、正在组装的圈与包序号；历史已发布时间戳保护仍保留。诊断在所有等待状态继续发布。

订阅端点变化时，HAL 在启用新流前非阻塞清理最多 256 个已经排在系统 UDP 接收队列中的数据报，诊断公开清理数量、预算耗尽和接收错误。设备侧的队列 generation 保护和这次主机清理都用于减少旧租约残留；v1 数据包没有 session 字段，因此不能严格隔绝仍在网络中、稍后才到达的跨租约旧包。主机仍应用当前 IP/boot、数据年龄、序号及时间戳单调保护，不把这些措施描述为完整的端到端 session 隔离。

## 时间戳与可观察误差

S3 启动后单调微秒时钟通过双向 UDP 对时拟合到主机 `monotonic_ns`。采样事件先映射到主机单调时间，再通过主机 ROS 时间与单调时间之间的观测基准转换；不会用主机收包时刻冒充采样时刻。

节点使用 STEADY_TIME 定时器每 2 ms 非阻塞轮询，单轮最多处理 200 个数据报；诊断定时器也使用 STEADY_TIME，使 ROS/系统时钟后跳时仍能继续接收并检测跳变。每轮用 monotonic-before / ROS-now / monotonic-after 夹取当前 ROS 时间，并采用单调区间中点更新转换基准，以跟随系统时间的平滑 NTP slew，避免旧基准偏移不断积累。相邻观测残差超过 50 ms 则视为时钟步变，暂停发布、清空正在组装的数据并重新同步。诊断公开基准残差、观测区间和区间半宽；该半宽加入时间同步质量门限。

已发布时间戳的历史保护值保留；任何不递增的后续时间戳直接丢弃并重新同步，不通过强行加一伪造连续性。使用当前系统时钟偏移转换百毫秒前的扫描首点仍有微小 slew 近似误差；逐点物理时间误差还受下述未标定采样因素限制。

发布前还检查数据年龄：默认拒绝早于主机当前单调时刻 0.5 s 或超前 20 ms 的数据。该检查位于单调保护更新之前，异常未来时间戳不会污染后续保护状态。

### 激光 v1 时间估计

v1 的 `rx_complete_us` 是 S3 软件解析并验证包完成的时刻，未测量雷达内部缓存延迟。使用下列近似：

```text
串口线时间 = UART包字节数 × 10 / 115200
包末采样估计 = rx_complete_us - 串口线时间 - lidar_device_delay_us
包首采样估计 = 包末采样估计 - (包内点数 - 1) / lidar_sample_rate_hz
```

默认 `lidar_device_delay_us=0` 代表未标定，不代表雷达内部无延迟。数据到达延迟、UART 批处理和设备缓存都会影响估计。`lidar_device_delay_calibrated` 默认 false；只有实际标定后才应改成 true。

### IMU 时间估计

IMU 的 `sample_us` 是 S3 轮询 data-ready 的时间区间中点。`uncertainty_us` 描述该轮询区间，并不覆盖传感器滤波器的所有相位误差。

默认 MPU6050 DLPF=3，按原厂表中的加速度 4.9 ms、角速度 4.8 ms 采用共同近似延迟 `imu_filter_delay_us=4850`。节点在映射前扣除此估计。它不是频率无关的精确延迟；未标定的频响相位误差在 diagnostics 中明确标记。不会把轮询区间窄误报为整个测量时间的不确定度。

## 完整圈与角度

- 只有已验证的 UART 包才进入组圈；CT 的 bit0 标识新圈。
- 启动时的首个半圈直接丢弃。收到下一圈边界后，才确认并发布上一完整圈。
- 丢包、重复、乱序、坏包或超时使当前圈失效；下一有效边界重新开始，无需重启 HAL。
- 只根据收到的真实完整圈发布，没有定时器重发旧圈凑频率。默认参考 10 Hz，实际雷达可能约 8–9 Hz；必须实测并和第四个人冻结验收值。
- 原厂正角为顺时针，ROS 角度取负，消息采用 **负 `angle_increment`、`angle_min > angle_max`、正 `time_increment`**。ranges 保持顺时针采集顺序，不反转数组后仍填写正时间增量。
- 默认使用原厂三角雷达二级修正：`atan(21.8*(155.3-distance_mm)/(155.3*distance_mm))`。可通过 `lidar_angle_correction: none` 关闭，用于对照。
- 修正后的角度不均匀，需要重投影到均匀顺时针网格。`scan_bins=0` 使用本圈收到的点数；空栅格填 `inf`，无强度数据时 intensities 为空。
- 距离修正使点落到本圈起点之前、或末栅格之后时，点会丢弃并计数，绝不通过取模移到另一圈时间位置。
- 均匀角网格的正时间增量为近似。诊断公开插值误差，超过 `max_scan_interpolation_error_ms` 默认 10 ms 时报警；不能将这一消息解释为精确逐点硬件时间戳。

Cartographer 的原厂转换实现处理负角增量：[msg_conversion.cc](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/msg_conversion.cc)。仍应在团队使用的实际版本中联调方向、时间和代价地图消费行为。

## IMU 单位与标定

加速度为 m/s²（保留重力），角速度为 rad/s。未提供姿态估计，`orientation_covariance[0] = -1`。

`imu_acceleration_bias_m_s2` 和 `imu_angular_velocity_bias_rad_s` 默认三轴 0，应由第三个人提供实测标定值。不要将静止时的重力分量当成加速度零偏扣掉。`imu_*_covariance` 默认全 0，按消息语义表示未知；没有伪造精度。安装轴变换应与 `imu_link` 的模型定义一致，避免同时在数据和 TF 中重复旋转。

## 运行检查

```bash
ros2 topic info /car/scan --verbose
ros2 topic info /car/imu/data_raw --verbose
ros2 topic hz /car/scan
ros2 topic hz /car/imu/data_raw
ros2 topic echo /diagnostics
ros2 topic echo /car/scan --once --qos-reliability best_effort
ros2 topic echo /car/imu/data_raw --once --qos-reliability best_effort
```

Jazzy 的 `ros2 topic hz` 自带 SensorData QoS，不给它追加不支持的 `--qos-reliability` 参数。

diagnostics 包含实际发布频率、序号缺口、无效/超长数据报、坏圈丢弃、不同步启动标识、时钟 RTT/偏移/漂移/残差/不确定度、非单调或过旧/未来时间戳丢弃、插值误差以及接收预算耗尽计数。扫描 <9 Hz 或 IMU <45 Hz 时会提示低于默认参考的 90%；这不是认定雷达硬件能达到固定 10 Hz。

### 在 RViz 查看雷达扫描

在已安装 ROS 2 Jazzy、具备图形桌面的 Ubuntu 上，用下面一条命令同时启动 HAL 和 RViz：

```bash
ros2 launch smartcar_sensor_hal sensors.launch.py rviz:=true
```

先按“编译与启动”完成环境 source。`rviz` 默认 false；加入参数后加载随包安装的 `config/lidar.rviz`。如果 HAL 已经运行，在另一个已 source 的终端只打开 RViz，避免第二个 HAL 争用 UDP 8888 和订阅租约：

```bash
rviz2 -d "$(ros2 pkg prefix --share smartcar_sensor_hal)/config/lidar.rviz"
```

配置使用 Fixed Frame=`laser_link`，订阅 `/car/scan`，Reliability=`Best Effort`、Durability=`Volatile`，俯视 XY 平面、每格 1 m，青色 3 像素点显示当前一圈。这里展示的是二维 `LaserScan`，无需转换成 `PointCloud2`。固件没有强度数据，因此使用固定颜色。轴线红色为 +X，绿色为 +Y，可用障碍物核对前、左、右与实际安装方向。鼠标滚轮可缩放；若修改了 `laser_frame`，同时修改 RViz 的 Fixed Frame 和轴线 Reference Frame。

初查使用同一个 `laser_link` 坐标系，不需要额外发布 TF。整车模式再切换到第三个人维护的统一静态 TF 和目标坐标系。RViz 没有点时，先检查 `/car/scan` 频率及 `/diagnostics` 是否已对时锁定，再检查显示项状态和 QoS。本配置只在 Mac 上做静态格式检查，尚未实际运行 RViz；需要在 Ubuntu/Jazzy 上确认图形显示。

## 录制与回放

建议用 rosbag 留存标准消息及队友发布的 TF：

```bash
ros2 bag record -o sensor_acceptance /car/scan /car/imu/data_raw /diagnostics /tf /tf_static
ros2 bag info sensor_acceptance
```

停止实时 HAL 后，可用 `ros2 bag play sensor_acceptance` 回放已录制的话题。实时 HAL 仍拒绝仿真时钟；涉及 `--clock` 的仿真/回放由单独启动配置处理。

需要保留原始 UDP 时，在 YAML 设置 `record_jsonl_path`。每行保留主机单调收包时刻、来源 IP/端口和原数据报 base64；记录器后台写盘且队列有界，丢弃数量与写盘错误公开到 diagnostics。此版提供 JSONL 录制，不把历史原始数据直接注入实时 HAL 伪装成当前传感器时间。JSONL 再处理需保留其录制时基和对应同步记录。

## 无 ROS 核心测试

在本包目录运行：

```bash
python3 -m unittest discover -s test -v
```

这些测试可在 Mac 运行，验证组圈、序号缺口/乱序恢复、跨零角展开、负角增量与正时间增量、二级修正边界点不跨圈、超时、启动标识变化、时钟 slew/step 与时间戳保护。`test_node_smoke.py` 使用轻量 ROS 接口替身检查 IMU 字段、超长包丢弃、ACK 异常不终止和参数拒绝行为；它不是实际 ROS 运行测试。这些检查不代替 Ubuntu/Jazzy 上的真实构建、话题、RViz、TF 和实车验收。

分工文档正式稳定性测试仍需同一 AP、距离不超过 3 m 连续运行 10 min，并保留 rosbag 与协议统计；方向和 0.5/1/2 m 测距也需实测。
