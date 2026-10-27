# ESP32-S3 雷达 / MPU6050 / ROS 2 HAL — 0.6.0

这套工程将 X3PRO 和 MPU6050 接到同一块 S3，经 Wi-Fi UDP 上发，在上位机解包、对时并发布 ROS 2 话题。Mac 上先验证原始数据，队友的 Ubuntu 24.04 / ROS 2 Jazzy 运行 HAL。

默认由电脑主动发现、订阅 S3，无需填写电脑或小车 IP。切换电脑时结束原接收程序，再启动新节点，详见 [DISCOVERY.md](DISCOVERY.md)。

| 输出 | 消息 | 参考频率 | frame_id |
|---|---|---|---|
| `/car/scan` | `sensor_msgs/msg/LaserScan` | 实际完整扫描，参考 10 Hz | `laser_link` |
| `/car/imu/data_raw` | `sensor_msgs/msg/Imu` | 名义 50 Hz | `imu_link` |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 1 Hz | 不涉及安装 TF |

LaserScan/Imu 使用 SensorDataQoS。HAL 不发布 TF；第 3 人通过统一 URDF 和唯一 robot_state_publisher 维护固定安装关系，完成轴向、IMU 标定和 Cartographer 接入。

## 1. 当前接线

| 信号 | S3 |
|---|---|
| X3PRO Tx | GPIO44 |
| MPU6050 SCL | GPIO21 |
| MPU6050 SDA | GPIO47 |
| 两传感器 GND | 与 S3 共地 |
| MPU6050 INT | 当前未接，程序使用 DATA_RDY 轮询 |

雷达 PWM 和电源沿用当前已工作的接法；程序不驱动 PWM 或发送雷达 UART 指令。日志使用左侧 OTG 原生 USB，雷达使用 UART1 经 GPIO Matrix 路由到 GPIO44。
IMU 初始化探测 0x68/0x69，并检查 WHO_AM_I=0x68。AD0 当前悬空；模块是否自带固定电平需核对，地址不稳定时应固定 AD0。I2C 上拉应接 3.3 V。

MPU6050 为 ±2g、±250 °/s、DLPF=3、SMPLRT_DIV=19，名义输出 50 Hz。每 2 ms 读状态，只有新的合格 DATA_RDY 数据才上发；读状态频率不是上发频率。

## 2. 本机网络配置

当前 Wi-Fi 为 `Mifi_049`。0.6.0 通过主动发现学习通信地址，自动模式忽略静态 HOST_IPV4；密码通过隐藏输入框保存在本机 `firmware/sdkconfig`，不写进源码包。
S3 需接入该网络的 2.4 GHz 频段，接收机与 S3 必须能够在局域网互通。

下次修改网络，在工程根目录运行：

```bash
python3 tools/configure_network.py --ssid Mifi_049 --mac-dialog
```

也可以运行 `idf.py menuconfig`，填写 SSID、密码，保持自动发现开启。
**切换到 Ubuntu 时，退出 Mac 接收程序，再启动 Ubuntu HAL，无需改 IP 或重新烧录。** 设备同时只接受一个有效订阅。
数据接收端口默认 8888，S3 的对时端口为 8889，发现和订阅端口为 8890。每 2 秒续租，6 秒租约到期后可重新认领。

## 3. 先用 Mac 检查原始数据

按 Control+C 停止旧接收程序，重新启动：

```bash
python3 tools/udp_receiver.py
```

Python 标准库即可，不需要 pip。脚本主动广播发现并订阅 S3，再处理心跳、雷达和 IMU。
多车时用 `--device-id` 选择稳定设备身份；广播受限时可用 `--device-ip` 做单播发现。`--legacy` 兼容旧静态地址固件；`--no-sync` 关闭对时。

想在 Mac 实时查看二维雷达图，先结束旧接收端，再运行：

```bash
python3 tools/udp_receiver.py --view
```

打开 [本机雷达预览](http://127.0.0.1:8765/)。它复用同一接收端和完整圈组装，显示真实最新扫描，不需要在 Mac 安装 ROS。
图上方为雷达前向 +X，左侧为 +Y，网格单位为米；半径可选 1/2/4/8 米。暂停仅冻结画面，后台仍收数据；超过 1 秒无完整扫描会提示中断。
页面只绑定 127.0.0.1，和终端统计共用 UDP 8888，不能同时再开第二个接收端。

在 VS Code 的 ESP-IDF Terminal，按 Control+] 退出旧监视器，从 firmware 目录执行：

```bash
idf.py -p /dev/cu.usbmodem1301 flash monitor
```

端口以 `ls /dev/cu.*` 的实际结果为准。不要重复运行 set-target，否则可能重置本机配置。烧录成功后若停在 waiting for download，松开 BOOT，再短按 RST。

## 4. 成功现象与统计含义

S3 启动日志版本应是 0.6.0，并看到发现服务、MPU6050 的 I2C 地址、GPIO 和 nominal=50Hz。

```text
mpu6050: ready=... captured=... sent=... bracket_drop=... read_boundary_drop=... i2c_err=...
lidar_uart: RX=... B/s total=... valid=... samples=... checksum_bad=... overflow=...
lidar_udp: captured=... sent=... queue_drop=... wifi_drop=... send_err=...
smartcar_udp: TX seq=...; ACK OK
```

计数为累计值，应观察前后差分。联网前 wifi_drop 非零可以解释；持续增长的 i2c_err、UART overflow、queue_drop 或 send_err 需要排查。

Mac 应出现：

```text
LIDAR packets/s=... points/s=... valid_ranges/s=... min_m=... crc_errors=0 ...
IMU packets/s=... accel_m_s2=(...) gyro_rad_s=(...) temp_c=... ready_bracket_half_us=...
SYNC locked=True reason=locked rtt_ms=... drift_ppm=... estimate_uncertainty_ms=...
```

IMU packets/s 应接近 50；静止时加速度模长通常约 9.81 m/s²，角速度应接近零，但是否达标须由第 3 人校准。发送的是原始测量，不提供姿态，未去除重力。
雷达 points/s 结合前阶段日志约 4000，包含无回波点；min_m 是该窗口所有有效点中的最近距离，不代表车头方向。packets/s 是小包数，不是扫描圈数。
SYNC locked 只说明软件时钟映射合格，不证明物理采样时间已精确标定。

可保存原始有效消息：

```bash
python3 tools/udp_receiver.py --record captures/sensors.jsonl
```

## 5. Ubuntu 24.04 / ROS 2 Jazzy 发布

把 `ros2/smartcar_sensor_hal` 整个包交给队友，放入其工作空间 src，然后：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/smartcar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select smartcar_sensor_hal
source install/setup.bash
ros2 launch smartcar_sensor_hal sensors.launch.py
```

需要在 Ubuntu 同时打开准备好的 RViz 俯视扫描图，在该命令后添加 `rviz:=true`。配置使用 `laser_link`、`/car/scan` 和 Best Effort QoS。

具体参数、依赖和操作见 [ROS 2 包说明](ros2/smartcar_sensor_hal/README.md)。
默认启动不需要任何 IP 参数。ROS 节点和 Mac 接收程序共用独占订阅，结束原程序后再切换。

首轮检查：

```bash
ros2 topic info /car/scan --verbose
ros2 topic info /car/imu/data_raw --verbose
ros2 topic hz /car/scan
ros2 topic hz /car/imu/data_raw
ros2 topic echo /diagnostics --once
```

Jazzy 的 topic hz 默认使用 SensorDataQoS。未对时、对时过期、同步误差估计过大或启动标识不符时，节点拒绝给传感器盖伪造时间戳并暂停发布，查看 diagnostics 原因。

## 6. 时间对齐与范围

见 [TIMING.md](TIMING.md)。两路数据共享 S3 时钟，并经低 RTT 往返对时、漂移拟合映射到 ROS 时间；实时模式使用 use_sim_time=false。
MPU6050 没接 INT，就绪时间只可用轮询区间估计；X3PRO 标准包缺少逐束硬件时间戳，只能用串口传输、采样率和设备延迟模型反推。默认 IMU 滤波延迟补偿和雷达设备延迟尚未实机标定，不能承诺亚毫秒物理对齐。
扫描保留时间顺序，角增量为负、时间增量为正；固定角网格及二级修正会引入近似，诊断报告时间插值误差。

## 7. 分工验收

- 方向：RViz 中前、左、右障碍与实物一致，frame 和统一 URDF 对上。
- 测距：对准平墙，在 0.5、1、2 m 记录中心附近有效点的中位数，按分工方案建议误差不超过 0.05 m。
- 连续通信：同 AP、相距不超过 3 m 运行 10 分钟，记录 rosbag 和设备/主机计数；扫描平均频率不低于实际设定的 90%。程序按真实圈发，不能复制旧圈补成 10 Hz。
- 异常：丢包、乱序、截断和坏 CRC 可计数丢弃；受影响的扫描圈丢弃，下一完整圈恢复，不必重启 HAL。设备重启或网络恢复后重新对时。
- 标定和交接：IMU 零偏、加速度尺度、安装 TF 由第 3 人负责，将结果填入 HAL 的偏置/协方差参数；不把六轴静止测量直接当作姿态。

## 8. 验证状态

主机单元测试、C 原生校验、ESP-IDF 编译与本机 UDP 联调记录见 verification/。
当前交付属于可实机联调的实现；最终 MPU6050 50 Hz、无线对时误差、ROS 2 真机发布及十分钟验收必须在相应硬件与 Ubuntu 环境确认。
编译验证使用单独的占位网络配置，不把测试 Wi-Fi 写进开发板。本机真实 sdkconfig 与 build 含配置，源码包按白名单排除，分享源码包即可。

## 文件

- firmware/：统一 ESP-IDF C 固件；当前入口是 main/smartcar_main.c。
- tools/udp_receiver.py：Mac 原始数据、心跳和软件对时诊断。
- tools/lidar_protocol.py、sensor_protocol.py、clock_sync.py：标准库协议与时间核心。
- ros2/smartcar_sensor_hal/：可独立拷贝到 Ubuntu 的 ROS 2 包。
- [PROTOCOL.md](PROTOCOL.md)：网络消息、CRC、时钟请求/响应定义。
- [DISCOVERY.md](DISCOVERY.md)：主动发现、订阅、续租与切换电脑。
- [TIMING.md](TIMING.md)：采样时间估计、ROS 时间映射、误差与实测方法。
