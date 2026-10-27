# 两传感器的时间对齐与精度边界

## 本次实现的时间路径

雷达和 MPU6050 都先用同一个 S3 `esp_timer_get_time()` 单调时钟记时。
设备时间不会因 Wi-Fi 延迟、上位机时钟校正而改变；每次 MCU 重启用新的 `boot_id` 区分时间起点。
上位机通过独立 UDP 8889 对时，拟合 S3 微秒到上位机单调纳秒的仿射关系，再映射到 ROS 时间。
收到 UDP 的时刻只用于链路诊断，不直接作为传感器消息的 `header.stamp`。

```text
传感器事件估计时间（S3 us）
  -> host_monotonic_ns = a * device_us + b
  -> ROS_ns = host_monotonic_ns + 当前观测的 ROS/单调时钟基准差
```

`a` 接近 1000 ns/us，用持续对时估计晶振漂移。两路数据共用这一份映射，不能分别用“本次收包时间减一个常数”形成两个漂移的时间轴。
ROS 使用实时系统时间，即 `use_sim_time=false`；实时采集节点不把墙上时间直接混进仿真 `/clock`。
节点每轮用 monotonic-before / ROS-now / monotonic-after 夹取 ROS 时间，以单调区间中点更新基准，跟随系统时钟平滑 NTP slew；观测区间半宽也加入同步质量门限。
相邻观测残差超过 50 ms 时视为时钟步变，重新对时并丢弃旧圈，防止跨时钟时期拼帧。诊断公开基准残差及观测区间。
发布前拒绝过旧（默认 0.5 s）、过分超前（默认 20 ms）或不递增的时间戳，不用改写时间戳掩盖异常。

## 往返对时及它能证明什么

一次交换记录：上位机发送 `h1`，S3 接收 `d2`，S3 发送 `d3`，上位机接收 `h4`。
主机时刻是 monotonic ns，设备时刻是 monotonic us。四个时间点都是用户态软件时间戳，并非无线射频或内核时间戳。

```text
RTT_ns = (h4 - h1) - 1000 * (d3 - d2)
device_mid_us = (d2 + d3) / 2
host_mid_ns = (h1 + h4) / 2
```

按低 RTT 样本筛选，使用有限窗口稳健拟合；至少 5 个合格样本且覆盖至少 1 秒才能锁定。
每 0.5 秒继续交换，超过 5 秒没有有效模型更新时拒绝转换；nonce、回显、来源 IP/端口、长度均检查。
设备重启重新锁定，旧启动的迟到回复不能把时钟切回去。
诊断给出 RTT、漂移 ppm、拟合残差和统计误差估计。ROS HAL 还对过大的同步统计误差拒发。

**统计误差估计不是总误差保证。** Wi-Fi 两个方向的不对称延迟会留下无法仅靠往返交换分离的固定偏差。
例如前向 1 ms、反向 9 ms，即使总 RTT 稳定、拟合残差很小，中点估计仍可能偏约 4 ms。
软件计时分辨率为微秒，不等于物理采样或端到端对齐达到微秒精度。

公式依据 [NTP 原始规范](https://www.rfc-editor.org/rfc/rfc5905)。MCU 计时接口见 [ESP Timer 文档](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/api-reference/system/esp_timer.html)。ROS 实时与仿真时钟关系见 [ROS 2 Clock and Time](https://design.ros2.org/articles/clock_and_time.html)。

## MPU6050：当前没有 INT

已确认 SCL=GPIO21、SDA=GPIO47，INT/AD0 等其余信号未接。程序探测 0x68 和 0x69；地址探测不能代替稳定的 AD0 电平，如地址不稳定，应核对模块自带上下拉并固定 AD0。

芯片配置 ±2g、±250 °/s、DLPF=3、SMPLRT_DIV=19。DLPF 启用时内部基准 1 kHz，输出名义为 1000/(19+1)=50 Hz。
每 2 ms 轮询 DATA_RDY；这是读状态的频率，网络仅对新的合格测量上发一次。
一次连续 I2C burst 读取 14 字节，保持加速度、温度和陀螺仪来自同一次寄存器更新。

上一次确认未就绪的事务起始、首次确认就绪的事务结束，组成就绪时刻的保守观察区间。
`sample_us` 取中点，`uncertainty_us` 是该区间的半宽向上取整；区间超过 6 ms、I2C 失败或读取可能越过下一次名义更新时丢弃并计数。
在任务严重延迟或断线时，不能从无 FIFO/INT 的寄存器接口恢复漏掉的每次采样；seq 只记录观察到的 ready 事件，不能当作绝对硬件样本计数。

DLPF=3 的资料给出加速度约 4.9 ms、陀螺仪约 4.8 ms 延时。
网络保留原始就绪估计时间；ROS 参数 `imu_filter_delay_us` 默认 4850，作为共同时间的近似补偿。
它不是逐频率精确相位校正，也不包含在 `uncertainty_us` 的观察区间中。两组分量共享一个 Imu header，本身仍有约 0.1 ms 的名义延时差以及实际滤波相位误差。
要进一步改善，应增加 MPU6050 INT 到已核实空闲的 S3 GPIO，由 DATA_RDY 中断打时间戳；中断仍标记数据就绪，需要考虑滤波延时。

寄存器、分频、burst 一致性和延时表依据 [InvenSense 原厂 Register Map，Sparkfun 保存的副本](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf)，第 12、13、27、29–32 页。

## X3PRO：当前没有逐点硬件时间戳

现有标准 AA55 距离包含角度、距离、圈标记，没有每一束激光的物理采样时间戳。
`rx_complete_us` 是 S3 软件验证出完整包的时刻；本次降低 UART 事件阈值并取消等待凑齐 512 字节的读取方式，以减少人为缓冲延迟。
它仍包含 UART 驱动缓冲、任务调度和设备内部处理延迟。

当前估算为：

```text
packet_last_us ≈ rx_complete_us - raw_packet_bytes * 10 / baud * 1e6
                 - lidar_device_delay_us
packet_first_us ≈ packet_last_us - (N - 1) * 1e6 / sample_rate_hz
```

默认 baud=115200、sample_rate_hz=4000 来自用户提供的这份 X3PRO 规格书。10 是 8N1 的每字节线路位数。
该模型假定包在采集后发送，并不能证明设备内部打包过程正好符合这个时序；固定设备延迟默认 0 表示未标定，**不是已知没有延迟**。
串口线上的间隙、芯片内部缓存、型号差异、真实采样频率误差仍会造成偏差。
必须通过实测辨识 `lidar_device_delay_us` 和采样频率；仅通过 UDP 对时不能恢复数据中不存在的硬件信息。

按圈标记组装完整扫描，序号缺口/坏包/乱序使受影响的一圈失效，随后圈边界自动恢复。
扫描保持物理采集顺序：原始顺时针角度转换为 ROS 坐标角后，`angle_increment` 为负、`time_increment` 为正。
不能把 ranges 数组倒序后仍填正的时间间隔，这会让运动补偿使用相反的采样顺序。
固定角网格的二级角度修正和重采样会引入额外近似，不能把其 `time_increment` 宣称为逐束硬件真值。
`LaserScan.header.stamp` 标记输出第一束的估计时间，`scan_time` 用真实相邻圈间隔，不能通过重复旧圈凑 10 Hz。

标准包和二级角修正参考 [YDLIDAR 原厂协议](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/YDLidar-SDK-Communication-Protocol.md)。消息时间和角度约定见 [ROS 2 LaserScan.msg](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/LaserScan.msg)。Cartographer 官方 [msg_conversion.cc](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/msg_conversion.cc) 支持负角增量；仍应核对队友实际安装版本及 Nav2/RViz 的消费结果。

## 时间对齐不等于把两条流强行配成同频

IMU 名义间隔 20 ms，雷达一圈约 100–200 ms。各自按原始时间发布到同一 ROS 时间轴，由建图器读取覆盖这段扫描的 IMU 序列。
不把每圈全部激光点盖成同一个“收到这一圈”的时刻，也不把最近的一个 IMU 点复制到整圈。
节点不先调用 ApproximateTime 再丢掉其余 IMU；同步器只配消息，不能纠正采样时间偏差。
需要运动补偿时使用逐束估计时间与 IMU 插值/建图器的去畸变逻辑；应保留全部 50 Hz IMU 数据。

## 如何验收及升级

1. 先在静止状态录制至少 60 秒：查看 IMU 实测频率、时间间隔、角速度零偏和加速度模长；静止数据不能单独证明两个传感器的时间偏差。
2. 同 AP、距离不超过 3 m 连续 10 分钟，记录 rosbag 与 `/diagnostics`、S3 UART/I2C/队列/发送统计。平均扫描频率至少达到实际设定的 90%，不要只看某一秒。
3. 对着固定墙体做可控左右转动，比较雷达方位变化与 gyro_z 的时间关系，估计固定延迟及抖动；必须先由第 3 人核对轴向、安装 TF 和陀螺零偏。
4. 重启 MCU、断开/恢复 Wi-Fi、注入坏 CRC/乱序/截断数据，检查重新锁定及完整圈恢复。S3/上位机时间跳变不能拼接成伪连续数据。
5. 报告中分开列出就绪观察区间、网络对时统计、激光采样模型误差和安装/滤波延时。实测前不给出“误差小于 1 ms”的保证。

如果项目要求可证明的亚毫秒物理对齐，优先升级到支持硬件采样时间戳/外触发的雷达、IMU DATA_RDY 接线，以及共享 PPS/触发或有硬件时间戳的同步网络；单纯把 UDP 改成 TCP 或提高上发频率无法获得缺失的物理时间戳。
