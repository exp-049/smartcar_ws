# Smartcar UDP v1：心跳、X3PRO、MPU6050 与时钟同步

固件 `0.6.0` 使用本协议，数据 payload 兼容 `0.5.0`。接收机在 UDP 8888 端口接收 UTF-8 JSON，每个 UDP 数据报是一条完整消息。

0.6.0 新增 UDP 8890 主动发现与租约控制，详见 [DISCOVERY.md](DISCOVERY.md)。设备只向活动订阅者发送传感器数据。
心跳、雷达、IMU 共享目标端口，各有独立 socket 和序号；S3 的 UDP 8889 提供独立二进制对时响应。

## 心跳管理消息

```json
{"type":"heartbeat","version":1,"boot_id":"0123456789abcdef","seq":0,"uptime_us":123456}
```

Mac 保持原行为，回复 `ACK <boot_id> <seq>`，不附加换行。
心跳是网络连通检查，频率约 1 Hz，不代表雷达数据吞吐量。

## 雷达数据消息

```json
{"type":"lidar_packet","version":1,"boot_id":"0123456789abcdef","seq":42,"rx_complete_us":123456789,"data_hex":"aa55a1030100012deb6ba00f401f0000","crc32":"d223eed5"}
```

| 字段 | 类型与含义 |
|---|---|
| `type` | 固定字符串 `lidar_packet` |
| `version` | 整数 1 |
| `boot_id` | 16 个小写十六进制字符，S3 每次启动重新生成，与心跳相同 |
| `seq` | uint32，雷达流独立序号，每个通过原厂校验的 UART 包推进一次 |
| `rx_complete_us` | 0 至 INT64_MAX，S3 对整包校验完成后取得的单调启动时间，单位微秒 |
| `data_hex` | 原始雷达包的完整小写 hex，12 至 520 字节原数据，即 24 至 1040 个字符 |
| `crc32` | 8 个小写十六进制字符，覆盖下面定义的二进制元数据及原始包 |

编码固定为紧凑 JSON，不添加空白或其他字段。即使采用最大字段值与 520 字节原包，总长也仅为 1191 字节（不含结尾 NUL）；上限为 1200 字节。UDP 不发送 C 字符串结尾 NUL。
每个原厂包单独封装，无需应用层分片，也不把一次 UART read 当作一条消息。

雷达包不逐包等待 ACK，不做重传；旧的原始测量不因补发堵塞新数据采集。

## CRC32 的跨语言定义

采用 CRC-32/ISO-HDLC，与 Python `zlib.crc32()` 一致。
先构造 26 字节元数据，然后附加原始雷达包：

```python
metadata = struct.pack(
    '<4sQIQH',
    b'SC1L',
    int(boot_id, 16),
    seq,
    rx_complete_us,
    len(raw_packet),
)
crc32 = f'{zlib.crc32(metadata + raw_packet) & 0xffffffff:08x}'
```

`SC1L` 只属于 CRC 输入的类型标签，不作为另一个 JSON 字段发送。所有多字节数字为小端；禁止直接复制有填充的 C 结构体。
上面的 JSON 是 C/Python 共同测试向量。

## 原厂包的接收验证

`AA 55 CT N FSA_le16 LSA_le16 CS_le16` 后面跟 N 个距离 word，总长 `10 + 2*N`。
接收端核对长度、角度标记、外层 CRC32、原厂 XOR 校验，再解码。
完整 CT 字节参与 XOR，不只取 CT 的最低位。

距离按 `raw_word / 4000.0` 转为米。本阶段按 X3PRO 规格范围 0.12 至 8 米筛选有效距离，零和极小原值不会作为贴脸障碍输出。
角度先按 FSA/LSA 做原始顺时针插值，处理跨 360 度，并明确标记为未应用型号二级角度修正。
该数据不是已转换安装外参及 ROS 坐标约定的最终 LaserScan。

## 时间与丢包语义

- `rx_complete_us` 是软件确认完整包的时间，包含 UART 缓冲和处理延迟，不是激光首束的物理采样时间，也不是 UTC/ROS 时间。
- 主机录制额外保存 `host_receive_unix_ns`，它是主机收包时间，不能冒充传感器采样时刻。
- ROS 2 HAL 对时及采样时间模型见 TIMING.md；网络对时成功不能单独证明物理采样时间或 Cartographer 精度已验收。
- S3 在采集时推进 seq，即使 Wi-Fi 未就绪、队列满或发送失败，seq 也不回退。因此首次看到的雷达 seq 可以不是 0。
- 队列深度为 16；超过 250 ms 的队列项丢弃，避免网络恢复时发送过期积压。
- 主机按来源 IP、boot_id 和消息类型分别跟踪；uint32 模运算处理序号回绕，重复、乱序与观察到的缺口分别计数。
- 缺口是估计，不是严格丢包率；也不能发现 S3 在 UART 采集/校验之前丢掉的字节，需要结合 S3 的 UART 溢出及校验计数。

## 可选录制

接收脚本使用 `--record 文件路径` 时保存有效消息的 JSONL，包含主机接收时间、来源和原消息。数据不包含 Wi-Fi 名称或密码。
Mac 录制保留原始包与设备时钟，供 HAL 开发和验证使用；Mac 接收脚本不发布 ROS 2 话题，ROS 2 包另见 ros2/。

## MPU6050 消息（0.5.0 新增）

```json
{"type":"imu_packet","version":1,"boot_id":"0123456789abcdef","seq":42,"sample_us":123456789,"uncertainty_us":1235,"read_end_us":123458999,"data_hex":"0102fffe4000000080001234fedc","crc32":"dfdfa90e"}
```

| 字段 | 含义 |
|---|---|
| `boot_id` | 与同一次启动的心跳、雷达相同 |
| `seq` | uint32；观察到一次 DATA_RDY 就推进，包括因读取/区间不合格而丢弃的事件 |
| `sample_us` | int64 非负；DATA_RDY 轮询观察区间中点，尚未减去滤波延迟 |
| `uncertainty_us` | uint32；就绪观察区间半宽向上取整，不包含滤波相位和网络误差 |
| `read_end_us` | int64 非负；14 字节寄存器 burst 读取结束时的 S3 时间，不能早于 sample_us |
| `data_hex` | 14 个原始字节，28 个 hex 字符，寄存器 0x3B..0x48；高字节在前 |
| `crc32` | 38 字节小端元数据 + 14 原始字节的 CRC-32/ISO-HDLC |

```python
metadata = struct.pack(
    '<4sQIQIQH', b'SC1I', int(boot_id, 16), seq,
    sample_us, uncertainty_us, read_end_us, 14,
)
crc32 = f'{zlib.crc32(metadata + raw) & 0xffffffff:08x}'
ax, ay, az, temperature, gx, gy, gz = struct.unpack('>7h', raw)
accel_m_s2 = raw_accel * 9.80665 / 16384.0
gyro_rad_s = math.radians(raw_gyro / 131.0)
temperature_c = temperature / 340.0 + 36.53
```

上面的 JSON 是 C/Python 共同 golden。这里的量程固定为 ±2g / ±250 °/s；改量程需要同步修改协议版本或增加被 CRC 保护的配置，不能只改寄存器。
IMU 消息最大 320 字节，不逐包 ACK，不补发旧采样。重力保留，姿态未知；偏置和协方差由第 3 人标定后在 HAL 配置中设置。
无 INT/FIFO 时，seq 不是绝对物理采样计数，无法据此精确恢复漏过的芯片更新次数。

## 软件时钟同步（UDP 8889）

主机主动请求，S3 只接受当前有效订阅者的 IPv4；旧静态模式则使用配置地址。默认每 0.5 秒一次；响应回到请求源端口，端口独立于数据接收端。
请求恰为 20 字节：

```python
struct.pack('<4sQQ', b'SCQ1', nonce_uint64, host_t1_monotonic_ns)
```

响应恰为 44 字节：

```python
struct.pack('<4sQQQQQ', b'SCR1', nonce_uint64, echoed_host_t1_ns,
            device_t2_us, device_t3_us, int(boot_id, 16))
```

S3 在 recvfrom 返回后记 t2，在响应编码和 sendto 前记 t3；上位机在请求发送前记 t1，在响应 recvfrom 后记 t4。
这些时刻均为软件观测，未被伪称为 Wi-Fi 物理收发时间戳。nonce/回显/来源/长度都须匹配；异常、迟到、RTT 过大及负 RTT 样本不入模型。
使用低 RTT 过滤和漂移拟合，不调整 S3 原始时钟。对时通道采用固定二进制长度和随机 nonce 关联，无单独 CRC；不是认证或公网安全协议。
无锁定、启动不匹配、超过 5 秒未有效更新时拒绝转换为 ROS 时间；不得改用主机收包时间兜底。ROS 实时模式的时间基准和物理延迟限制见 [TIMING.md](TIMING.md)。
