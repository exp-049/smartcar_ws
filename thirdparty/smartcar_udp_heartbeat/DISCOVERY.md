# 0.6.0：电脑主动发现与订阅

## 使用结果

S3 只需要 MiFi 的 2.4 GHz Wi-Fi 名称和密码，连上后通过 DHCP 获取自己的地址。
Mac 接收程序或 Ubuntu ROS 2 HAL 启动后主动寻找设备，收到设备身份回复后建立订阅。
S3 从订阅请求的来源地址学习电脑 IP，从请求中取得数据端口；雷达、IMU、心跳数据仍是单播。
只要在同一可互通局域网，电脑或设备地址改变后可以重新发现，无需把新 IP 写入固件。

## 启动

Mac：

```bash
python3 tools/udp_receiver.py --view
```

Ubuntu ROS 2 Jazzy：

```bash
ros2 launch smartcar_sensor_hal sensors.launch.py rviz:=true
```

默认不需要 `--device-ip` 或 `device_ip:=...`。GUI 图像暂停只冻结画面，不释放订阅；切换电脑时应结束原接收程序。

## 发现、占用与恢复

1. 电脑向 UDP 8890 广播短发现请求，带随机 nonce。
2. S3 单播回复 nonce、稳定 MAC 设备 ID、本次 boot_id、对时端口和能力标志。
3. 电脑搜集候选约 0.7 秒。只有一辆车时自动订阅；多辆车时显示候选，等待指定 device_id。
4. 订阅建立 6 秒租约，每 2 秒续租。请求的来源 IPv4 和随机 session 共同识别订阅者。
5. 有效租约拒绝另一接收程序接管；原程序退出会尽力释放。异常断开后租约到期即可重新认领。
6. 收到的 offer/ACK 必须匹配来源端口、nonce、设备身份、boot 与 session。迟到旧回复不能延长当前租约。
7. 设备重启、IP 改变、租约失效时停止使用旧端点，重新发现/订阅；重新建立时钟映射并清空未完成扫描。

设备 ID 由 Wi-Fi MAC 派生，不随 IP 或重启改变。首次选中的设备会被客户端记住，重连时保持同一设备。
同一设备/同一 boot/同一 IP 的临时掉线重发现会复用原 session，避免被自己的尚未失效租约拒绝；设备 boot 或端点变化时重新建立会话。
队列项保存路由 generation。更换订阅者或路由失效后，旧 generation 的队列项被丢弃，防止旧数据转发给新电脑。

## 多设备与广播受限

有多辆车时，用日志显示的 12 位 MAC ID 选择，例如：

```bash
python3 tools/udp_receiver.py --view --device-id 2884855438b8
ros2 launch smartcar_sensor_hal sensors.launch.py device_id:=2884855438b8 rviz:=true
```

ID 示例必须替换为实际设备。这个参数选择设备身份，不是填写 IP。

广播必须能在 MiFi 局域网内到达 S3。如果网络开启客户端隔离或屏蔽广播，客户端会报告未发现。
可配置当前网段的定向广播地址：

```bash
python3 tools/udp_receiver.py --discovery-address 192.168.0.255 --view
```

该地址只是当前 /24 网段的示例，应按实际网络掩码确定。手工指定设备 IP 也可作为排查路径：

```bash
python3 tools/udp_receiver.py --device-ip 192.168.0.117 --view
```

该模式仍发送发现与订阅请求，不会绕过设备的独占租约。ROS 参数 `device_ip` 和 `broadcast_address` 提供同类覆盖。

## 旧固件兼容

Mac 的 `--legacy` 用于 0.5.0 及更早的固定目标地址固件，仅被动监听，不发送发现/订阅。
要把 0.6.0 固件切回静态模式，可在 menuconfig 关闭 `SMARTCAR_AUTO_DISCOVERY`，再设置原有 HOST_IPV4/PORT 并重新烧录。
默认自动模式完全忽略静态 HOST_IPV4；更换电脑不需要重新烧录。

## 端口与测量

控制消息采用固定长度小端编码：

| 消息 | Python struct | 字段 | 字节数 |
|---|---|---|---|
| 发现 | `<4sQ` | SCD1, nonce | 12 |
| 设备回复 | `<4sQQQHH` | SCA1, nonce, device_id, boot_id, sync_port, capabilities | 32 |
| 订阅/续租 | `<4sQQQH` | SCS1, nonce, boot_id, session_id, data_port | 30 |
| 释放 | `<4sQQQH` | SCE1, nonce, boot_id, session_id, 0 | 30 |
| ACK | `<4sQQQQHH` | SCK1, nonce, device_id, boot_id, session_id, lease_ms, status | 40 |

capabilities bit0 为雷达、bit1 为 IMU。status 0 成功、1 占用、2 boot 不符、3 端口非法；订阅成功 lease_ms=6000，释放或错误时为 0。
零 session、非法长度或 magic 直接丢弃。MAC/ROS 在端点变更时还会有界清理本机接收队列。
数据 v1 没有 session 字段，不能严格识别所有跨租约的在途迟到包；ROS 仍使用年龄与单调时间门控。

| 用途 | 端口与方向 |
|---|---|
| 发现、订阅、续租、释放 | 电脑临时 UDP 端口 → S3 8890；回复回到该临时端口 |
| 雷达、IMU、心跳 | S3 → 当前订阅电脑的数据端口，默认 8888 |
| 四时刻对时 | 当前订阅电脑临时端口 → S3 8889 |

发现与订阅改变通信端点，不改变传感器 payload、单位、CRC 或采样时间定义。
该修改解决地址配置和重连问题；网络乱序、永久丢包测量、扫描重排及物理时间标定仍按各自机制处理。
