# ESP32 底盘 micro-ROS UDP 准备记录

- 2026-10-27：源码和主机入口已改为 UDP，ESP-IDF 固件与 ROS bringup 包编译完成。
- 按用户要求，尚未刷写 ESP32，也未进行 Wi-Fi、发现、UDP 话题或电机实测。当前设备上的固件仍为此前联调的串口版本。
- 传感器资料中的目标 SSID 为 `Mifi_049`。传感器源码快照不含密码，底盘本地 `sdkconfig` 中密码尚未配置；当前编译出的 UDP 固件不能直接连接该网络。

## 网络和发现

ESP32 V1 以 STA 模式接入 2.4 GHz Wi-Fi，通过 DHCP 获取地址。电脑上的发现程序从选定网卡的 IPv4 网段计算定向广播地址，主动查找底盘，再通过设备 MAC ID、随机 boot ID 和可续订的单订阅者租约锁定目标。发现协议和六秒租约复用 `smartcar_sensor_hal` 与 `smartcar_udp_heartbeat` 的实现；租约用于选择主机，不能替代底盘速度命令的 300 ms 看门狗。

| 用途 | UDP 端口 |
|---|---:|
| 传感器数据（HAL 默认） | 8888 |
| 传感器对时（固件默认） | 8889 |
| 传感器发现与订阅（HAL 默认） | 8890 |
| 底盘 micro-ROS Agent（默认） | 8891 |
| 底盘发现与订阅（默认） | 8892 |

底盘使用与传感器相同的发现报文结构，发现响应中原名为 `sync_port` 的字段通告底盘 Agent 端口；底盘不在该端口提供传感器对时服务。电脑和 ESP32 必须能在同一局域网互通，路由器需要允许终端间通信和定向广播。地址由 DHCP 和发现机制确定，不固定 ESP32 或电脑 IP。发现协议不是加密认证协议，仅在可信局域网使用。

Wi-Fi 掉线、租约过期或发现目标变更时固件会结束当前会话并停止、解除使能底盘；重连需要重新发现。有效 `cmd_vel` 仍会尝试自动使能，实际行驶和急停链路需要后续在有人看护时验证。

## 密码配置和启动

密码仅写入本地被 Git 忽略的 `firmware/smartcar_chassis_v1/sdkconfig`；`sdkconfig.old` 也被忽略。可以复用传感器包的隐藏输入工具，设置目标 SSID 时执行：

```bash
cd /home/doc049/dev/smartcar_ws
python3 thirdparty/smartcar_udp_heartbeat/tools/configure_network.py \
  --ssid Mifi_049 --config firmware/smartcar_chassis_v1/sdkconfig
```

输入密码后必须重新构建固件；实际刷写和联网验证留待用户回复后进行。旧串口应用镜像和构建配置保存在当前任务工作目录的 `work/chassis_serial_backup/`，刷写前还需要根据设备现场状态确认动力电源和串口占用。

主机 UDP Agent 与底盘主动发现合并在现有 launch：

```bash
source /opt/ros/jazzy/setup.bash
source /home/doc049/Documents/Codex/2026-10-27/kan/work/micro_ros_agent_ws/install/local_setup.bash
source /home/doc049/dev/smartcar_ws/install/local_setup.bash
ros2 launch smartcar_bringup agent.launch.py
```

默认 `agent_port:=8891 chassis_discovery_port:=8892`。多网卡环境可传 `chassis_interface:=wlp4s0`；如已知广播地址、底盘 MAC ID 或底盘 IP，可分别传 `chassis_broadcast_address`、`chassis_device_id`、`chassis_device_ip`。有多个底盘时须指定设备 ID。实机建图和导航入口也使用这些参数。要使用旧串口固件，需显式指定 `agent_transport:=serial agent_device:=... agent_baudrate:=460800`；串口和 UDP Agent 不应重复占用同一个设备或端口。
