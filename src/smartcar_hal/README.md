# ESP32-S3 UDP HAL 接入

本工作区已集成 `smartcar_sensor_hal`：它使用 `thirdparty/smartcar_udp_heartbeat` 中的协议、发现、
订阅租约和对时实现，从 ESP32-S3 接收 X3PRO 雷达与 MPU6050 数据，发布 `/car/scan`、
`/car/imu/data_raw` 和 `/diagnostics`。固定安装 TF 仍由唯一的 `robot_state_publisher` 发布。

`hal.launch.py` 和实机 bringup 默认启动 `smartcar_sensor_hal/sensor_hal`，并加载该包的
`config/sensors.yaml`。配置中的 `broadcast_address` 当前按 `192.168.0.0/24` 网络设置；
其他网络请复制该 YAML 并改成设备所在子网的广播地址，然后通过 `hal_params_file:=/绝对路径/配置.yaml` 指定。
也可直接启动驱动自带的发现与 RViz 入口：

```bash
ros2 launch smartcar_sensor_hal sensors.launch.py
```

如需替换驱动，可继续通过 `hal_package`、`hal_executable` 和 `hal_params_file` 传入其他 ROS 包与参数文件。
`hal.template.yaml` 只作为外部驱动的配置模板，不能直接用于启动。

传感器单位分别为 m、rad、rad/s 和 m/s²；IMU 数据保留重力。底盘速度看门狗、IMU/雷达安装外参、
轴向约定和实机标定仍由对应固件及统一 URDF 负责。完整报文、时间估算、网络切换和实机联调说明见
`thirdparty/smartcar_udp_heartbeat/{README.md,PROTOCOL.md,DISCOVERY.md,TIMING.md}`。
