# Vendored third-party sources

`smartcar_udp_heartbeat/` 是从用户本机工程目录导入的 ESP32-S3 UDP 心跳、雷达和 IMU 源码快照。
构建产物、运行日志、本机 ESP-IDF `sdkconfig` 和缓存目录未纳入。

ROS 2 Jazzy 驱动通过 `src/smartcar_sensor_hal` 集成并随工作区构建；此处保留完整上游包及协议资料。
`COLCON_IGNORE` 防止 colcon 把 vendored ROS 包当作第二份工作区包。升级驱动时请同步更新工作区包。

`micro_ros_agent/` 保存 micro-ROS Agent Jazzy 源码（上游提交 `7c93232`）及其本地 XRCE-DDS 源码包；
`micro_ros_msgs/` 保存 Agent 所需的消息定义（上游提交 `b1ef852`）。ROS 包本身通过 `../src/micro_ros_agent` 和
`../src/micro_ros_msgs` 软链接加入工作区。Agent 的 CMake 补丁从 `micro_ros_agent/downloads/` 读取已校验的
`Micro-XRCE-DDS-Agent-v2.4.3.tar.gz`，SHA256 固定在 `SuperBuild.cmake` 中，避免构建时重新下载。
