cmake_minimum_required(VERSION 3.8)
project(smartcar_sim_io)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(imu_sanitizer src/imu_sanitizer.cpp)
ament_target_dependencies(imu_sanitizer rclcpp sensor_msgs)
add_executable(odom_republisher src/odom_republisher.cpp)
ament_target_dependencies(odom_republisher rclcpp nav_msgs)
add_executable(cmd_watchdog src/cmd_watchdog.cpp)
ament_target_dependencies(cmd_watchdog rclcpp geometry_msgs)

install(TARGETS imu_sanitizer odom_republisher cmd_watchdog DESTINATION lib/${PROJECT_NAME})
ament_package()
