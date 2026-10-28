set(chassis_sources "chassis_core.c" "chassis_hw.c")
set(chassis_requires driver esp_timer nvs_flash)

if(CONFIG_SMARTCAR_CHASSIS_MICROROS_UDP)
    set(sensor_network_dir "${PROJECT_DIR}/../../thirdparty/smartcar_udp_heartbeat/firmware/main")
    list(APPEND chassis_sources "microros_main.c" "chassis_wifi.c" "esp32_udp_transport.c"
         "${sensor_network_dir}/discovery_wire.c" "${sensor_network_dir}/host_route.c"
         "${sensor_network_dir}/host_discovery.c")
    list(APPEND chassis_requires micro_ros_espidf_component esp_wifi esp_event esp_netif lwip)
elseif(CONFIG_SMARTCAR_CHASSIS_MICROROS_SERIAL)
    list(APPEND chassis_sources "microros_main.c" "esp32_serial_transport.c")
    list(APPEND chassis_requires micro_ros_espidf_component)
else()
    list(APPEND chassis_sources "console_main.c")
endif()

idf_component_register(
    SRCS ${chassis_sources}
    INCLUDE_DIRS "." ${sensor_network_dir}
    REQUIRES ${chassis_requires}
)
