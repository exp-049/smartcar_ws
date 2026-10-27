idf_component_register(
    SRCS "smartcar_main.c" "udp_ack.c" "lidar_uart.c" "x3_packet_parser.c"
         "lidar_transport.c" "lidar_wire.c" "mpu6050.c" "sensor_wire.c" "time_sync.c"
         "host_route.c" "discovery_wire.c" "host_discovery.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_wifi esp_event esp_netif esp_timer esp_system esp_hw_support nvs_flash lwip esp_driver_uart esp_driver_gpio esp_driver_i2c
)
