set(chassis_sources "chassis_core.c" "chassis_hw.c")
set(chassis_requires driver esp_timer nvs_flash)

if(CONFIG_SMARTCAR_CHASSIS_MICROROS_SERIAL)
    list(APPEND chassis_sources "microros_main.c" "esp32_serial_transport.c")
    list(APPEND chassis_requires micro_ros_espidf_component)
else()
    list(APPEND chassis_sources "console_main.c")
endif()

idf_component_register(
    SRCS ${chassis_sources}
    INCLUDE_DIRS "."
    REQUIRES ${chassis_requires}
)
