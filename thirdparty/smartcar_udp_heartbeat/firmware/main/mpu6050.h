#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* GPIO SCL/SDA come from CONFIG_SMARTCAR_IMU_{SCL,SDA}_GPIO.
 * No INT pin is used. A 2 ms status poll brackets data-ready; transmitted
 * uncertainty bounds this software observation, not DLPF/physical latency.
 * Initialization failure can be logged without stopping unrelated sensors.
 */
esp_err_t mpu6050_start(EventGroupHandle_t wifi_events, EventBits_t ready_bit,
                        const char *boot_id);
