#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* Local UDP discovery/control service, enabled by AUTO_DISCOVERY. Call after
 * host_route_init; Wi-Fi event callbacks should update host_route readiness.
 * Device ID is the stable ESP_MAC_WIFI_STA bytes interpreted as a 48-bit value.
 */
esp_err_t host_discovery_start(EventGroupHandle_t wifi_events, EventBits_t ready_bit,
                               const char *boot_id);
