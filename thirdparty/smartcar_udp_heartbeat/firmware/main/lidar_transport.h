#pragma once

#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

void lidar_transport_start(EventGroupHandle_t wifi_events, EventBits_t ready_bit,
                            const char *boot_id);
void lidar_transport_capture_packet(const uint8_t *raw_packet, size_t length, void *context);
