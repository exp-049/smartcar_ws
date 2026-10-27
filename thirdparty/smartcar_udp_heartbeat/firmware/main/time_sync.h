#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* A separate UDP SCQ1/SCR1 software timestamp responder. Requests are accepted
 * only from CONFIG_SMARTCAR_HOST_IPV4. No wall clock or sample time is changed.
 */
esp_err_t time_sync_start(EventGroupHandle_t wifi_events, EventBits_t ready_bit,
                         const char *boot_id);
