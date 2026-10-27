/* Wi-Fi / UDP link test for the ESP32-S3 smartcar sensor firmware.
 * Heartbeat uptime is the transmit timestamp, not a sensor sampling timestamp.
 */
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "udp_ack.h"
#include "lidar_uart.h"
#include "lidar_transport.h"

#define WIFI_READY_BIT BIT0
#define HEARTBEAT_PERIOD_MS 1000
#define ACK_TIMEOUT_MS 800

static const char *TAG = "smartcar_udp";
static EventGroupHandle_t s_wifi_events;
static char s_boot_id[17];

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event = event_data;
        xEventGroupClearBits(s_wifi_events, WIFI_READY_BIT);
        ESP_LOGW(TAG, "Wi-Fi disconnected: reason=%u; reconnecting", event->reason);
        esp_wifi_connect();
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        ESP_LOGI(TAG, "Wi-Fi ready; ESP32 IPv4=" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_READY_BIT);
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP) {
        xEventGroupClearBits(s_wifi_events, WIFI_READY_BIT);
    }
}

static void udp_heartbeat_task(void *arg)
{
    (void)arg;
    struct sockaddr_in destination = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_SMARTCAR_HOST_PORT),
    };
    if (inet_pton(AF_INET, CONFIG_SMARTCAR_HOST_IPV4, &destination.sin_addr) != 1 ||
        destination.sin_addr.s_addr == htonl(INADDR_ANY) ||
        destination.sin_addr.s_addr == htonl(INADDR_BROADCAST) ||
        (ntohl(destination.sin_addr.s_addr) >> 24) == 127) {
        ESP_LOGE(TAG, "Invalid Mac IPv4 address; configure Smartcar UDP Heartbeat in menuconfig");
        vTaskDelete(NULL);
        return;
    }

    uint32_t sequence = 0;
    while (true) {
        xEventGroupWaitBits(s_wifi_events, WIFI_READY_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            ESP_LOGE(TAG, "socket failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
            continue;
        }
        const struct timeval timeout = {
            .tv_sec = 0,
            .tv_usec = ACK_TIMEOUT_MS * 1000,
        };
        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0 ||
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) != 0) {
            ESP_LOGE(TAG, "socket timeout setup failed: errno=%d", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
            continue;
        }
        ESP_LOGI(TAG, "UDP destination=%s:%d; boot_id=%s", CONFIG_SMARTCAR_HOST_IPV4,
                 CONFIG_SMARTCAR_HOST_PORT, s_boot_id);
        TickType_t last_wake = xTaskGetTickCount();

        while (xEventGroupGetBits(s_wifi_events) & WIFI_READY_BIT) {
            const uint32_t current_sequence = sequence++;
            char payload[192];
            int payload_length = snprintf(
                payload, sizeof(payload),
                "{\"type\":\"heartbeat\",\"version\":1,\"boot_id\":\"%s\","
                "\"seq\":%" PRIu32 ",\"uptime_us\":%" PRId64 "}",
                s_boot_id, current_sequence, esp_timer_get_time());
            if (payload_length < 0 || payload_length >= (int)sizeof(payload)) {
                ESP_LOGE(TAG, "heartbeat encoding failed");
                break;
            }
            int sent = sendto(sock, payload, payload_length, 0,
                              (const struct sockaddr *)&destination, sizeof(destination));
            if (sent != payload_length) {
                ESP_LOGW(TAG, "sendto failed: errno=%d; reopening socket", errno);
                break;
            }

            char expected[64];
            snprintf(expected, sizeof(expected), "ACK %s %" PRIu32,
                     s_boot_id, current_sequence);
            udp_ack_stats_t ack_stats;
            int ack_result = udp_wait_for_ack(sock, &destination, expected, strlen(expected),
                                              ACK_TIMEOUT_MS, esp_timer_get_time, &ack_stats);
            if (ack_result < 0) {
                ESP_LOGW(TAG, "ACK socket error: errno=%d; reopening socket", errno);
                break;
            }
            unsigned ignored = ack_stats.wrong_source_packets + ack_stats.unmatched_payload_packets;
            if (ack_result == 1) {
                ESP_LOGI(TAG, "TX seq=%" PRIu32 " bytes=%d; ACK OK; ignored=%u",
                         current_sequence, sent, ignored);
            } else {
                ESP_LOGW(TAG, "TX seq=%" PRIu32 "; ACK timeout after %d ms; wrong_source=%u unmatched=%u",
                         current_sequence, ACK_TIMEOUT_MS, ack_stats.wrong_source_packets,
                         ack_stats.unmatched_payload_packets);
                if (ack_stats.wrong_source_packets > 0) {
                    char source_address[INET_ADDRSTRLEN] = "unknown";
                    inet_ntop(AF_INET, &ack_stats.last_wrong_source.sin_addr,
                              source_address, sizeof(source_address));
                    ESP_LOGW(TAG, "Last unexpected peer=%s:%u; expected=%s:%d", source_address,
                             ntohs(ack_stats.last_wrong_source.sin_port),
                             CONFIG_SMARTCAR_HOST_IPV4, CONFIG_SMARTCAR_HOST_PORT);
                }
                if (ack_stats.unmatched_payload_packets > 0) {
                    ESP_LOGW(TAG, "Expected '%s'; last unmatched '%s'", expected,
                             ack_stats.last_unmatched_reply);
                }
            }
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
        }
        close(sock);
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
    }
}

void app_main(void)
{
    const size_t ssid_length = strlen(CONFIG_SMARTCAR_WIFI_SSID);
    const size_t password_length = strlen(CONFIG_SMARTCAR_WIFI_PASSWORD);
    if (ssid_length == 0 || ssid_length > 32 || password_length > 64) {
        ESP_LOGE(TAG, "Configure a Wi-Fi SSID (1-32 bytes) and password (0-64 bytes) in menuconfig");
        return;
    }
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);
    s_wifi_events = xEventGroupCreate();
    if (s_wifi_events == NULL) {
        ESP_LOGE(TAG, "Unable to allocate Wi-Fi event group");
        return;
    }
    snprintf(s_boot_id, sizeof(s_boot_id), "%08" PRIx32 "%08" PRIx32,
             esp_random(), esp_random());

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (esp_netif_create_default_wifi_sta() == NULL) {
        ESP_LOGE(TAG, "Unable to create Wi-Fi station interface");
        return;
    }
    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    memcpy(wifi_config.sta.ssid, CONFIG_SMARTCAR_WIFI_SSID, ssid_length);
    memcpy(wifi_config.sta.password, CONFIG_SMARTCAR_WIFI_PASSWORD, password_length);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Wi-Fi starting; heartbeat=1 Hz; password is not logged");
#ifdef CONFIG_SMARTCAR_LIDAR_ENABLE
    lidar_transport_start(s_wifi_events, WIFI_READY_BIT, s_boot_id);
#endif
    lidar_uart_start();
    if (xTaskCreate(udp_heartbeat_task, "udp_heartbeat", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Unable to create UDP heartbeat task");
    }
}
