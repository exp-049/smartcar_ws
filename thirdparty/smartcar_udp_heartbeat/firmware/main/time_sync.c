#include "time_sync.h"
#include "sensor_wire.h"
#include "host_route.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

typedef struct {
    uint64_t replied;
    uint64_t bad_length;
    uint64_t bad_source;
    uint64_t bad_magic;
    uint64_t socket_errors;
} sync_stats_t;

static const char *TAG = "time_sync";
static EventGroupHandle_t s_wifi_events;
static EventBits_t s_ready_bit;
static uint64_t s_boot_value;
static TaskHandle_t s_task;

static int open_sync_socket(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        return -1;
    }
    const struct timeval receive_timeout = {.tv_sec = 0, .tv_usec = 250000};
    const struct timeval send_timeout = {.tv_sec = 0, .tv_usec = 20000};
    const struct sockaddr_in local = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_SMARTCAR_SYNC_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout)) != 0 ||
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout)) != 0 ||
        bind(sock, (const struct sockaddr *)&local, sizeof(local)) != 0) {
        close(sock);
        return -1;
    }
    return sock;
}

static void sync_task(void *argument)
{
    (void)argument;
    int sock = -1;
    int64_t report_us = esp_timer_get_time();
    sync_stats_t stats = {0};
    while (true) {
        if (!(xEventGroupGetBits(s_wifi_events) & s_ready_bit)) {
            if (sock >= 0) {
                close(sock);
                sock = -1;
            }
            xEventGroupWaitBits(s_wifi_events, s_ready_bit, pdFALSE, pdTRUE, pdMS_TO_TICKS(500));
        } else {
            if (sock < 0) {
                sock = open_sync_socket();
                if (sock < 0) {
                    ++stats.socket_errors;
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            }
            if (sock >= 0) {
                /* One extra byte distinguishes oversized (including truncated)
                 * requests from the exact 20-byte SCQ1 request. */
                uint8_t request[SENSOR_SYNC_REQUEST_BYTES + 1];
                struct sockaddr_in source = {0};
                socklen_t source_length = sizeof(source);
                int length = recvfrom(sock, request, sizeof(request), 0,
                                      (struct sockaddr *)&source, &source_length);
                const int64_t device_t2_us = esp_timer_get_time();
                if (length < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                        ++stats.socket_errors;
                        close(sock);
                        sock = -1;
                    }
                } else if (source_length < sizeof(source) || source.sin_family != AF_INET ||
                           !host_route_authorized(source.sin_addr)) {
                    ++stats.bad_source;
                } else if (length != SENSOR_SYNC_REQUEST_BYTES) {
                    ++stats.bad_length;
                } else {
                    uint64_t nonce, host_t1_ns;
                    if (!sensor_wire_sync_request_decode(request, (size_t)length, &nonce, &host_t1_ns)) {
                        ++stats.bad_magic;
                    } else {
                        uint8_t response[SENSOR_SYNC_RESPONSE_BYTES];
                        /* t3 is the last software timestamp before encoding the
                         * fixed 44-byte reply and calling sendto. It is NOT a
                         * kernel TX or radio timestamp. A successful response
                         * only supplies a measurement for the host's RTT filter.
                         */
                        int response_length = sensor_wire_sync_response_encode(response, sizeof(response),
                            nonce, host_t1_ns, device_t2_us, esp_timer_get_time(), s_boot_value);
                        if (response_length != SENSOR_SYNC_RESPONSE_BYTES ||
                            sendto(sock, response, (size_t)response_length, 0,
                                   (const struct sockaddr *)&source, sizeof(source)) != response_length) {
                            ++stats.socket_errors;
                        } else {
                            ++stats.replied;
                        }
                    }
                }
            }
        }
        int64_t now_us = esp_timer_get_time();
        if (now_us - report_us >= 5000000) {
            ESP_LOGI(TAG, "replied=%" PRIu64 " bad_length=%" PRIu64 " bad_source=%" PRIu64
                     " bad_magic=%" PRIu64 " socket_err=%" PRIu64,
                     stats.replied, stats.bad_length, stats.bad_source, stats.bad_magic, stats.socket_errors);
            report_us = now_us;
        }
    }
}

esp_err_t time_sync_start(EventGroupHandle_t wifi_events, EventBits_t ready_bit, const char *boot_id)
{
    if (s_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (wifi_events == NULL || ready_bit == 0 ||
        CONFIG_SMARTCAR_SYNC_PORT < 1 || CONFIG_SMARTCAR_SYNC_PORT > 65535 ||
        !sensor_wire_boot_id_value(boot_id, &s_boot_value)) {
        return ESP_ERR_INVALID_ARG;
    }
    s_wifi_events = wifi_events;
    s_ready_bit = ready_bit;
    if (xTaskCreate(sync_task, "time_sync", 4096, NULL, 5, &s_task) != pdPASS) {
        s_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "UDP port=%d; SCQ1/SCR1; only active subscriber accepted; software timestamps",
             CONFIG_SMARTCAR_SYNC_PORT);
    return ESP_OK;
}
