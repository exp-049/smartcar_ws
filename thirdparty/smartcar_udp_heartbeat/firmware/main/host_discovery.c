#include "host_discovery.h"
#include "discovery_wire.h"
#include "host_route.h"

#include "esp_log.h"
#include "sdkconfig.h"

#ifdef CONFIG_SMARTCAR_AUTO_DISCOVERY
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <sys/time.h>
#include <unistd.h>
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

typedef struct {
    uint64_t announced;
    uint64_t accepted;
    uint64_t released;
    uint64_t rejected;
    uint64_t malformed;
    uint64_t socket_errors;
} discovery_stats_t;

static const char *TAG = "host_discovery";
static EventGroupHandle_t s_wifi_events;
static EventBits_t s_ready_bit;
static uint64_t s_device_id;
static uint64_t s_boot_id;
static TaskHandle_t s_task;

static uint16_t capabilities(void)
{
    uint16_t flags = 0;
#ifdef CONFIG_SMARTCAR_LIDAR_ENABLE
    flags |= DISCOVERY_CAPABILITY_LIDAR;
#endif
#ifdef CONFIG_SMARTCAR_IMU_ENABLE
    flags |= DISCOVERY_CAPABILITY_IMU;
#endif
    return flags;
}

static int open_discovery_socket(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        return -1;
    }
    const struct timeval receive_timeout = {.tv_sec = 0, .tv_usec = 200000};
    const struct timeval send_timeout = {.tv_sec = 0, .tv_usec = 20000};
    const struct sockaddr_in local = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_SMARTCAR_DISCOVERY_PORT),
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

static void discovery_task(void *argument)
{
    (void)argument;
    int sock = -1;
    int64_t report_us = esp_timer_get_time();
    discovery_stats_t stats = {0};
    while (true) {
        bool ready = (xEventGroupGetBits(s_wifi_events) & s_ready_bit) != 0;
        if (!ready) {
            /* The Wi-Fi callback invalidates immediately; this also protects
             * callers which only supplied the EventGroup without callbacks. */
            host_route_set_wifi_ready(false);
            if (sock >= 0) {
                close(sock);
                sock = -1;
            }
            xEventGroupWaitBits(s_wifi_events, s_ready_bit, pdFALSE, pdTRUE, pdMS_TO_TICKS(500));
        } else {
            host_route_set_wifi_ready(true);
            if (sock < 0) {
                sock = open_discovery_socket();
                if (sock < 0) {
                    ++stats.socket_errors;
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            }
            if (sock >= 0) {
                uint8_t data[DISCOVERY_CONTROL_BYTES + 1];
                struct sockaddr_in source = {0};
                socklen_t source_length = sizeof(source);
                int length = recvfrom(sock, data, sizeof(data), 0,
                                      (struct sockaddr *)&source, &source_length);
                if (length < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                        ++stats.socket_errors;
                        close(sock);
                        sock = -1;
                    }
                } else if (!(xEventGroupGetBits(s_wifi_events) & s_ready_bit)) {
                    host_route_set_wifi_ready(false);
                } else {
                    discovery_request_t request;
                    if (source_length < sizeof(source) || source.sin_family != AF_INET ||
                        !discovery_wire_request_decode(data, (size_t)length, &request)) {
                        ++stats.malformed;
                    } else {
                        uint8_t response[DISCOVERY_ACK_BYTES];
                        int response_length;
                        if (request.kind == DISCOVERY_DISCOVER) {
                            response_length = discovery_wire_announce_encode(response, sizeof(response),
                                request.nonce, s_device_id, s_boot_id, CONFIG_SMARTCAR_SYNC_PORT, capabilities());
                            ++stats.announced;
                        } else {
                            discovery_status_t status;
                            uint16_t lease_ms = 0;
                            if (request.boot_id != s_boot_id) {
                                status = DISCOVERY_STATUS_WRONG_BOOT;
                            } else if ((request.kind == DISCOVERY_SUBSCRIBE && request.data_port < 1024) ||
                                       (request.kind == DISCOVERY_RELEASE && request.data_port != 0)) {
                                status = DISCOVERY_STATUS_BAD_PORT;
                            } else if (request.kind == DISCOVERY_SUBSCRIBE) {
                                status = (discovery_status_t)host_route_claim(source.sin_addr,
                                    request.session_id, request.data_port);
                                if (status == DISCOVERY_STATUS_OK) {
                                    lease_ms = DISCOVERY_LEASE_MS;
                                    ++stats.accepted;
                                }
                            } else {
                                status = (discovery_status_t)host_route_release(source.sin_addr, request.session_id);
                                if (status == DISCOVERY_STATUS_OK) {
                                    ++stats.released;
                                }
                            }
                            if (status != DISCOVERY_STATUS_OK) {
                                ++stats.rejected;
                            }
                            response_length = discovery_wire_ack_encode(response, sizeof(response), request.nonce,
                                s_device_id, s_boot_id, request.session_id, lease_ms, status);
                        }
                        if (response_length < 0 ||
                            sendto(sock, response, (size_t)response_length, 0,
                                   (const struct sockaddr *)&source, sizeof(source)) != response_length) {
                            ++stats.socket_errors;
                        }
                    }
                }
            }
        }
        int64_t now_us = esp_timer_get_time();
        if (now_us - report_us >= 5000000) {
            ESP_LOGI(TAG, "announced=%" PRIu64 " accepted=%" PRIu64 " released=%" PRIu64
                     " rejected=%" PRIu64 " malformed=%" PRIu64 " socket_err=%" PRIu64,
                     stats.announced, stats.accepted, stats.released, stats.rejected,
                     stats.malformed, stats.socket_errors);
            report_us = now_us;
        }
    }
}

esp_err_t host_discovery_start(EventGroupHandle_t wifi_events, EventBits_t ready_bit, const char *boot_id)
{
    if (s_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (wifi_events == NULL || ready_bit == 0 ||
        CONFIG_SMARTCAR_DISCOVERY_PORT < 1 || CONFIG_SMARTCAR_DISCOVERY_PORT > 65535 ||
        CONFIG_SMARTCAR_SYNC_PORT < 1 || CONFIG_SMARTCAR_SYNC_PORT > 65535 ||
        CONFIG_SMARTCAR_DISCOVERY_PORT == CONFIG_SMARTCAR_SYNC_PORT ||
        !discovery_wire_boot_id_value(boot_id, &s_boot_id)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t mac[6];
    esp_err_t error = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (error != ESP_OK) {
        return error;
    }
    s_device_id = 0;
    for (size_t index = 0; index < sizeof(mac); ++index) {
        s_device_id = (s_device_id << 8) | mac[index];
    }
    s_wifi_events = wifi_events;
    s_ready_bit = ready_bit;
    if (xTaskCreate(discovery_task, "host_discovery", 4096, NULL, 5, &s_task) != pdPASS) {
        s_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "UDP discovery port=%d; device_id=%012" PRIx64 "; lease=%dms; no subscriber means no sensor UDP",
             CONFIG_SMARTCAR_DISCOVERY_PORT, s_device_id, DISCOVERY_LEASE_MS);
    return ESP_OK;
}
#else
esp_err_t host_discovery_start(EventGroupHandle_t wifi_events, EventBits_t ready_bit, const char *boot_id)
{
    (void)wifi_events;
    (void)ready_bit;
    (void)boot_id;
    ESP_LOGI("host_discovery", "Auto discovery disabled; using configured static host route");
    return ESP_OK;
}
#endif
