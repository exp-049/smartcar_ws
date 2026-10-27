#include "lidar_transport.h"
#include "lidar_wire.h"
#include "host_route.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#define LIDAR_QUEUE_DEPTH 16
#define LIDAR_MAX_QUEUE_AGE_US 250000

typedef struct {
    uint32_t sequence;
    int64_t rx_complete_us;
    uint16_t length;
    uint32_t route_generation;
    uint8_t raw[LIDAR_WIRE_RAW_MAX];
} lidar_item_t;

typedef struct {
    uint64_t captured;
    uint64_t sent;
    uint64_t queue_drops;
    uint64_t wifi_drops;
    uint64_t no_host_drops;
    uint64_t old_drops;
    uint64_t send_errors;
    uint64_t encode_errors;
} lidar_transport_stats_t;

static const char *TAG = "lidar_udp";
static QueueHandle_t s_packets;
static EventGroupHandle_t s_wifi_events;
static EventBits_t s_ready_bit;
static char s_boot_id[17];
static uint32_t s_sequence;
static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static lidar_transport_stats_t s_stats;

static void increment_counter(uint64_t *counter)
{
    portENTER_CRITICAL(&s_stats_lock);
    ++*counter;
    portEXIT_CRITICAL(&s_stats_lock);
}

void lidar_transport_capture_packet(const uint8_t *raw_packet, size_t length, void *context)
{
    (void)context;
    lidar_item_t item;
    item.rx_complete_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_stats_lock);
    item.sequence = s_sequence++;
    ++s_stats.captured;
    portEXIT_CRITICAL(&s_stats_lock);

    if (s_packets == NULL || s_wifi_events == NULL ||
        !(xEventGroupGetBits(s_wifi_events) & s_ready_bit)) {
        increment_counter(&s_stats.wifi_drops);
        return;
    }
    struct sockaddr_in destination;
    if (!host_route_get(&destination, &item.route_generation)) {
        increment_counter(&s_stats.no_host_drops);
        return;
    }
    if (raw_packet == NULL || length < 12 || length > sizeof(item.raw)) {
        increment_counter(&s_stats.encode_errors);
        return;
    }
    item.length = (uint16_t)length;
    memcpy(item.raw, raw_packet, length);
    if (xQueueSend(s_packets, &item, 0) != pdPASS) {
        increment_counter(&s_stats.queue_drops);
    }
}

static void lidar_udp_task(void *argument)
{
    (void)argument;
    int sock = -1;
    int64_t last_report_us = esp_timer_get_time();
    lidar_item_t item;
    char message[LIDAR_WIRE_DATAGRAM_MAX + 1];
    while (true) {
        if (xQueueReceive(s_packets, &item, pdMS_TO_TICKS(100)) == pdPASS) {
            struct sockaddr_in destination;
            uint32_t generation;
            bool have_host = host_route_get(&destination, &generation);
            if (!(xEventGroupGetBits(s_wifi_events) & s_ready_bit)) {
                increment_counter(&s_stats.wifi_drops);
            } else if (!have_host || generation != item.route_generation) {
                increment_counter(&s_stats.no_host_drops);
            } else if (esp_timer_get_time() - item.rx_complete_us > LIDAR_MAX_QUEUE_AGE_US) {
                increment_counter(&s_stats.old_drops);
            } else {
                if (sock < 0) {
                    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                    if (sock >= 0) {
                        const struct timeval timeout = {.tv_sec = 0, .tv_usec = 20000};
                        if (setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) != 0) {
                            close(sock);
                            sock = -1;
                        }
                    }
                }
                if (sock < 0) {
                    increment_counter(&s_stats.send_errors);
                } else {
                    int length = lidar_wire_encode(message, sizeof(message), s_boot_id,
                                                    item.sequence, item.rx_complete_us,
                                                    item.raw, item.length);
                    if (length < 0) {
                        increment_counter(&s_stats.encode_errors);
                    } else if (sendto(sock, message, length, 0,
                                      (const struct sockaddr *)&destination, sizeof(destination)) != length) {
                        increment_counter(&s_stats.send_errors);
                        close(sock);
                        sock = -1;
                    } else {
                        increment_counter(&s_stats.sent);
                    }
                }
            }
        }
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_report_us >= 1000000) {
            lidar_transport_stats_t stats;
            portENTER_CRITICAL(&s_stats_lock);
            stats = s_stats;
            portEXIT_CRITICAL(&s_stats_lock);
            ESP_LOGI(TAG, "captured=%" PRIu64 " sent=%" PRIu64 " queue_drop=%" PRIu64
                     " wifi_drop=%" PRIu64 " no_host_drop=%" PRIu64 " old_drop=%" PRIu64 " send_err=%" PRIu64
                     " encode_err=%" PRIu64,
                     stats.captured, stats.sent, stats.queue_drops, stats.wifi_drops, stats.no_host_drops,
                     stats.old_drops, stats.send_errors, stats.encode_errors);
            last_report_us = now_us;
        }
    }
}

void lidar_transport_start(EventGroupHandle_t wifi_events, EventBits_t ready_bit,
                            const char *boot_id)
{
    if (s_packets != NULL || wifi_events == NULL || boot_id == NULL || strlen(boot_id) != 16) {
        ESP_LOGE(TAG, "Invalid lidar UDP initialization");
        return;
    }
    s_wifi_events = wifi_events;
    s_ready_bit = ready_bit;
    memcpy(s_boot_id, boot_id, sizeof(s_boot_id));
    s_packets = xQueueCreate(LIDAR_QUEUE_DEPTH, sizeof(lidar_item_t));
    if (s_packets == NULL) {
        ESP_LOGE(TAG, "Unable to allocate bounded lidar queue");
        return;
    }
    if (xTaskCreate(lidar_udp_task, "lidar_udp", 6144, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Unable to create lidar sender task");
        vQueueDelete(s_packets);
        s_packets = NULL;
        return;
    }
    ESP_LOGI(TAG, "Lidar UDP uses active host route; bounded queue=%d; no per-packet ACK",
             LIDAR_QUEUE_DEPTH);
}
