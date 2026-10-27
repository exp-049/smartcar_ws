#include "lidar_uart.h"

#include <inttypes.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"

#ifdef CONFIG_SMARTCAR_LIDAR_ENABLE
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "x3_packet_parser.h"
#include "lidar_transport.h"

#define LIDAR_UART UART_NUM_1
#define RX_BUFFER_BYTES 8192
#define PARTIAL_PACKET_GAP_US 250000

static const char *TAG = "lidar_uart";
static QueueHandle_t s_uart_events;

static void lidar_uart_task(void *argument)
{
    (void)argument;
    x3_packet_parser_t parser;
    x3_packet_parser_init(&parser);
    x3_packet_parser_set_handler(&parser, lidar_transport_capture_packet, NULL);
    uint8_t incoming[512];
    char last_hex[49] = "none";
    uint64_t overflows = 0;
    uint64_t line_errors = 0;
    uint64_t previous_bytes = 0;
    uint64_t previous_packets = 0;
    uint64_t previous_markers = 0;
    int64_t last_rx_us = 0;
    int64_t report_us = esp_timer_get_time();

    while (true) {
        uart_event_t event;
        // Wait for an RX event, then drain what is already buffered. Asking for
        // 512 bytes with a 20 ms timeout would add avoidable timestamp latency.
        BaseType_t have_event = xQueueReceive(s_uart_events, &event, pdMS_TO_TICKS(20));
        while (have_event == pdTRUE) {
            if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                ++overflows;
                uart_flush_input(LIDAR_UART);
                xQueueReset(s_uart_events);
                x3_packet_parser_drop_partial(&parser);
            } else if (event.type == UART_PARITY_ERR || event.type == UART_FRAME_ERR ||
                       event.type == UART_BREAK) {
                ++line_errors;
            }
            have_event = xQueueReceive(s_uart_events, &event, 0);
        }

        size_t available = 0;
        esp_err_t buffered_result = uart_get_buffered_data_len(LIDAR_UART, &available);
        size_t requested = available < sizeof(incoming) ? available : sizeof(incoming);
        int count = buffered_result != ESP_OK ? -1 :
                    (requested == 0 ? 0 : uart_read_bytes(LIDAR_UART, incoming, requested, 0));
        int64_t now_us = esp_timer_get_time();
        if (last_rx_us != 0 && now_us - last_rx_us > PARTIAL_PACKET_GAP_US) {
            x3_packet_parser_drop_partial(&parser);
        }
        if (count > 0) {
            last_rx_us = now_us;
            size_t shown = (size_t)count < 16 ? (size_t)count : 16;
            for (size_t index = 0; index < shown; ++index) {
                snprintf(last_hex + 3 * index, sizeof(last_hex) - 3 * index,
                         "%02X ", incoming[index]);
            }
            x3_packet_parser_feed(&parser, incoming, (size_t)count);
        } else if (count < 0) {
            ESP_LOGE(TAG, "uart_read_bytes failed");
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (now_us - report_us >= 1000000) {
            double elapsed = (now_us - report_us) / 1000000.0;
            ESP_LOGI(TAG, "RX=%.0f B/s total=%" PRIu64 " valid=%" PRIu64 " (+%" PRIu64
                     "/window) samples=%" PRIu64 " start_markers=%.2f/s checksum_bad=%" PRIu64
                     " header_bad=%" PRIu64 " overflow=%" PRIu64 " line_errors=%" PRIu64,
                     (parser.bytes - previous_bytes) / elapsed, parser.bytes,
                     parser.valid_packets, parser.valid_packets - previous_packets,
                     parser.sample_words, (parser.start_markers - previous_markers) / elapsed,
                     parser.checksum_errors, parser.header_errors, overflows, line_errors);
            if (parser.valid_packets > previous_packets) {
                ESP_LOGI(TAG, "Last packet: CT=0x%02X points=%u FSA_raw=%u LSA_raw=%u first_distance_raw=%u CS=0x%04X",
                         parser.last_ct, parser.last_sample_count, parser.last_first_angle_raw,
                         parser.last_last_angle_raw, parser.last_first_distance_raw, parser.last_checksum);
            } else if (parser.bytes == 0) {
                ESP_LOGW(TAG, "No UART bytes: check radar Tx -> GPIO%d and common GND",
                         CONFIG_SMARTCAR_LIDAR_RX_GPIO);
            } else if (parser.valid_packets == 0) {
                ESP_LOGW(TAG, "Bytes received but no verified standard packets; latest bytes: %s", last_hex);
            }
            previous_bytes = parser.bytes;
            previous_packets = parser.valid_packets;
            previous_markers = parser.start_markers;
            report_us = now_us;
        }
    }
}

void lidar_uart_start(void)
{
    const int pin = CONFIG_SMARTCAR_LIDAR_RX_GPIO;
    if (!GPIO_IS_VALID_GPIO(pin) || pin == 19 || pin == 20 ||
        pin == 35 || pin == 36 || pin == 37 ||
        pin == 0 || pin == 3 || pin == 45 || pin == 46) {
        ESP_LOGE(TAG, "GPIO%d conflicts with USB, PSRAM, strapping, or is invalid; UART diagnostics disabled", pin);
        return;
    }
    const uart_config_t settings = {
        .baud_rate = CONFIG_SMARTCAR_LIDAR_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t error = uart_param_config(LIDAR_UART, &settings);
    if (error == ESP_OK) {
        error = uart_set_pin(LIDAR_UART, UART_PIN_NO_CHANGE, pin,
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (error == ESP_OK) {
        error = uart_driver_install(LIDAR_UART, RX_BUFFER_BYTES, 0, 16, &s_uart_events, 0);
    }
    if (error == ESP_OK) {
        error = uart_set_rx_full_threshold(LIDAR_UART, 16);
    }
    if (error == ESP_OK) {
        error = uart_set_rx_timeout(LIDAR_UART, 2);
    }
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "UART setup failed: %s", esp_err_to_name(error));
        return;
    }
    gpio_pulldown_dis(pin);
    gpio_pullup_en(pin);
    ESP_LOGI(TAG, "UART1 receive-only: radar Tx -> GPIO%d, baud=%d, 8N1; PWM untouched",
             pin, CONFIG_SMARTCAR_LIDAR_BAUD);
    if (xTaskCreate(lidar_uart_task, "lidar_uart", 6144, NULL, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Unable to create UART acquisition task");
        uart_driver_delete(LIDAR_UART);
    }
}
#else
void lidar_uart_start(void)
{
    ESP_LOGI("lidar_uart", "UART diagnostics disabled in menuconfig");
}
#endif
