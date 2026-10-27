#include "esp32_serial_transport.h"

#include "driver/uart.h"
#include "sdkconfig.h"

#define CHASSIS_UROS_UART UART_NUM_0
#define CHASSIS_UROS_BAUD 460800
#define CHASSIS_UROS_RX_BUFFER 4096

bool smartcar_serial_open(struct uxrCustomTransport *transport)
{
    (void)transport;
    const uart_config_t config = {
        .baud_rate = CHASSIS_UROS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_param_config(CHASSIS_UROS_UART, &config) != ESP_OK) return false;
    if (uart_set_pin(CHASSIS_UROS_UART, CONFIG_MICROROS_UART_TXD,
                     CONFIG_MICROROS_UART_RXD, CONFIG_MICROROS_UART_RTS,
                     CONFIG_MICROROS_UART_CTS) != ESP_OK) return false;
    return uart_driver_install(CHASSIS_UROS_UART, CHASSIS_UROS_RX_BUFFER,
                               CHASSIS_UROS_RX_BUFFER, 0, NULL, 0) == ESP_OK;
}

bool smartcar_serial_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    esp_err_t result = uart_driver_delete(CHASSIS_UROS_UART);
    return result == ESP_OK || result == ESP_ERR_INVALID_STATE;
}

size_t smartcar_serial_write(struct uxrCustomTransport *transport,
                             const uint8_t *buffer, size_t length,
                             uint8_t *error_code)
{
    (void)transport;
    int written = uart_write_bytes(CHASSIS_UROS_UART, (const char *)buffer, length);
    if (written < 0) {
        if (error_code) *error_code = 1;
        return 0;
    }
    if (error_code) *error_code = 0;
    return (size_t)written;
}

size_t smartcar_serial_read(struct uxrCustomTransport *transport,
                            uint8_t *buffer, size_t length, int timeout_ms,
                            uint8_t *error_code)
{
    (void)transport;
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms > 0 ? timeout_ms : 0);
    int received = uart_read_bytes(CHASSIS_UROS_UART, buffer, length, timeout_ticks);
    if (received < 0) {
        if (error_code) *error_code = 1;
        return 0;
    }
    if (error_code) *error_code = 0;
    return (size_t)received;
}
