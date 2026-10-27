#include "mpu6050.h"
#include "sensor_wire.h"
#include "host_route.h"

#include "esp_log.h"
#include "sdkconfig.h"

#ifdef CONFIG_SMARTCAR_IMU_ENABLE
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

/* MPU-6000/6050 register map RM-MPU-6000A-00, rev 4.2. INT_RD_CLEAR=0:
 * only reading INT_STATUS clears DATA_RDY. A burst reads the sensor shadow
 * registers coherently. CONFIG=3 gives a 1 kHz gyro output; divider 19 gives
 * nominal 50 Hz. DLPF delay (~4.9 ms accel/~4.8 ms gyro) is NOT removed from
 * timestamps or included in the observation uncertainty.
 */
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_FIFO_EN 0x23
#define REG_INT_PIN_CFG 0x37
#define REG_INT_ENABLE 0x38
#define REG_INT_STATUS 0x3A
#define REG_ACCEL_XOUT_H 0x3B
#define REG_USER_CTRL 0x6A
#define REG_PWR_MGMT_1 0x6B
#define REG_PWR_MGMT_2 0x6C
#define REG_WHO_AM_I 0x75
#define DATA_READY_BIT 0x01
#define POLL_INTERVAL_US 2000
#define NOMINAL_PERIOD_US 20000
#define MAX_READY_BRACKET_US 6000
/* The IDF synchronous driver converts milliseconds to FreeRTOS ticks. A 5 ms
 * timeout becomes zero ticks at the default 100 Hz and can fail before the
 * transfer interrupt arrives. This is only a failure timeout; healthy reads
 * still complete immediately and acquisition timing is checked separately. */
#define I2C_TIMEOUT_MS 30
#define IMU_QUEUE_DEPTH 16
#define MAX_QUEUE_AGE_US 250000

typedef struct {
    uint32_t sequence;
    uint32_t route_generation;
    int64_t sample_us;
    uint32_t uncertainty_us;
    int64_t read_end_us;
    uint8_t raw[IMU_WIRE_RAW_BYTES];
} imu_item_t;

typedef struct {
    uint64_t ready_events;
    uint64_t captured;
    uint64_t sent;
    uint64_t bracket_drops;
    uint64_t read_boundary_drops;
    uint64_t i2c_errors;
    uint64_t coalesced_polls;
    uint64_t queue_drops;
    uint64_t wifi_drops;
    uint64_t no_host_drops;
    uint64_t old_drops;
    uint64_t send_errors;
    uint64_t encode_errors;
} imu_stats_t;

static const char *TAG = "mpu6050";
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_device;
static QueueHandle_t s_queue;
static TaskHandle_t s_capture_task;
static TaskHandle_t s_sender_task;
static esp_timer_handle_t s_poll_timer;
static EventGroupHandle_t s_wifi_events;
static EventBits_t s_ready_bit;
static char s_boot_id[17];
static uint32_t s_sequence;
static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static imu_stats_t s_stats;

static void add_counter(uint64_t *counter, uint64_t count)
{
    portENTER_CRITICAL(&s_stats_lock);
    *counter += count;
    portEXIT_CRITICAL(&s_stats_lock);
}

static esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(s_device, &reg, 1, data, length, I2C_TIMEOUT_MS);
}

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t bytes[] = {reg, value};
    return i2c_master_transmit(s_device, bytes, sizeof(bytes), I2C_TIMEOUT_MS);
}

static esp_err_t configure_device(void)
{
    esp_err_t error = write_register(REG_PWR_MGMT_1, 0x80);
    if (error != ESP_OK) {
        return error;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    /* PLL with X gyro reference, temperature enabled, all sensor axes awake. */
    error = write_register(REG_PWR_MGMT_1, 0x01);
    if (error != ESP_OK) {
        return error;
    }
    vTaskDelay(pdMS_TO_TICKS(30));
    const uint8_t settings[][2] = {
        {REG_PWR_MGMT_2, 0x00}, {REG_USER_CTRL, 0x00}, {REG_FIFO_EN, 0x00},
        {REG_CONFIG, 0x03}, {REG_GYRO_CONFIG, 0x00}, {REG_ACCEL_CONFIG, 0x00},
        {REG_SMPLRT_DIV, 19}, {REG_INT_PIN_CFG, 0x20}, {REG_INT_ENABLE, DATA_READY_BIT},
    };
    for (size_t index = 0; index < sizeof(settings) / sizeof(settings[0]); ++index) {
        error = write_register(settings[index][0], settings[index][1]);
        if (error != ESP_OK) {
            return error;
        }
        uint8_t actual = 0xFF;
        error = read_registers(settings[index][0], &actual, 1);
        if (error != ESP_OK || actual != settings[index][1]) {
            ESP_LOGE(TAG, "Configuration verification failed at register 0x%02x", settings[index][0]);
            return error != ESP_OK ? error : ESP_ERR_INVALID_RESPONSE;
        }
    }
    uint8_t ignored;
    return read_registers(REG_INT_STATUS, &ignored, 1);
}

/* Runs in the esp_timer task, never performs I2C or network operations. */
static void poll_timer_callback(void *argument)
{
    (void)argument;
    if (s_capture_task != NULL) {
        xTaskNotifyGive(s_capture_task);
    }
}

static void capture_task(void *argument)
{
    (void)argument;
    int64_t not_ready_start_us = 0;
    while (true) {
        uint32_t ticks = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ticks > 1) {
            add_counter(&s_stats.coalesced_polls, ticks - 1);
        }
        uint8_t status = 0;
        int64_t poll_start_us = esp_timer_get_time();
        esp_err_t error = read_registers(REG_INT_STATUS, &status, 1);
        int64_t poll_end_us = esp_timer_get_time();
        if (error != ESP_OK) {
            add_counter(&s_stats.i2c_errors, 1);
            /* The failed transaction may have cleared INT_STATUS. */
            not_ready_start_us = 0;
            continue;
        }
        if (!(status & DATA_READY_BIT)) {
            /* The bit could change during this transaction: start is a safe
             * lower observation bound, rather than its end. Keep the latest. */
            not_ready_start_us = poll_start_us;
            continue;
        }
        imu_item_t item;
        item.sequence = s_sequence++;  /* Per ready event, including local drops. */
        add_counter(&s_stats.ready_events, 1);
        int64_t lower_us = not_ready_start_us;
        not_ready_start_us = 0;  /* Require an observed not-ready before reuse. */
        int64_t width_us = poll_end_us - lower_us;
        if (lower_us <= 0 || width_us <= 0 || width_us > MAX_READY_BRACKET_US) {
            add_counter(&s_stats.bracket_drops, 1);
            continue;
        }
        item.sample_us = lower_us + width_us / 2;
        item.uncertainty_us = (uint32_t)((width_us + 1) / 2);
        error = read_registers(REG_ACCEL_XOUT_H, item.raw, sizeof(item.raw));
        item.read_end_us = esp_timer_get_time();
        if (error != ESP_OK) {
            add_counter(&s_stats.i2c_errors, 1);
            continue;
        }
        /* Reject a delayed burst that can cross the earliest next nominal
         * 20 ms update. No FIFO or edge pin exists to recover missed updates.
         * Identical raw values are legitimate; DATA_RDY (not byte changes)
         * prevents sending the same ready observation twice.
         */
        if (item.read_end_us - lower_us >= NOMINAL_PERIOD_US ||
            item.read_end_us < poll_end_us) {
            add_counter(&s_stats.read_boundary_drops, 1);
            continue;
        }
        add_counter(&s_stats.captured, 1);
        struct sockaddr_in destination;
        if (!(xEventGroupGetBits(s_wifi_events) & s_ready_bit)) {
            add_counter(&s_stats.wifi_drops, 1);
        } else if (!host_route_get(&destination, &item.route_generation)) {
            add_counter(&s_stats.no_host_drops, 1);
        } else if (xQueueSend(s_queue, &item, 0) != pdPASS) {
            add_counter(&s_stats.queue_drops, 1);
        }
    }
}

static void sender_task(void *argument)
{
    (void)argument;
    int sock = -1;
    int64_t report_us = esp_timer_get_time();
    imu_item_t item;
    char message[IMU_WIRE_DATAGRAM_MAX + 1];
    while (true) {
        if (xQueueReceive(s_queue, &item, pdMS_TO_TICKS(100)) == pdPASS) {
            struct sockaddr_in destination;
            uint32_t generation;
            bool have_host = host_route_get(&destination, &generation);
            if (!(xEventGroupGetBits(s_wifi_events) & s_ready_bit)) {
                add_counter(&s_stats.wifi_drops, 1);
            } else if (!have_host || generation != item.route_generation) {
                add_counter(&s_stats.no_host_drops, 1);
            } else if (esp_timer_get_time() - item.read_end_us > MAX_QUEUE_AGE_US) {
                add_counter(&s_stats.old_drops, 1);
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
                int length = imu_wire_encode(message, sizeof(message), s_boot_id, item.sequence,
                                              item.sample_us, item.uncertainty_us, item.read_end_us,
                                              item.raw, sizeof(item.raw));
                if (length < 0) {
                    add_counter(&s_stats.encode_errors, 1);
                } else if (sock < 0) {
                    add_counter(&s_stats.send_errors, 1);
                } else if (sendto(sock, message, (size_t)length, 0,
                                  (const struct sockaddr *)&destination, sizeof(destination)) != length) {
                    add_counter(&s_stats.send_errors, 1);
                    close(sock);
                    sock = -1;
                } else {
                    add_counter(&s_stats.sent, 1);
                }
            }
        }
        int64_t now_us = esp_timer_get_time();
        if (now_us - report_us >= 1000000) {
            imu_stats_t stats;
            portENTER_CRITICAL(&s_stats_lock);
            stats = s_stats;
            portEXIT_CRITICAL(&s_stats_lock);
            ESP_LOGI(TAG, "ready=%" PRIu64 " captured=%" PRIu64 " sent=%" PRIu64
                     " bracket_drop=%" PRIu64 " read_boundary_drop=%" PRIu64
                     " i2c_err=%" PRIu64 " poll_coalesced=%" PRIu64
                     " queue_drop=%" PRIu64 " wifi_drop=%" PRIu64 " no_host_drop=%" PRIu64 " old_drop=%" PRIu64
                     " send_err=%" PRIu64 " encode_err=%" PRIu64,
                     stats.ready_events, stats.captured, stats.sent, stats.bracket_drops,
                     stats.read_boundary_drops, stats.i2c_errors, stats.coalesced_polls,
                     stats.queue_drops, stats.wifi_drops, stats.no_host_drops, stats.old_drops,
                     stats.send_errors, stats.encode_errors);
            report_us = now_us;
        }
    }
}

static void release_initialization(void)
{
    if (s_poll_timer != NULL) {
        esp_timer_stop(s_poll_timer);
        esp_timer_delete(s_poll_timer);
        s_poll_timer = NULL;
    }
    if (s_capture_task != NULL) {
        vTaskDelete(s_capture_task);
        s_capture_task = NULL;
    }
    if (s_sender_task != NULL) {
        vTaskDelete(s_sender_task);
        s_sender_task = NULL;
    }
    if (s_queue != NULL) {
        vQueueDelete(s_queue);
        s_queue = NULL;
    }
    if (s_device != NULL) {
        i2c_master_bus_rm_device(s_device);
        s_device = NULL;
    }
    if (s_bus != NULL) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
}

esp_err_t mpu6050_start(EventGroupHandle_t wifi_events, EventBits_t ready_bit, const char *boot_id)
{
    uint64_t ignored_boot;
    if (s_bus != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (wifi_events == NULL || ready_bit == 0 ||
        !sensor_wire_boot_id_value(boot_id, &ignored_boot)) {
        return ESP_ERR_INVALID_ARG;
    }
    const int scl = CONFIG_SMARTCAR_IMU_SCL_GPIO;
    const int sda = CONFIG_SMARTCAR_IMU_SDA_GPIO;
    if (scl == sda || !GPIO_IS_VALID_OUTPUT_GPIO(scl) || !GPIO_IS_VALID_OUTPUT_GPIO(sda) ||
        scl == 19 || scl == 20 || sda == 19 || sda == 20 ||
        (scl >= 35 && scl <= 37) || (sda >= 35 && sda <= 37)) {
        return ESP_ERR_INVALID_ARG;
    }
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = -1,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t error = i2c_new_master_bus(&bus_config, &s_bus);
    if (error != ESP_OK) {
        return error;
    }
    unsigned address;
    for (address = 0x68; address <= 0x69; ++address) {
        if (i2c_master_probe(s_bus, address, I2C_TIMEOUT_MS) != ESP_OK) {
            continue;
        }
        const i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = address,
            .scl_speed_hz = 400000,
        };
        error = i2c_master_bus_add_device(s_bus, &config, &s_device);
        if (error != ESP_OK) {
            release_initialization();
            return error;
        }
        uint8_t identity = 0;
        error = read_registers(REG_WHO_AM_I, &identity, 1);
        if (error == ESP_OK && identity == 0x68) {
            break;
        }
        i2c_master_bus_rm_device(s_device);
        s_device = NULL;
    }
    if (s_device == NULL) {
        ESP_LOGE(TAG, "No MPU6050 WHO_AM_I=0x68 at I2C 0x68/0x69; check power, SCL/SDA, AD0");
        release_initialization();
        return ESP_ERR_NOT_FOUND;
    }
    error = configure_device();
    if (error != ESP_OK) {
        release_initialization();
        return error;
    }
    s_wifi_events = wifi_events;
    s_ready_bit = ready_bit;
    memcpy(s_boot_id, boot_id, sizeof(s_boot_id));
    s_sequence = 0;
    memset(&s_stats, 0, sizeof(s_stats));
    s_queue = xQueueCreate(IMU_QUEUE_DEPTH, sizeof(imu_item_t));
    if (s_queue == NULL) {
        release_initialization();
        return ESP_ERR_NO_MEM;
    }
    const esp_timer_create_args_t timer_args = {
        .callback = poll_timer_callback,
        .name = "imu_poll",
        .skip_unhandled_events = true,
    };
    error = esp_timer_create(&timer_args, &s_poll_timer);
    if (error != ESP_OK) {
        release_initialization();
        return error;
    }
    if (xTaskCreate(capture_task, "imu_capture", 4096, NULL, 8, &s_capture_task) != pdPASS ||
        xTaskCreate(sender_task, "imu_udp", 4096, NULL, 4, &s_sender_task) != pdPASS) {
        release_initialization();
        return ESP_ERR_NO_MEM;
    }
    error = esp_timer_start_periodic(s_poll_timer, POLL_INTERVAL_US);
    if (error != ESP_OK) {
        release_initialization();
        return error;
    }
    ESP_LOGI(TAG, "I2C address=0x%02x SCL=%d SDA=%d; +/-2g +/-250dps DLPF=3 nominal=50Hz",
             address, scl, sda);
    ESP_LOGI(TAG, "No INT: 2ms polling, ready bracket <=%dus; midpoint timestamp includes DLPF delay; AD0 needs stable level",
             MAX_READY_BRACKET_US);
    return ESP_OK;
}
#else
esp_err_t mpu6050_start(EventGroupHandle_t wifi_events, EventBits_t ready_bit, const char *boot_id)
{
    (void)wifi_events;
    (void)ready_bit;
    (void)boot_id;
    ESP_LOGI("mpu6050", "IMU disabled in menuconfig");
    return ESP_OK;
}
#endif
