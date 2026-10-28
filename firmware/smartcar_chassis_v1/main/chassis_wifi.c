#include "chassis_wifi.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "chassis_hw.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "host_discovery.h"
#include "host_route.h"
#include "sdkconfig.h"

#define WIFI_READY_BIT BIT0
static EventGroupHandle_t s_wifi_events;
static const char *TAG = "chassis_wifi";

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event = data;
        ESP_LOGW(TAG, "Wi-Fi disconnected, reason=%d; reconnecting", event ? event->reason : -1);
        xEventGroupClearBits(s_wifi_events, WIFI_READY_BIT);
        host_route_set_wifi_ready(false);
        chassis_emergency_stop();
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = data;
        if (event) ESP_LOGI(TAG, "Wi-Fi connected, IPv4=" IPSTR, IP2STR(&event->ip_info.ip));
        host_route_set_wifi_ready(true);
        xEventGroupSetBits(s_wifi_events, WIFI_READY_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        xEventGroupClearBits(s_wifi_events, WIFI_READY_BIT);
        host_route_set_wifi_ready(false);
        chassis_emergency_stop();
    }
}

bool chassis_wifi_route_ready(void)
{
    return host_route_get(NULL, NULL);
}

esp_err_t chassis_wifi_start(void)
{
    const char *ssid = CONFIG_SMARTCAR_WIFI_SSID;
    const char *password = CONFIG_SMARTCAR_WIFI_PASSWORD;
    size_t ssid_len = strlen(ssid), password_len = strlen(password);
    if (ssid_len == 0 || ssid_len > 32 || password_len < 8 || password_len > 64 ||
        CONFIG_SMARTCAR_DISCOVERY_PORT == CONFIG_SMARTCAR_SYNC_PORT) {
        ESP_LOGE(TAG, "Configure a 2.4 GHz SSID and password in local sdkconfig; check UDP ports");
        return ESP_ERR_INVALID_ARG;
    }

    s_wifi_events = xEventGroupCreate();
    if (!s_wifi_events) return ESP_ERR_NO_MEM;
    esp_err_t err = host_route_init();
    if (err != ESP_OK) return err;
    err = esp_netif_init();
    if (err != ESP_OK) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK) return err;
    if (!esp_netif_create_default_wifi_sta()) return ESP_ERR_NO_MEM;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init);
    if (err != ESP_OK) return err;
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    if (err != ESP_OK) return err;
    err = esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    if (err != ESP_OK) return err;
    char boot_id[17];
    uint64_t boot = ((uint64_t)esp_random() << 32) | esp_random();
    snprintf(boot_id, sizeof(boot_id), "%016" PRIx64, boot);
    err = host_discovery_start(s_wifi_events, WIFI_READY_BIT, boot_id);
    if (err != ESP_OK) return err;
    wifi_config_t config = {0};
    memcpy(config.sta.ssid, ssid, ssid_len);
    memcpy(config.sta.password, password, password_len);
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_config(WIFI_IF_STA, &config);
    if (err != ESP_OK) return err;
    return esp_wifi_start();
}
