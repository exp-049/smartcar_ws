#include "host_route.h"
#include "discovery_wire.h"

#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_initialized;
static bool s_wifi_ready;
static discovery_lease_t s_lease;
#ifndef CONFIG_SMARTCAR_AUTO_DISCOVERY
static struct sockaddr_in s_static_destination;
#endif

esp_err_t host_route_init(void)
{
#ifndef CONFIG_SMARTCAR_AUTO_DISCOVERY
    struct sockaddr_in destination = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_SMARTCAR_HOST_PORT),
    };
    if (CONFIG_SMARTCAR_HOST_PORT < 1 || CONFIG_SMARTCAR_HOST_PORT > 65535 ||
        inet_pton(AF_INET, CONFIG_SMARTCAR_HOST_IPV4, &destination.sin_addr) != 1) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t address = ntohl(destination.sin_addr.s_addr);
    if (address == INADDR_ANY || address == INADDR_BROADCAST ||
        (address >> 24) == 127 || (address >> 28) >= 14) {
        return ESP_ERR_INVALID_ARG;
    }
#endif
    portENTER_CRITICAL(&s_lock);
    if (s_initialized) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    discovery_lease_init(&s_lease);
    s_wifi_ready = false;
#ifndef CONFIG_SMARTCAR_AUTO_DISCOVERY
    s_static_destination = destination;
#endif
    s_initialized = true;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

void host_route_set_wifi_ready(bool ready)
{
    portENTER_CRITICAL(&s_lock);
    if (s_initialized && s_wifi_ready != ready) {
        s_wifi_ready = ready;
        discovery_lease_clear(&s_lease);
    }
    portEXIT_CRITICAL(&s_lock);
}

void host_route_clear(void)
{
    host_route_set_wifi_ready(false);
}

bool host_route_get(struct sockaddr_in *destination, uint32_t *generation)
{
    struct sockaddr_in route = {0};
    bool active = false;
    portENTER_CRITICAL(&s_lock);
    if (s_initialized && s_wifi_ready) {
#ifdef CONFIG_SMARTCAR_AUTO_DISCOVERY
        if (discovery_lease_current(&s_lease, esp_timer_get_time())) {
            route.sin_family = AF_INET;
            route.sin_port = htons(s_lease.data_port);
            route.sin_addr.s_addr = s_lease.source_ipv4;
            active = true;
        }
#else
        route = s_static_destination;
        active = true;
#endif
    }
    if (destination != NULL) {
        *destination = route;
    }
    if (generation != NULL) {
        *generation = s_lease.generation;
    }
    portEXIT_CRITICAL(&s_lock);
    return active;
}

bool host_route_authorized(struct in_addr source)
{
    struct sockaddr_in destination;
    return host_route_get(&destination, NULL) && destination.sin_addr.s_addr == source.s_addr;
}

uint16_t host_route_claim(struct in_addr source, uint64_t session_id, uint16_t data_port)
{
    discovery_status_t status = DISCOVERY_STATUS_BUSY;
#ifdef CONFIG_SMARTCAR_AUTO_DISCOVERY
    portENTER_CRITICAL(&s_lock);
    if (s_initialized && s_wifi_ready) {
        status = discovery_lease_claim(&s_lease, source.s_addr, session_id,
                                       data_port, esp_timer_get_time());
    }
    portEXIT_CRITICAL(&s_lock);
#else
    (void)source;
    (void)session_id;
    (void)data_port;
#endif
    return (uint16_t)status;
}

uint16_t host_route_release(struct in_addr source, uint64_t session_id)
{
    discovery_status_t status = DISCOVERY_STATUS_BUSY;
#ifdef CONFIG_SMARTCAR_AUTO_DISCOVERY
    portENTER_CRITICAL(&s_lock);
    if (s_initialized && s_wifi_ready) {
        status = discovery_lease_release(&s_lease, source.s_addr, session_id,
                                         esp_timer_get_time());
    }
    portEXIT_CRITICAL(&s_lock);
#else
    (void)source;
    (void)session_id;
#endif
    return (uint16_t)status;
}
