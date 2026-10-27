#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

/* Initialize before Wi-Fi starts. AUTO_DISCOVERY=y starts with no route;
 * otherwise the configured static IPv4/port is validated once. */
esp_err_t host_route_init(void);

/* The Wi-Fi event handler should call true on GotIP and false on disconnect or
 * LostIP. Each transition invalidates old queue generations. In static mode a
 * reconnect enables the configured route again; dynamic mode needs a new lease.
 */
void host_route_set_wifi_ready(bool ready);
/* Revokes the route and marks Wi-Fi not ready, equivalent to set(false). */
void host_route_clear(void);

/* An atomic route/generation snapshot. Both output pointers are optional.
 * False means uninitialized, Wi-Fi down, or no unexpired subscription. The
 * generation output is written even on false; destination is zeroed on false.
 * Capture and sender must compare generations to drop old queued packets.
 */
bool host_route_get(struct sockaddr_in *destination, uint32_t *generation);
bool host_route_authorized(struct in_addr source);

/* Control-plane helpers; source comes from recvfrom, never from the payload.
 * Return discovery_status_t numeric values 0..3. Only dynamic mode accepts
 * claims/releases; the server validates boot ID and release port beforehand.
 */
uint16_t host_route_claim(struct in_addr source, uint64_t session_id, uint16_t data_port);
uint16_t host_route_release(struct in_addr source, uint64_t session_id);
