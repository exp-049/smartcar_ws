#pragma once

/* Native-testable discovery codec and single-subscriber lease policy.
 * No native structs are copied onto the wire: every integer is explicit LE.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DISCOVERY_REQUEST_BYTES 12
#define DISCOVERY_CONTROL_BYTES 30
#define DISCOVERY_ANNOUNCE_BYTES 32
#define DISCOVERY_ACK_BYTES 40
#define DISCOVERY_LEASE_MS 6000
#define DISCOVERY_LEASE_US INT64_C(6000000)
#define DISCOVERY_CAPABILITY_LIDAR 1
#define DISCOVERY_CAPABILITY_IMU 2

typedef enum {
    DISCOVERY_DISCOVER,
    DISCOVERY_SUBSCRIBE,
    DISCOVERY_RELEASE,
} discovery_request_kind_t;

typedef enum {
    DISCOVERY_STATUS_OK = 0,
    DISCOVERY_STATUS_BUSY = 1,
    DISCOVERY_STATUS_WRONG_BOOT = 2,
    DISCOVERY_STATUS_BAD_PORT = 3,
} discovery_status_t;

typedef struct {
    discovery_request_kind_t kind;
    uint64_t nonce;
    uint64_t boot_id;
    uint64_t session_id;
    uint16_t data_port;
} discovery_request_t;

/* Strict length/magic; SCS1/SCE1 require nonzero session_id. Port validity is
 * checked separately so a well-formed bad-port request can receive status 3.
 */
bool discovery_wire_request_decode(const uint8_t *data, size_t length,
                                    discovery_request_t *request);
bool discovery_wire_boot_id_value(const char *boot_id, uint64_t *value);
int discovery_wire_announce_encode(uint8_t *output, size_t capacity,
                                   uint64_t nonce, uint64_t device_id,
                                   uint64_t boot_id, uint16_t sync_port,
                                   uint16_t capabilities);
int discovery_wire_ack_encode(uint8_t *output, size_t capacity, uint64_t nonce,
                              uint64_t device_id, uint64_t boot_id,
                              uint64_t session_id, uint16_t lease_ms,
                              discovery_status_t status);

typedef struct {
    bool active;
    /* Opaque IPv4 bytes, compared without endianness conversion. */
    uint32_t source_ipv4;
    uint64_t session_id;
    uint16_t data_port;
    int64_t expires_us;
    uint32_t generation;
} discovery_lease_t;

void discovery_lease_init(discovery_lease_t *lease);
/* Explicit invalidation always advances generation; zero is reserved for an
 * uninitialized route. Renewing an unchanged route does not advance it. */
void discovery_lease_clear(discovery_lease_t *lease);
bool discovery_lease_expire(discovery_lease_t *lease, int64_t now_us);
bool discovery_lease_current(discovery_lease_t *lease, int64_t now_us);
/* Wrong boot is handled by the server before changing policy. Malformed zero
 * sessions are discarded by the codec; direct callers must also reject them.
 */
discovery_status_t discovery_lease_claim(discovery_lease_t *lease,
                                        uint32_t source_ipv4, uint64_t session_id,
                                        uint16_t data_port, int64_t now_us);
discovery_status_t discovery_lease_release(discovery_lease_t *lease,
                                          uint32_t source_ipv4, uint64_t session_id,
                                          int64_t now_us);
