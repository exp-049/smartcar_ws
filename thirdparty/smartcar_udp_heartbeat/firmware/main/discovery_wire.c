#include "discovery_wire.h"

#include <string.h>

static uint64_t get_le(const uint8_t *input, size_t length)
{
    uint64_t value = 0;
    for (size_t index = 0; index < length; ++index) {
        value |= (uint64_t)input[index] << (index * 8);
    }
    return value;
}

static void put_le(uint8_t *output, uint64_t value, size_t length)
{
    for (size_t index = 0; index < length; ++index) {
        output[index] = (uint8_t)(value >> (index * 8));
    }
}

bool discovery_wire_boot_id_value(const char *boot_id, uint64_t *value)
{
    if (boot_id == NULL || value == NULL || strlen(boot_id) != 16) {
        return false;
    }
    uint64_t result = 0;
    for (size_t index = 0; index < 16; ++index) {
        unsigned digit;
        if (boot_id[index] >= '0' && boot_id[index] <= '9') {
            digit = (unsigned)(boot_id[index] - '0');
        } else if (boot_id[index] >= 'a' && boot_id[index] <= 'f') {
            digit = (unsigned)(boot_id[index] - 'a' + 10);
        } else {
            return false;
        }
        result = (result << 4) | digit;
    }
    *value = result;
    return true;
}

bool discovery_wire_request_decode(const uint8_t *data, size_t length,
                                    discovery_request_t *request)
{
    if (data == NULL || request == NULL || length < 4) {
        return false;
    }
    discovery_request_t parsed = {0};
    if (memcmp(data, "SCD1", 4) == 0 && length == DISCOVERY_REQUEST_BYTES) {
        parsed.kind = DISCOVERY_DISCOVER;
        parsed.nonce = get_le(data + 4, 8);
    } else if (length == DISCOVERY_CONTROL_BYTES &&
               (memcmp(data, "SCS1", 4) == 0 || memcmp(data, "SCE1", 4) == 0)) {
        parsed.kind = data[2] == 'S' ? DISCOVERY_SUBSCRIBE : DISCOVERY_RELEASE;
        parsed.nonce = get_le(data + 4, 8);
        parsed.boot_id = get_le(data + 12, 8);
        parsed.session_id = get_le(data + 20, 8);
        parsed.data_port = (uint16_t)get_le(data + 28, 2);
        if (parsed.session_id == 0) {
            return false;
        }
    } else {
        return false;
    }
    *request = parsed;
    return true;
}

int discovery_wire_announce_encode(uint8_t *output, size_t capacity,
                                   uint64_t nonce, uint64_t device_id,
                                   uint64_t boot_id, uint16_t sync_port,
                                   uint16_t capabilities)
{
    if (output == NULL || capacity < DISCOVERY_ANNOUNCE_BYTES || sync_port == 0 ||
        (device_id >> 48) != 0 || (capabilities & ~3U) != 0) {
        return -1;
    }
    memcpy(output, "SCA1", 4);
    put_le(output + 4, nonce, 8);
    put_le(output + 12, device_id, 8);
    put_le(output + 20, boot_id, 8);
    put_le(output + 28, sync_port, 2);
    put_le(output + 30, capabilities, 2);
    return DISCOVERY_ANNOUNCE_BYTES;
}

int discovery_wire_ack_encode(uint8_t *output, size_t capacity, uint64_t nonce,
                              uint64_t device_id, uint64_t boot_id,
                              uint64_t session_id, uint16_t lease_ms,
                              discovery_status_t status)
{
    if (output == NULL || capacity < DISCOVERY_ACK_BYTES || session_id == 0 ||
        (device_id >> 48) != 0 || (unsigned)status > DISCOVERY_STATUS_BAD_PORT ||
        (status != DISCOVERY_STATUS_OK && lease_ms != 0) ||
        (lease_ms != 0 && lease_ms != DISCOVERY_LEASE_MS)) {
        return -1;
    }
    memcpy(output, "SCK1", 4);
    put_le(output + 4, nonce, 8);
    put_le(output + 12, device_id, 8);
    put_le(output + 20, boot_id, 8);
    put_le(output + 28, session_id, 8);
    put_le(output + 36, lease_ms, 2);
    put_le(output + 38, status, 2);
    return DISCOVERY_ACK_BYTES;
}

static void next_generation(discovery_lease_t *lease)
{
    if (++lease->generation == 0) {
        lease->generation = 1;
    }
}

void discovery_lease_init(discovery_lease_t *lease)
{
    if (lease != NULL) {
        memset(lease, 0, sizeof(*lease));
        lease->generation = 1;
    }
}

void discovery_lease_clear(discovery_lease_t *lease)
{
    if (lease != NULL) {
        lease->active = false;
        lease->source_ipv4 = 0;
        lease->session_id = 0;
        lease->data_port = 0;
        lease->expires_us = 0;
        next_generation(lease);
    }
}

bool discovery_lease_expire(discovery_lease_t *lease, int64_t now_us)
{
    if (lease != NULL && lease->active && (now_us < 0 || now_us >= lease->expires_us)) {
        discovery_lease_clear(lease);
        return true;
    }
    return false;
}

bool discovery_lease_current(discovery_lease_t *lease, int64_t now_us)
{
    if (lease == NULL) {
        return false;
    }
    discovery_lease_expire(lease, now_us);
    return lease->active;
}

discovery_status_t discovery_lease_claim(discovery_lease_t *lease,
                                        uint32_t source_ipv4, uint64_t session_id,
                                        uint16_t data_port, int64_t now_us)
{
    if (lease == NULL || session_id == 0 || now_us < 0 || now_us > INT64_MAX - DISCOVERY_LEASE_US) {
        return DISCOVERY_STATUS_BUSY;
    }
    if (data_port < 1024) {
        return DISCOVERY_STATUS_BAD_PORT;
    }
    discovery_lease_expire(lease, now_us);
    if (lease->active && (lease->source_ipv4 != source_ipv4 || lease->session_id != session_id)) {
        return DISCOVERY_STATUS_BUSY;
    }
    if (!lease->active || lease->data_port != data_port) {
        next_generation(lease);
    }
    lease->active = true;
    lease->source_ipv4 = source_ipv4;
    lease->session_id = session_id;
    lease->data_port = data_port;
    lease->expires_us = now_us + DISCOVERY_LEASE_US;
    return DISCOVERY_STATUS_OK;
}

discovery_status_t discovery_lease_release(discovery_lease_t *lease,
                                          uint32_t source_ipv4, uint64_t session_id,
                                          int64_t now_us)
{
    if (!discovery_lease_current(lease, now_us) || session_id == 0 ||
        lease->source_ipv4 != source_ipv4 || lease->session_id != session_id) {
        return DISCOVERY_STATUS_BUSY;
    }
    discovery_lease_clear(lease);
    return DISCOVERY_STATUS_OK;
}
