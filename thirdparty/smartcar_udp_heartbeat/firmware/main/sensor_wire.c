#include "sensor_wire.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static void put_le(uint8_t *output, uint64_t value, size_t length)
{
    for (size_t index = 0; index < length; ++index) {
        output[index] = (uint8_t)(value >> (index * 8));
    }
}

static uint64_t get_u64_le(const uint8_t *input)
{
    uint64_t value = 0;
    for (size_t index = 0; index < 8; ++index) {
        value |= (uint64_t)input[index] << (index * 8);
    }
    return value;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *bytes, size_t length)
{
    for (size_t index = 0; index < length; ++index) {
        crc ^= bytes[index];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return crc;
}

bool sensor_wire_boot_id_value(const char *boot_id, uint64_t *value)
{
    if (boot_id == NULL || value == NULL || strlen(boot_id) != 16) {
        return false;
    }
    uint64_t parsed = 0;
    for (size_t index = 0; index < 16; ++index) {
        unsigned digit;
        if (boot_id[index] >= '0' && boot_id[index] <= '9') {
            digit = (unsigned)(boot_id[index] - '0');
        } else if (boot_id[index] >= 'a' && boot_id[index] <= 'f') {
            digit = (unsigned)(boot_id[index] - 'a' + 10);
        } else {
            return false;
        }
        parsed = (parsed << 4) | digit;
    }
    *value = parsed;
    return true;
}

int imu_wire_encode(char *output, size_t capacity, const char *boot_id,
                    uint32_t sequence, int64_t sample_us, uint32_t uncertainty_us,
                    int64_t read_end_us, const uint8_t *raw, size_t raw_length)
{
    uint64_t boot_value;
    if (output == NULL || capacity == 0 || raw == NULL || raw_length != IMU_WIRE_RAW_BYTES ||
        sample_us < 0 || read_end_us < sample_us ||
        !sensor_wire_boot_id_value(boot_id, &boot_value)) {
        return -1;
    }
    uint8_t metadata[38];
    memcpy(metadata, "SC1I", 4);
    put_le(metadata + 4, boot_value, 8);
    put_le(metadata + 12, sequence, 4);
    put_le(metadata + 16, (uint64_t)sample_us, 8);
    put_le(metadata + 24, uncertainty_us, 4);
    put_le(metadata + 28, (uint64_t)read_end_us, 8);
    put_le(metadata + 36, IMU_WIRE_RAW_BYTES, 2);
    uint32_t crc = crc32_update(0xFFFFFFFFU, metadata, sizeof(metadata));
    crc = crc32_update(crc, raw, raw_length) ^ 0xFFFFFFFFU;

    char data_hex[IMU_WIRE_RAW_BYTES * 2 + 1];
    static const char hex[] = "0123456789abcdef";
    for (size_t index = 0; index < raw_length; ++index) {
        data_hex[index * 2] = hex[raw[index] >> 4];
        data_hex[index * 2 + 1] = hex[raw[index] & 15];
    }
    data_hex[sizeof(data_hex) - 1] = '\0';
    int length = snprintf(output, capacity,
        "{\"type\":\"imu_packet\",\"version\":1,\"boot_id\":\"%s\",\"seq\":%" PRIu32
        ",\"sample_us\":%" PRId64 ",\"uncertainty_us\":%" PRIu32
        ",\"read_end_us\":%" PRId64 ",\"data_hex\":\"%s\",\"crc32\":\"%08" PRIx32 "\"}",
        boot_id, sequence, sample_us, uncertainty_us, read_end_us, data_hex, crc);
    if (length < 0 || (size_t)length >= capacity || length > IMU_WIRE_DATAGRAM_MAX) {
        return -1;
    }
    return length;
}

bool sensor_wire_sync_request_decode(const uint8_t *request, size_t length,
                                     uint64_t *nonce, uint64_t *host_t1_ns)
{
    if (request == NULL || nonce == NULL || host_t1_ns == NULL ||
        length != SENSOR_SYNC_REQUEST_BYTES || memcmp(request, "SCQ1", 4) != 0) {
        return false;
    }
    *nonce = get_u64_le(request + 4);
    *host_t1_ns = get_u64_le(request + 12);
    return true;
}

int sensor_wire_sync_response_encode(uint8_t *output, size_t capacity,
                                     uint64_t nonce, uint64_t host_t1_ns,
                                     int64_t device_t2_us, int64_t device_t3_us,
                                     uint64_t boot_value)
{
    if (output == NULL || capacity < SENSOR_SYNC_RESPONSE_BYTES ||
        device_t2_us < 0 || device_t3_us < device_t2_us) {
        return -1;
    }
    memcpy(output, "SCR1", 4);
    put_le(output + 4, nonce, 8);
    put_le(output + 12, host_t1_ns, 8);
    put_le(output + 20, (uint64_t)device_t2_us, 8);
    put_le(output + 28, (uint64_t)device_t3_us, 8);
    put_le(output + 36, boot_value, 8);
    return SENSOR_SYNC_RESPONSE_BYTES;
}
