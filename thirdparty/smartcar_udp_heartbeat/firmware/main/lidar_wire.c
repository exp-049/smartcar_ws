#include "lidar_wire.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static uint32_t crc32_update(uint32_t crc, const uint8_t *bytes, size_t length)
{
    for (size_t index = 0; index < length; ++index) {
        crc ^= bytes[index];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return crc;
}

static void put_little(uint8_t *output, uint64_t value, size_t length)
{
    for (size_t index = 0; index < length; ++index) {
        output[index] = (uint8_t)(value >> (index * 8));
    }
}

int lidar_wire_encode(char *output, size_t capacity, const char *boot_id,
                       uint32_t sequence, int64_t rx_complete_us,
                       const uint8_t *raw_packet, size_t raw_length)
{
    if (output == NULL || capacity == 0 || boot_id == NULL || strlen(boot_id) != 16 ||
        raw_packet == NULL || raw_length < 12 || raw_length > LIDAR_WIRE_RAW_MAX ||
        rx_complete_us < 0) {
        return -1;
    }
    uint64_t boot_value = 0;
    for (size_t index = 0; index < 16; ++index) {
        char digit = boot_id[index];
        unsigned value;
        if (digit >= '0' && digit <= '9') {
            value = digit - '0';
        } else if (digit >= 'a' && digit <= 'f') {
            value = digit - 'a' + 10;
        } else {
            return -1;
        }
        boot_value = (boot_value << 4) | value;
    }

    /* Python equivalent: struct.pack('<4sQIQH', b'SC1L', boot, seq, t, n). */
    uint8_t metadata[26];
    memcpy(metadata, "SC1L", 4);
    put_little(metadata + 4, boot_value, 8);
    put_little(metadata + 12, sequence, 4);
    put_little(metadata + 16, (uint64_t)rx_complete_us, 8);
    put_little(metadata + 24, raw_length, 2);
    uint32_t crc = crc32_update(0xFFFFFFFFU, metadata, sizeof(metadata));
    crc = crc32_update(crc, raw_packet, raw_length) ^ 0xFFFFFFFFU;

    int prefix = snprintf(output, capacity,
        "{\"type\":\"lidar_packet\",\"version\":1,\"boot_id\":\"%s\","
        "\"seq\":%" PRIu32 ",\"rx_complete_us\":%" PRId64 ",\"data_hex\":\"",
        boot_id, sequence, rx_complete_us);
    if (prefix < 0 || (size_t)prefix >= capacity) {
        return -1;
    }
    /* Tail is 21 bytes plus NUL; reserve enough before writing the hex. */
    if ((size_t)prefix + raw_length * 2 + 22 > capacity) {
        return -1;
    }
    static const char hex[] = "0123456789abcdef";
    size_t position = (size_t)prefix;
    for (size_t index = 0; index < raw_length; ++index) {
        output[position++] = hex[raw_packet[index] >> 4];
        output[position++] = hex[raw_packet[index] & 0x0F];
    }
    int tail = snprintf(output + position, capacity - position,
                        "\",\"crc32\":\"%08" PRIx32 "\"}", crc);
    if (tail < 0 || (size_t)tail >= capacity - position ||
        position + (size_t)tail > LIDAR_WIRE_DATAGRAM_MAX) {
        return -1;
    }
    return (int)(position + (size_t)tail);
}
