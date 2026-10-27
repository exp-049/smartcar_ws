#pragma once

#include <stddef.h>
#include <stdint.h>

#define LIDAR_WIRE_RAW_MAX 520
#define LIDAR_WIRE_DATAGRAM_MAX 1200

/* Compact UTF-8 JSON, no trailing NUL on the wire. CRC32 protects metadata
 * and the exact original UART packet. Return encoded length, or -1.
 */
int lidar_wire_encode(char *output, size_t capacity, const char *boot_id,
                       uint32_t sequence, int64_t rx_complete_us,
                       const uint8_t *raw_packet, size_t raw_length);
