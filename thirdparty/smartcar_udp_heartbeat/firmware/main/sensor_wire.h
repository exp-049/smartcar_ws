#pragma once

/* Portable encoders: no ESP-IDF headers, packed structs, or native endianness. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define IMU_WIRE_RAW_BYTES 14
#define IMU_WIRE_DATAGRAM_MAX 320
#define SENSOR_SYNC_REQUEST_BYTES 20
#define SENSOR_SYNC_RESPONSE_BYTES 44

bool sensor_wire_boot_id_value(const char *boot_id, uint64_t *value);

/* Raw registers are exactly 14 bytes: big-endian accel XYZ, temperature,
 * gyro XYZ from 0x3B. sample_us is the data-ready bracket midpoint, before
 * DLPF delay compensation, not a precise physical or UTC sample time.
 * Standard CRC32 covers explicit little-endian '<4sQIQIQH' metadata
 * ('SC1I', boot, seq, sample_us, uncertainty_us, read_end_us, 14) plus raw.
 * Return JSON byte count excluding NUL, or -1. No NUL is transmitted.
 */
int imu_wire_encode(char *output, size_t capacity, const char *boot_id,
                    uint32_t sequence, int64_t sample_us, uint32_t uncertainty_us,
                    int64_t read_end_us, const uint8_t *raw, size_t raw_length);

/* SCQ1 <nonce:uint64, host_t1_monotonic_ns:uint64>, little-endian.
 * Neither nonce nor the echoed host clock is interpreted as device time.
 * Source-IP and request/response correlation checks belong to the caller.
 */
bool sensor_wire_sync_request_decode(const uint8_t *request, size_t length,
                                     uint64_t *nonce, uint64_t *host_t1_ns);
/* SCR1 + nonce, echoed t1, device t2_us, device t3_us, boot:uint64 (LE).
 * Software receive/send timestamps do not represent physical Wi-Fi airtime.
 */
int sensor_wire_sync_response_encode(uint8_t *output, size_t capacity,
                                     uint64_t nonce, uint64_t host_t1_ns,
                                     int64_t device_t2_us, int64_t device_t3_us,
                                     uint64_t boot_value);
