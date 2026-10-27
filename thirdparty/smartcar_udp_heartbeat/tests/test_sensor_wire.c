#include "sensor_wire.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static const uint8_t raw[] = {
    0x01, 0x02, 0xFF, 0xFE, 0x40, 0x00, 0x00, 0x00,
    0x80, 0x00, 0x12, 0x34, 0xFE, 0xDC,
};
/* Independently generated with Python struct.pack('<4sQIQIQH', ...) and
 * zlib.crc32(metadata + raw), rather than the C encoder under test. */
static const char golden[] =
    "{\"type\":\"imu_packet\",\"version\":1,\"boot_id\":\"0123456789abcdef\",\"seq\":42,"
    "\"sample_us\":123456789,\"uncertainty_us\":1235,\"read_end_us\":123458999,"
    "\"data_hex\":\"0102fffe4000000080001234fedc\",\"crc32\":\"dfdfa90e\"}";

static void test_imu(void)
{
    char output[IMU_WIRE_DATAGRAM_MAX + 2];
    int length = imu_wire_encode(output, sizeof(output), "0123456789abcdef", 42,
                                 123456789, 1235, 123458999, raw, sizeof(raw));
    assert(length == (int)strlen(golden));
    assert(strcmp(output, golden) == 0);
    assert(imu_wire_encode(output, (size_t)length + 1, "0123456789abcdef", 42,
                           123456789, 1235, 123458999, raw, sizeof(raw)) == length);
    memset(output, 0x5A, sizeof(output));
    assert(imu_wire_encode(output, (size_t)length, "0123456789abcdef", 42,
                           123456789, 1235, 123458999, raw, sizeof(raw)) == -1);
    assert(output[length] == 0x5A);
    assert(imu_wire_encode(output, 1, "0123456789abcdef", 42,
                           123456789, 1235, 123458999, raw, sizeof(raw)) == -1);
    assert(imu_wire_encode(NULL, sizeof(output), "0123456789abcdef", 0,
                           0, 0, 0, raw, sizeof(raw)) == -1);
    assert(imu_wire_encode(output, sizeof(output), "0123456789ABCDEF", 0,
                           0, 0, 0, raw, sizeof(raw)) == -1);
    assert(imu_wire_encode(output, sizeof(output), "0123456789abcdef", 0,
                           -1, 0, 0, raw, sizeof(raw)) == -1);
    assert(imu_wire_encode(output, sizeof(output), "0123456789abcdef", 0,
                           10, 0, 9, raw, sizeof(raw)) == -1);
    assert(imu_wire_encode(output, sizeof(output), "0123456789abcdef", 0,
                           0, 0, 0, raw, sizeof(raw) - 1) == -1);
    assert(imu_wire_encode(output, sizeof(output), "0123456789abcdef", 0,
                           0, 0, 0, NULL, sizeof(raw)) == -1);
    length = imu_wire_encode(output, sizeof(output), "ffffffffffffffff", UINT32_MAX,
                             INT64_MAX, UINT32_MAX, INT64_MAX, raw, sizeof(raw));
    assert(length > 0 && length <= IMU_WIRE_DATAGRAM_MAX);
    assert((size_t)length == strlen(output));
    /* All protected metadata fields and all raw bytes must affect CRC. */
    const char *boot_ids[] = {"0123456789abcdee", "0123456789abcdef"};
    for (unsigned field = 0; field < 6; ++field) {
        uint8_t changed[sizeof(raw)];
        memcpy(changed, raw, sizeof(changed));
        if (field == 5) {
            changed[13] ^= 1;
        }
        assert(imu_wire_encode(output, sizeof(output), boot_ids[field == 0 ? 0 : 1],
                               42 + (field == 1), 123456789 + (field == 2),
                               1235 + (field == 3), 123458999 + (field == 4),
                               changed, sizeof(changed)) > 0);
        assert(strstr(output, "\"crc32\":\"dfdfa90e\"") == NULL);
    }
    for (size_t index = 0; index < sizeof(raw); ++index) {
        uint8_t changed[sizeof(raw)];
        memcpy(changed, raw, sizeof(changed));
        changed[index] ^= 1;
        assert(imu_wire_encode(output, sizeof(output), "0123456789abcdef", 42,
                               123456789, 1235, 123458999, changed, sizeof(changed)) > 0);
        assert(strstr(output, "\"crc32\":\"dfdfa90e\"") == NULL);
    }
}

static void test_sync(void)
{
    uint64_t boot_value = 0;
    assert(sensor_wire_boot_id_value("0123456789abcdef", &boot_value));
    assert(boot_value == UINT64_C(0x0123456789ABCDEF));
    assert(!sensor_wire_boot_id_value("0123456789abcdeg", &boot_value));
    assert(!sensor_wire_boot_id_value("0123456789ABCDEF", &boot_value));
    assert(!sensor_wire_boot_id_value("0123456789abcde", &boot_value));
    assert(!sensor_wire_boot_id_value(NULL, &boot_value));
    assert(!sensor_wire_boot_id_value("0123456789abcdef", NULL));
    const uint8_t request[] = {
        'S', 'C', 'Q', '1', 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
    };
    uint64_t nonce = 0, host_t1 = 0;
    assert(sensor_wire_sync_request_decode(request, sizeof(request), &nonce, &host_t1));
    assert(nonce == UINT64_C(0x1122334455667788));
    assert(host_t1 == UINT64_C(0x0102030405060708));
    for (size_t length = 0; length < sizeof(request); ++length) {
        assert(!sensor_wire_sync_request_decode(request, length, &nonce, &host_t1));
    }
    assert(!sensor_wire_sync_request_decode(request, sizeof(request) + 1, &nonce, &host_t1));
    uint8_t changed[sizeof(request)];
    memcpy(changed, request, sizeof(changed));
    changed[3] = '0';
    assert(!sensor_wire_sync_request_decode(changed, sizeof(changed), &nonce, &host_t1));
    assert(!sensor_wire_sync_request_decode(NULL, sizeof(request), &nonce, &host_t1));
    assert(!sensor_wire_sync_request_decode(request, sizeof(request), NULL, &host_t1));
    uint8_t response[SENSOR_SYNC_RESPONSE_BYTES + 1];
    memset(response, 0x5A, sizeof(response));
    const uint8_t expected[] = {
        0x53, 0x43, 0x52, 0x31, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
        0x15, 0xCD, 0x5B, 0x07, 0, 0, 0, 0,
        0xE7, 0xCD, 0x5B, 0x07, 0, 0, 0, 0,
        0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,
    };
    assert(sizeof(expected) == SENSOR_SYNC_RESPONSE_BYTES);
    assert(sensor_wire_sync_response_encode(response, SENSOR_SYNC_RESPONSE_BYTES, nonce, host_t1,
                                            123456789, 123456999, boot_value) == SENSOR_SYNC_RESPONSE_BYTES);
    assert(memcmp(response, expected, sizeof(expected)) == 0);
    assert(response[SENSOR_SYNC_RESPONSE_BYTES] == 0x5A);
    assert(sensor_wire_sync_response_encode(response, SENSOR_SYNC_RESPONSE_BYTES - 1, nonce, host_t1,
                                            0, 0, 0) == -1);
    assert(sensor_wire_sync_response_encode(response, sizeof(response), nonce, host_t1, -1, 0, 0) == -1);
    assert(sensor_wire_sync_response_encode(response, sizeof(response), nonce, host_t1, 2, 1, 0) == -1);
    assert(sensor_wire_sync_response_encode(NULL, sizeof(response), nonce, host_t1, 0, 0, 0) == -1);
    assert(sensor_wire_sync_response_encode(response, sizeof(response), UINT64_MAX, UINT64_MAX,
                                            INT64_MAX, INT64_MAX, UINT64_MAX) == SENSOR_SYNC_RESPONSE_BYTES);
}

int main(int argc, char **argv)
{
    test_imu();
    test_sync();
    if (argc == 2 && strcmp(argv[1], "--emit") == 0) {
        char output[IMU_WIRE_DATAGRAM_MAX + 1];
        int length = imu_wire_encode(output, sizeof(output), "0123456789abcdef", 42,
                                     123456789, 1235, 123458999, raw, sizeof(raw));
        assert(length > 0);
        puts(output);
    } else {
        puts("PASS: IMU independent CRC vector and protected fields; sync LE vectors; malformed inputs; exact bounds");
    }
    return 0;
}
