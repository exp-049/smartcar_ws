#include "lidar_wire.h"

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static const uint8_t golden[] = {
    0xAA, 0x55, 0xA1, 0x03, 0x01, 0x00, 0x01, 0x2D,
    0xEB, 0x6B, 0xA0, 0x0F, 0x40, 0x1F, 0x00, 0x00,
};

static void maximum_packet(uint8_t *raw)
{
    memset(raw, 0, LIDAR_WIRE_RAW_MAX);
    raw[0] = 0xAA;
    raw[1] = 0x55;
    raw[2] = 0xA1;
    raw[3] = 255;
    raw[4] = 1;
    raw[6] = 1;
    raw[7] = 0x2D;
    uint16_t checksum = 0x55AA ^ 0xFFA1 ^ 1 ^ 0x2D01;
    for (size_t index = 0; index < 255; ++index) {
        raw[10 + index * 2] = 0xA0;
        raw[11 + index * 2] = 0x0F;
        checksum ^= 4000;
    }
    raw[8] = checksum & 0xFF;
    raw[9] = checksum >> 8;
}

int main(int argc, char **argv)
{
    char output[LIDAR_WIRE_DATAGRAM_MAX + 1];
    uint8_t raw[LIDAR_WIRE_RAW_MAX];
    maximum_packet(raw);
    int length = lidar_wire_encode(output, sizeof(output), "0123456789abcdef", 42,
                                    123456789, golden, sizeof(golden));
    const char *expected = "{\"type\":\"lidar_packet\",\"version\":1,\"boot_id\":\"0123456789abcdef\","
        "\"seq\":42,\"rx_complete_us\":123456789,\"data_hex\":\"aa55a1030100012deb6ba00f401f0000\","
        "\"crc32\":\"d223eed5\"}";
    assert(length == (int)strlen(expected));
    assert(strcmp(output, expected) == 0);
    int emit = argc == 2 && strcmp(argv[1], "--emit") == 0;
    if (emit) {
        puts(output);
    }

    length = lidar_wire_encode(output, sizeof(output), "ffffffffffffffff", UINT32_MAX,
                               INT64_MAX, raw, sizeof(raw));
    assert(length == 1191);
    assert((size_t)length == strlen(output));
    assert(output[length] == '\0');
    if (emit) {
        puts(output);
        return 0;
    }
    assert(lidar_wire_encode(output, 1192, "ffffffffffffffff", UINT32_MAX,
                             INT64_MAX, raw, sizeof(raw)) == 1191);
    assert(lidar_wire_encode(output, 1191, "ffffffffffffffff", UINT32_MAX,
                             INT64_MAX, raw, sizeof(raw)) == -1);
    assert(lidar_wire_encode(output, 1, "0123456789abcdef", 0, 0, golden, sizeof(golden)) == -1);
    assert(lidar_wire_encode(output, sizeof(output), "G123456789abcdef", 0, 0, golden, sizeof(golden)) == -1);
    assert(lidar_wire_encode(output, sizeof(output), "0123456789abcdef", 0, -1, golden, sizeof(golden)) == -1);
    assert(lidar_wire_encode(output, sizeof(output), "0123456789abcdef", 0, 0, raw, sizeof(raw) + 1) == -1);
    puts("PASS: independent CRC vector, max 1191-byte datagram, exact buffer bounds, invalid arguments");
    return 0;
}
