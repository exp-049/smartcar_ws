#include "x3_packet_parser.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Independently calculated packet: CT=A1, N=3, angles 0/90 degrees,
 * distance words 4000/8000/0, checksum=6BEB.
 */
static const uint8_t golden[] = {
    0xAA, 0x55, 0xA1, 0x03, 0x01, 0x00, 0x01, 0x2D,
    0xEB, 0x6B, 0xA0, 0x0F, 0x40, 0x1F, 0x00, 0x00,
};

typedef struct {
    unsigned count;
    uint8_t copy[sizeof(golden)];
} callback_capture_t;

static void capture_packet(const uint8_t *packet, size_t length, void *context)
{
    callback_capture_t *capture = context;
    assert(length == sizeof(golden));
    assert(memcmp(packet, golden, sizeof(golden)) == 0);
    memcpy(capture->copy, packet, length);
    ++capture->count;
}

int main(void)
{
    x3_packet_parser_t parser;
    x3_packet_parser_init(&parser);
    for (size_t index = 0; index < sizeof(golden); ++index) {
        x3_packet_parser_feed(&parser, golden + index, 1);
    }
    assert(parser.valid_packets == 1 && parser.sample_words == 3);
    assert(parser.start_markers == 1 && parser.last_ct == 0xA1);
    assert(parser.last_checksum == 0x6BEB && parser.last_first_distance_raw == 4000);
    assert(parser.checksum_errors == 0 && parser.used == 0);
    puts("PASS: fragmented golden packet, full CT checksum, multi-point start marker");

    x3_packet_parser_init(&parser);
    const uint8_t noise[] = {0, 0xAA, 0, 0x55, 0xFF};
    x3_packet_parser_feed(&parser, noise, sizeof(noise));
    uint8_t pair[2 * sizeof(golden)];
    memcpy(pair, golden, sizeof(golden));
    memcpy(pair + sizeof(golden), golden, sizeof(golden));
    x3_packet_parser_feed(&parser, pair, sizeof(pair));
    assert(parser.valid_packets == 2 && parser.sample_words == 6);
    assert(parser.discarded_bytes == sizeof(noise));
    puts("PASS: leading noise and concatenated packets");

    x3_packet_parser_init(&parser);
    callback_capture_t capture = {0};
    x3_packet_parser_set_handler(&parser, capture_packet, &capture);
    x3_packet_parser_feed(&parser, pair, sizeof(pair));
    assert(capture.count == 2 && memcmp(capture.copy, golden, sizeof(golden)) == 0);
    puts("PASS: validated callback receives complete packet before buffer reuse");

    x3_packet_parser_init(&parser);
    uint8_t damaged[sizeof(golden)];
    memcpy(damaged, golden, sizeof(golden));
    damaged[8] ^= 1;
    x3_packet_parser_feed(&parser, damaged, sizeof(damaged));
    x3_packet_parser_feed(&parser, golden, sizeof(golden));
    assert(parser.checksum_errors == 1 && parser.valid_packets == 1);
    puts("PASS: checksum failure followed by automatic resynchronization");

    x3_packet_parser_init(&parser);
    const uint8_t zero_count[] = {0xAA, 0x55, 0, 0};
    x3_packet_parser_feed(&parser, zero_count, sizeof(zero_count));
    memcpy(damaged, golden, sizeof(golden));
    damaged[4] &= 0xFE;
    x3_packet_parser_feed(&parser, damaged, sizeof(damaged));
    x3_packet_parser_feed(&parser, golden, sizeof(golden));
    assert(parser.header_errors == 2 && parser.valid_packets == 1);
    puts("PASS: invalid count and angle flags rejected");

    x3_packet_parser_init(&parser);
    x3_packet_parser_feed(&parser, golden, 9);
    assert(parser.valid_packets == 0 && parser.used == 9);
    x3_packet_parser_drop_partial(&parser);
    x3_packet_parser_feed(&parser, golden + 9, sizeof(golden) - 9);
    x3_packet_parser_feed(&parser, golden, sizeof(golden));
    assert(parser.partial_resets == 1 && parser.valid_packets == 1);
    puts("PASS: truncated packet discarded before stream recovery");

    x3_packet_parser_init(&parser);
    uint8_t maximum[X3_MAX_PACKET_BYTES] = {0xAA, 0x55, 0, 255, 1, 0, 1, 0};
    uint16_t checksum = 0x55AA ^ 0xFF00 ^ 1 ^ 1;
    for (size_t index = 0; index < 255; ++index) {
        maximum[10 + index * 2] = 0xA0;
        maximum[11 + index * 2] = 0x0F;
        checksum ^= 4000;
    }
    maximum[8] = checksum & 0xFF;
    maximum[9] = checksum >> 8;
    for (size_t index = 0; index < sizeof(maximum); ++index) {
        x3_packet_parser_feed(&parser, maximum + index, 1);
    }
    assert(parser.valid_packets == 1 && parser.sample_words == 255 && parser.used == 0);
    puts("PASS: maximum bounded packet length");

    uint32_t random_state = 0x12345678;
    uint8_t fuzz[4096];
    for (size_t index = 0; index < sizeof(fuzz); ++index) {
        random_state = 1664525 * random_state + 1013904223;
        fuzz[index] = random_state >> 24;
    }
    x3_packet_parser_init(&parser);
    x3_packet_parser_feed(&parser, fuzz, sizeof(fuzz));
    for (int index = 0; index < 80; ++index) {
        x3_packet_parser_feed(&parser, golden, sizeof(golden));
    }
    assert(parser.valid_packets > 0 && parser.used <= sizeof(parser.buffer));
    assert(parser.last_checksum == 0x6BEB);
    puts("PASS: bounded random stream and eventual recovery");
    return 0;
}
