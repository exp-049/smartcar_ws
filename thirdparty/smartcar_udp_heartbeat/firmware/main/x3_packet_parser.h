#pragma once

#include <stddef.h>
#include <stdint.h>

/* Standard non-intensity YDLIDAR triangle packet: AA 55, CT, count,
 * FSA/LSA/checksum little-endian words, then count distance words.
 * The byte count bounds memory; actual X3PRO packet sizes are diagnosed.
 */
#define X3_MAX_PACKET_BYTES (10 + 2 * 255)

typedef void (*x3_packet_handler_t)(const uint8_t *raw_packet, size_t length, void *context);

typedef struct {
    uint8_t buffer[X3_MAX_PACKET_BYTES];
    size_t used;
    uint64_t bytes;
    uint64_t valid_packets;
    uint64_t sample_words;
    uint64_t start_markers;
    uint64_t checksum_errors;
    uint64_t header_errors;
    uint64_t discarded_bytes;
    uint64_t partial_resets;
    uint8_t last_ct;
    uint8_t last_sample_count;
    uint16_t last_first_angle_raw;
    uint16_t last_last_angle_raw;
    uint16_t last_first_distance_raw;
    uint16_t last_checksum;
    x3_packet_handler_t packet_handler;
    void *packet_handler_context;
} x3_packet_parser_t;

void x3_packet_parser_init(x3_packet_parser_t *parser);
void x3_packet_parser_feed(x3_packet_parser_t *parser, const uint8_t *bytes, size_t length);
void x3_packet_parser_drop_partial(x3_packet_parser_t *parser);
/* Handler must copy data before returning; the parser reuses its buffer. */
void x3_packet_parser_set_handler(x3_packet_parser_t *parser, x3_packet_handler_t handler,
                                  void *context);
