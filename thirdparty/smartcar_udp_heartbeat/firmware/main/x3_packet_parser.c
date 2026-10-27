#include "x3_packet_parser.h"

#include <stdbool.h>
#include <string.h>

static uint16_t little_word(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
}

static void consume(x3_packet_parser_t *parser, size_t count, bool discarded)
{
    if (discarded) {
        parser->discarded_bytes += count;
    }
    parser->used -= count;
    memmove(parser->buffer, parser->buffer + count, parser->used);
}

static void parse_available(x3_packet_parser_t *parser)
{
    while (parser->used >= 2) {
        if (parser->buffer[0] != 0xAA || parser->buffer[1] != 0x55) {
            size_t offset = 1;
            while (offset + 1 < parser->used &&
                   !(parser->buffer[offset] == 0xAA && parser->buffer[offset + 1] == 0x55)) {
                ++offset;
            }
            if (offset + 1 >= parser->used) {
                offset = parser->used - (parser->buffer[parser->used - 1] == 0xAA ? 1 : 0);
            }
            consume(parser, offset, true);
            continue;
        }
        if (parser->used < 4) {
            return;
        }
        const uint8_t point_count = parser->buffer[3];
        if (point_count == 0) {
            ++parser->header_errors;
            consume(parser, 1, true);
            continue;
        }
        if (parser->used < 10) {
            return;
        }
        const uint16_t first_angle = little_word(parser->buffer + 4);
        const uint16_t last_angle = little_word(parser->buffer + 6);
        if (!(first_angle & 1) || !(last_angle & 1) ||
            (first_angle >> 1) > 360 * 64 || (last_angle >> 1) > 360 * 64) {
            ++parser->header_errors;
            consume(parser, 1, true);
            continue;
        }
        const size_t packet_length = 10 + 2 * (size_t)point_count;
        if (parser->used < packet_length) {
            return;
        }
        uint16_t checksum = 0x55AA ^ little_word(parser->buffer + 2) ^ first_angle ^ last_angle;
        for (size_t offset = 10; offset < packet_length; offset += 2) {
            checksum ^= little_word(parser->buffer + offset);
        }
        if (checksum != little_word(parser->buffer + 8)) {
            ++parser->checksum_errors;
            consume(parser, 1, true);
            continue;
        }
        ++parser->valid_packets;
        parser->sample_words += point_count;
        parser->start_markers += (parser->buffer[2] & 1) != 0;
        parser->last_ct = parser->buffer[2];
        parser->last_sample_count = point_count;
        parser->last_first_angle_raw = first_angle;
        parser->last_last_angle_raw = last_angle;
        parser->last_first_distance_raw = little_word(parser->buffer + 10);
        parser->last_checksum = checksum;
        if (parser->packet_handler != NULL) {
            parser->packet_handler(parser->buffer, packet_length, parser->packet_handler_context);
        }
        consume(parser, packet_length, false);
    }
}

void x3_packet_parser_init(x3_packet_parser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
}

void x3_packet_parser_feed(x3_packet_parser_t *parser, const uint8_t *bytes, size_t length)
{
    parser->bytes += length;
    while (length > 0) {
        if (parser->used == sizeof(parser->buffer)) {
            /* A complete maximum-sized packet must have been consumed or
             * rejected already. Keep this fallback bounded on malformed input.
             */
            ++parser->header_errors;
            consume(parser, 1, true);
        }
        size_t available = sizeof(parser->buffer) - parser->used;
        size_t copied = length < available ? length : available;
        memcpy(parser->buffer + parser->used, bytes, copied);
        parser->used += copied;
        bytes += copied;
        length -= copied;
        parse_available(parser);
    }
}

void x3_packet_parser_drop_partial(x3_packet_parser_t *parser)
{
    if (parser->used > 0) {
        ++parser->partial_resets;
        parser->discarded_bytes += parser->used;
        parser->used = 0;
    }
}

void x3_packet_parser_set_handler(x3_packet_parser_t *parser, x3_packet_handler_t handler,
                                  void *context)
{
    parser->packet_handler = handler;
    parser->packet_handler_context = context;
}
