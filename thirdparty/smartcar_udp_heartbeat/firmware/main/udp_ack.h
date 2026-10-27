#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef ESP_PLATFORM
#include "lwip/sockets.h"
#else
#include <netinet/in.h>
#endif

typedef struct {
    unsigned wrong_source_packets;
    unsigned unmatched_payload_packets;
    struct sockaddr_in last_wrong_source;
    char last_unmatched_reply[64];
} udp_ack_stats_t;

#define UDP_ACK_MATCHED 1
#define UDP_ACK_TIMEOUT 0
#define UDP_ACK_ERROR (-1)
#define UDP_ACK_CANCELED (-2)

typedef bool (*udp_ack_cancel_fn_t)(void *context);

/* Return 1 for a matching reply, 0 for deadline expiry, or -1 for a socket
 * error (errno is preserved). Unexpected packets never extend the deadline.
 * now_us must return monotonic microseconds, such as esp_timer_get_time().
 */
int udp_wait_for_ack(int sock, const struct sockaddr_in *expected_peer,
                     const char *expected_ack, size_t expected_length,
                     int timeout_ms, int64_t (*now_us)(void), udp_ack_stats_t *stats);

/* As above, with an optional cheap cancellation predicate. A true predicate
 * returns UDP_ACK_CANCELED, distinct from socket errors. The predicate is
 * checked before waiting and before accepting a received reply. Each select
 * sleeps at most 100 ms; slices and unexpected traffic never reset the single
 * absolute deadline. NULL cancel preserves the original API's behaviour.
 */
int udp_wait_for_ack_cancelable(int sock, const struct sockaddr_in *expected_peer,
                                const char *expected_ack, size_t expected_length,
                                int timeout_ms, int64_t (*now_us)(void), udp_ack_stats_t *stats,
                                udp_ack_cancel_fn_t cancel, void *cancel_context);
