#include "udp_ack.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#ifndef ESP_PLATFORM
#include <sys/socket.h>
#endif

int udp_wait_for_ack(int sock, const struct sockaddr_in *expected_peer,
                     const char *expected_ack, size_t expected_length,
                     int timeout_ms, int64_t (*now_us)(void), udp_ack_stats_t *stats)
{
    return udp_wait_for_ack_cancelable(sock, expected_peer, expected_ack, expected_length,
                                       timeout_ms, now_us, stats, NULL, NULL);
}

int udp_wait_for_ack_cancelable(int sock, const struct sockaddr_in *expected_peer,
                                const char *expected_ack, size_t expected_length,
                                int timeout_ms, int64_t (*now_us)(void), udp_ack_stats_t *stats,
                                udp_ack_cancel_fn_t cancel, void *cancel_context)
{
    if (sock < 0 || sock >= FD_SETSIZE || expected_peer == NULL ||
        expected_ack == NULL || timeout_ms < 0 || now_us == NULL || stats == NULL) {
        errno = EINVAL;
        return UDP_ACK_ERROR;
    }
    memset(stats, 0, sizeof(*stats));
    const int64_t started = now_us();
    const int64_t duration_us = (int64_t)timeout_ms * 1000;
    if (started < 0 || started > INT64_MAX - duration_us) {
        errno = EINVAL;
        return UDP_ACK_ERROR;
    }
    const int64_t deadline = started + duration_us;

    while (true) {
        if (cancel != NULL && cancel(cancel_context)) {
            return UDP_ACK_CANCELED;
        }
        const int64_t remaining = deadline - now_us();
        if (remaining <= 0) {
            return UDP_ACK_TIMEOUT;
        }
        const int64_t slice_us = remaining < 100000 ? remaining : 100000;
        struct timeval wait = {
            .tv_sec = slice_us / 1000000,
            .tv_usec = slice_us % 1000000,
        };
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(sock, &readable);
        int ready = select(sock + 1, &readable, NULL, NULL, &wait);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            return UDP_ACK_ERROR;
        }
        if (cancel != NULL && cancel(cancel_context)) {
            return UDP_ACK_CANCELED;
        }
        if (ready == 0) {
            continue;
        }

        char reply[128];
        struct sockaddr_in source = {0};
        socklen_t source_length = sizeof(source);
        int length = recvfrom(sock, reply, sizeof(reply), MSG_DONTWAIT,
                              (struct sockaddr *)&source, &source_length);
        if (length < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            return UDP_ACK_ERROR;
        }
        if (cancel != NULL && cancel(cancel_context)) {
            return UDP_ACK_CANCELED;
        }
        if (source.sin_family != AF_INET ||
            source.sin_addr.s_addr != expected_peer->sin_addr.s_addr ||
            source.sin_port != expected_peer->sin_port) {
            ++stats->wrong_source_packets;
            stats->last_wrong_source = source;
            continue;
        }
        if ((size_t)length == expected_length &&
            memcmp(reply, expected_ack, expected_length) == 0) {
            return UDP_ACK_MATCHED;
        }

        ++stats->unmatched_payload_packets;
        size_t copy_length = (size_t)length < sizeof(stats->last_unmatched_reply) - 1
            ? (size_t)length : sizeof(stats->last_unmatched_reply) - 1;
        memcpy(stats->last_unmatched_reply, reply, copy_length);
        stats->last_unmatched_reply[copy_length] = '\0';
    }
}
