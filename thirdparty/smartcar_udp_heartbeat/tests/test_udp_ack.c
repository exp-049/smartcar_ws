#include "udp_ack.h"
#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static int64_t monotonic_us(void)
{
    struct timespec value;
    assert(clock_gettime(CLOCK_MONOTONIC, &value) == 0);
    return (int64_t)value.tv_sec * 1000000 + value.tv_nsec / 1000;
}

static void sleep_ms(int milliseconds)
{
    struct timespec duration = {milliseconds / 1000, (milliseconds % 1000) * 1000000L};
    nanosleep(&duration, NULL);
}

static int bound_socket(struct sockaddr_in *address)
{
    int descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    assert(descriptor >= 0);
    memset(address, 0, sizeof(*address));
#ifdef __APPLE__
    address->sin_len = sizeof(*address);
#endif
    address->sin_family = AF_INET;
    address->sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    assert(bind(descriptor, (struct sockaddr *)address, sizeof(*address)) == 0);
    socklen_t length = sizeof(*address);
    assert(getsockname(descriptor, (struct sockaddr *)address, &length) == 0);
    return descriptor;
}

typedef struct {
    int server;
    int wrong_source;
    int scenario;
    struct sockaddr_in client;
} sender_context_t;

static const char *CURRENT = "ACK 0123456789abcdef 1";
static const char *OLD = "ACK 0123456789abcdef 0";

static void send_reply(int descriptor, const struct sockaddr_in *address, const char *message)
{
    size_t length = strlen(message);
    assert(sendto(descriptor, message, length, 0,
                  (const struct sockaddr *)address, sizeof(*address)) == (ssize_t)length);
}

static void *sender(void *argument)
{
    sender_context_t *context = argument;
    if (context->scenario == 0) {
        send_reply(context->server, &context->client, OLD);
        sleep_ms(20);
        send_reply(context->server, &context->client, CURRENT);
    } else if (context->scenario == 1) {
        sleep_ms(70);
        send_reply(context->server, &context->client, OLD);
        sleep_ms(20);
        send_reply(context->server, &context->client, CURRENT);
    } else if (context->scenario == 2) {
        send_reply(context->wrong_source, &context->client, CURRENT);
        send_reply(context->server, &context->client, "ACK fedcba9876543210 1");
        sleep_ms(20);
        send_reply(context->server, &context->client, CURRENT);
    } else if (context->scenario == 3) {
        for (int index = 0; index < 80; ++index) {
            send_reply(context->server, &context->client, OLD);
            sleep_ms(5);
        }
    } else if (context->scenario == 4) {
        sleep_ms(400);
        send_reply(context->server, &context->client, CURRENT);
    }
    return NULL;
}

typedef struct {
    int64_t cancel_at_us;
    unsigned calls;
} cancel_context_t;

static bool cancel_when_due(void *argument)
{
    cancel_context_t *context = argument;
    ++context->calls;
    return monotonic_us() >= context->cancel_at_us;
}

static int64_t maximum_clock(void)
{
    return INT64_MAX;
}

static void test_cancellation(void)
{
    struct sockaddr_in client_address, server_address, wrong_address;
    int client = bound_socket(&client_address);
    int server = bound_socket(&server_address);
    int wrong = bound_socket(&wrong_address);
    udp_ack_stats_t stats;
    int64_t started = monotonic_us();
    cancel_context_t context = {.cancel_at_us = started + 25000};
    int result = udp_wait_for_ack_cancelable(client, &server_address, CURRENT, strlen(CURRENT),
        800, monotonic_us, &stats, cancel_when_due, &context);
    int64_t elapsed = monotonic_us() - started;
    assert(result == UDP_ACK_CANCELED);
    assert(elapsed >= 25000 && elapsed < 250000);
    assert(context.calls >= 2);
    assert(stats.wrong_source_packets == 0 && stats.unmatched_payload_packets == 0);
    printf("PASS cancellation during quiet wait elapsed_ms=%.1f\n", elapsed / 1000.0);

    /* A canceled wait must not consume/accept a queued ACK for an obsolete
     * route. The unchanged compatibility API can still read that datagram. */
    send_reply(server, &client_address, CURRENT);
    context.cancel_at_us = 0;
    context.calls = 0;
    started = monotonic_us();
    assert(udp_wait_for_ack_cancelable(client, &server_address, CURRENT, strlen(CURRENT),
        800, monotonic_us, &stats, cancel_when_due, &context) == UDP_ACK_CANCELED);
    assert(monotonic_us() - started < 50000 && context.calls == 1);
    assert(udp_wait_for_ack(client, &server_address, CURRENT, strlen(CURRENT),
        100, monotonic_us, &stats) == UDP_ACK_MATCHED);

    /* Repeated select slices must retain the original absolute deadline. */
    context.cancel_at_us = INT64_MAX;
    context.calls = 0;
    started = monotonic_us();
    assert(udp_wait_for_ack_cancelable(client, &server_address, CURRENT, strlen(CURRENT),
        230, monotonic_us, &stats, cancel_when_due, &context) == UDP_ACK_TIMEOUT);
    elapsed = monotonic_us() - started;
    assert(elapsed >= 200000 && elapsed < 500000 && context.calls >= 3);
    printf("PASS cancelable fixed deadline elapsed_ms=%.1f\n", elapsed / 1000.0);

    sender_context_t noisy = {server, wrong, 3, client_address};
    pthread_t thread;
    assert(pthread_create(&thread, NULL, sender, &noisy) == 0);
    started = monotonic_us();
    context.cancel_at_us = started + 30000;
    context.calls = 0;
    assert(udp_wait_for_ack_cancelable(client, &server_address, CURRENT, strlen(CURRENT),
        800, monotonic_us, &stats, cancel_when_due, &context) == UDP_ACK_CANCELED);
    elapsed = monotonic_us() - started;
    assert(elapsed >= 30000 && elapsed < 200000 && stats.unmatched_payload_packets > 0);
    assert(pthread_join(thread, NULL) == 0);
    printf("PASS cancellation amid stale-ACK traffic elapsed_ms=%.1f\n", elapsed / 1000.0);
    assert(udp_wait_for_ack_cancelable(client, &server_address, CURRENT, strlen(CURRENT),
        1, maximum_clock, &stats, NULL, NULL) == UDP_ACK_ERROR && errno == EINVAL);
    close(client);
    close(server);
    close(wrong);
}

int main(void)
{
    for (int scenario = 0; scenario < 5; ++scenario) {
        struct sockaddr_in client_address, server_address, wrong_address;
        int client = bound_socket(&client_address);
        int server = bound_socket(&server_address);
        int wrong = bound_socket(&wrong_address);
        sender_context_t context = {server, wrong, scenario, client_address};
        pthread_t thread;
        assert(pthread_create(&thread, NULL, sender, &context) == 0);
        udp_ack_stats_t stats;
        if (scenario == 1) {
            assert(udp_wait_for_ack(client, &server_address, OLD, strlen(OLD),
                                    40, monotonic_us, &stats) == 0);
        }
        int timeout_ms = scenario == 3 ? 80 : 800;
        int64_t started = monotonic_us();
        int result = udp_wait_for_ack(client, &server_address, CURRENT, strlen(CURRENT),
                                      timeout_ms, monotonic_us, &stats);
        int64_t elapsed = monotonic_us() - started;
        if (scenario == 3) {
            assert(result == 0);
            assert(elapsed < 300000);
            assert(stats.unmatched_payload_packets > 0);
        } else {
            assert(result == 1);
            if (scenario <= 2) {
                assert(stats.unmatched_payload_packets == 1);
            }
            if (scenario == 2) {
                assert(stats.wrong_source_packets == 1);
            }
        }
        assert(pthread_join(thread, NULL) == 0);
        close(client);
        close(server);
        close(wrong);
        printf("PASS scenario=%d elapsed_ms=%.1f\n", scenario, elapsed / 1000.0);
    }
    test_cancellation();
    puts("PASS: legacy ACK compatibility/filtering/deadline; quiet/noisy cancellation; cancellation before queued match");
    return 0;
}
