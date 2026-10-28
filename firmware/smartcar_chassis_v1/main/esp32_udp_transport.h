#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/transport.h>

typedef struct {
    int socket_fd;
    uint32_t route_generation;
} smartcar_udp_transport_t;

bool smartcar_udp_route_current(const smartcar_udp_transport_t *transport);
bool smartcar_udp_open(struct uxrCustomTransport *transport);
bool smartcar_udp_close(struct uxrCustomTransport *transport);
size_t smartcar_udp_write(struct uxrCustomTransport *transport, const uint8_t *buffer,
                          size_t length, uint8_t *error_code);
size_t smartcar_udp_read(struct uxrCustomTransport *transport, uint8_t *buffer,
                         size_t length, int timeout_ms, uint8_t *error_code);
