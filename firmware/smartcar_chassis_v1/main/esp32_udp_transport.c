#include "esp32_udp_transport.h"

#include <sys/select.h>
#include <unistd.h>
#include "host_route.h"
#include "lwip/sockets.h"

bool smartcar_udp_route_current(const smartcar_udp_transport_t *transport)
{
    uint32_t generation = 0;
    return transport && transport->socket_fd >= 0 &&
        host_route_get(NULL, &generation) && generation == transport->route_generation;
}

bool smartcar_udp_open(struct uxrCustomTransport *transport)
{
    smartcar_udp_transport_t *context = transport->args;
    struct sockaddr_in destination;
    uint32_t generation;
    if (!context || !host_route_get(&destination, &generation)) return false;
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) return false;
    if (connect(fd, (const struct sockaddr *)&destination, sizeof(destination)) != 0) {
        close(fd);
        return false;
    }
    context->socket_fd = fd;
    context->route_generation = generation;
    return true;
}

bool smartcar_udp_close(struct uxrCustomTransport *transport)
{
    smartcar_udp_transport_t *context = transport->args;
    if (!context) return false;
    if (context->socket_fd >= 0) close(context->socket_fd);
    context->socket_fd = -1;
    return true;
}

size_t smartcar_udp_write(struct uxrCustomTransport *transport, const uint8_t *buffer,
                          size_t length, uint8_t *error_code)
{
    smartcar_udp_transport_t *context = transport->args;
    if (!smartcar_udp_route_current(context)) {
        if (error_code) *error_code = 1;
        return 0;
    }
    int sent = send(context->socket_fd, buffer, length, 0);
    if (error_code) *error_code = sent < 0 || (size_t)sent != length;
    return sent < 0 ? 0 : (size_t)sent;
}

size_t smartcar_udp_read(struct uxrCustomTransport *transport, uint8_t *buffer,
                         size_t length, int timeout_ms, uint8_t *error_code)
{
    smartcar_udp_transport_t *context = transport->args;
    if (!smartcar_udp_route_current(context)) {
        if (error_code) *error_code = 1;
        return 0;
    }
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(context->socket_fd, &readfds);
    if (timeout_ms < 0) timeout_ms = 0;
    struct timeval timeout = {.tv_sec = timeout_ms / 1000,
                              .tv_usec = (timeout_ms % 1000) * 1000};
    int ready = select(context->socket_fd + 1, &readfds, NULL, NULL, &timeout);
    if (ready <= 0) {
        if (error_code) *error_code = ready < 0;
        return 0;
    }
    int count = recv(context->socket_fd, buffer, length, 0);
    if (error_code) *error_code = count < 0;
    return count < 0 ? 0 : (size_t)count;
}
