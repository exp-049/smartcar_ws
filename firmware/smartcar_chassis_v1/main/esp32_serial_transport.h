#pragma once

#include <uxr/client/transport.h>

bool smartcar_serial_open(struct uxrCustomTransport *transport);
bool smartcar_serial_close(struct uxrCustomTransport *transport);
size_t smartcar_serial_write(struct uxrCustomTransport *transport,
                             const uint8_t *buffer, size_t length,
                             uint8_t *error_code);
size_t smartcar_serial_read(struct uxrCustomTransport *transport,
                            uint8_t *buffer, size_t length, int timeout_ms,
                            uint8_t *error_code);
