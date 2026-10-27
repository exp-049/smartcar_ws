#pragma once

/* Passive receive-only acquisition. Does not drive lidar power, motor PWM,
 * or a UART TX pin. This diagnostic stage leaves the UDP heartbeat unchanged.
 */
void lidar_uart_start(void);
