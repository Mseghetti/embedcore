#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// UART configuration
typedef struct {
    uint32_t baud_rate;
    uint8_t data_bits;
    uint8_t stop_bits;
    bool parity_enabled;
    bool parity_odd;
} uart_config_t;

// Initialize UART system
void uart_init(void);

// Configure UART port
bool uart_configure(uint8_t port, const uart_config_t* config);

// Send data via UART
bool uart_send(uint8_t port, const uint8_t* data, uint16_t length);

// Receive data via UART
uint16_t uart_receive(uint8_t port, uint8_t* buffer, uint16_t max_length);

// Check if data is available
bool uart_data_available(uint8_t port);

// Send string via UART (null-terminated)
bool uart_send_string(uint8_t port, const char* str);

// Flush UART buffers
void uart_flush(uint8_t port);

#ifdef __cplusplus
}
#endif
