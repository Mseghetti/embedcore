#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// GPIO pin modes
typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_PULLUP = 2,
    GPIO_MODE_PULLDOWN = 3
} gpio_mode_t;

// GPIO pin states
typedef enum {
    GPIO_LOW = 0,
    GPIO_HIGH = 1
} gpio_state_t;

// Initialize GPIO system
void gpio_init(void);

// Configure GPIO pin mode
void gpio_set_mode(uint8_t pin, gpio_mode_t mode);

// Read GPIO pin state
gpio_state_t gpio_read(uint8_t pin);

// Write GPIO pin state
void gpio_write(uint8_t pin, gpio_state_t state);

// Toggle GPIO pin state
void gpio_toggle(uint8_t pin);

// Simulated delay for realistic timing
void gpio_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif
