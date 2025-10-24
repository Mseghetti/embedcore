#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Timer callback function type
typedef void (*timer_callback_t)(void);

// Timer configuration
typedef struct {
    uint32_t period_ms;
    bool auto_reload;
    timer_callback_t callback;
    bool enabled;
} timer_config_t;

// Initialize timer system
void timer_init(void);

// Configure timer
bool timer_configure(uint8_t timer_id, const timer_config_t* config);

// Start timer
void timer_start(uint8_t timer_id);

// Stop timer
void timer_stop(uint8_t timer_id);

// Get current system time in milliseconds
uint32_t timer_get_ms(void);

// Get current system time in microseconds
uint64_t timer_get_us(void);

// Delay for specified milliseconds
void timer_delay_ms(uint32_t ms);

// Delay for specified microseconds
void timer_delay_us(uint32_t us);

// Check if timer is running
bool timer_is_running(uint8_t timer_id);

#ifdef __cplusplus
}
#endif
