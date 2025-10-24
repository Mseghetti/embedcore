#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// PWM channel configuration
typedef struct {
    uint8_t channel;
    uint32_t frequency_hz;
    uint8_t duty_cycle_percent;
    bool enabled;
} pwm_config_t;

// Initialize PWM system
void pwm_init(void);

// Configure PWM channel
bool pwm_configure(uint8_t channel, uint32_t frequency_hz);

// Set PWM duty cycle (0-100%)
bool pwm_set_duty_cycle(uint8_t channel, uint8_t duty_cycle_percent);

// Enable/disable PWM channel
void pwm_enable(uint8_t channel, bool enable);

// Get current PWM configuration
pwm_config_t pwm_get_config(uint8_t channel);

// Convert angle (0-180 degrees) to servo duty cycle
uint8_t pwm_angle_to_duty_cycle(float angle_degrees);

#ifdef __cplusplus
}
#endif
