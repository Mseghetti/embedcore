#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Motor types
typedef enum {
    MOTOR_TYPE_DC = 0,
    MOTOR_TYPE_SERVO = 1,
    MOTOR_TYPE_STEPPER = 2
} motor_type_t;

// Motor direction
typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD = 1,
    MOTOR_DIR_REVERSE = 2
} motor_direction_t;

// Motor configuration
typedef struct {
    motor_type_t type;
    uint8_t pwm_channel;
    uint8_t direction_pin;
    uint8_t enable_pin;
    uint32_t max_speed;
    uint32_t min_speed;
    bool enabled;
} motor_config_t;

// Motor state
typedef struct {
    motor_direction_t direction;
    uint32_t speed;
    float position;  // For servo motors (0-180 degrees)
    bool is_moving;
} motor_state_t;

// Initialize motor system
void motor_init(void);

// Configure motor
bool motor_configure(uint8_t motor_id, const motor_config_t* config);

// Set motor speed (0-100%)
bool motor_set_speed(uint8_t motor_id, uint32_t speed);

// Set motor direction
bool motor_set_direction(uint8_t motor_id, motor_direction_t direction);

// Set servo motor position (0-180 degrees)
bool motor_set_position(uint8_t motor_id, float position);

// Stop motor
void motor_stop(uint8_t motor_id);

// Enable/disable motor
void motor_enable(uint8_t motor_id, bool enable);

// Get motor state
motor_state_t motor_get_state(uint8_t motor_id);

// Emergency stop all motors
void motor_emergency_stop(void);

#ifdef __cplusplus
}
#endif
