#include "motor.h"
#include "pwm.h"
#include "gpio.h"
#include <iostream>
#include <map>
#include <algorithm>
#include <cstring>

// Simulated motor state
static std::map<uint8_t, motor_config_t> motor_configs;
static std::map<uint8_t, motor_state_t> motor_states;
static bool motor_initialized = false;

void motor_init(void) {
    if (motor_initialized) return;
    
    std::cout << "[MOTOR] Initializing motor system..." << std::endl;
    pwm_init();
    gpio_init();
    motor_initialized = true;
}

bool motor_configure(uint8_t motor_id, const motor_config_t* config) {
    if (!motor_initialized) motor_init();
    
    if (!config) {
        std::cout << "[MOTOR] Invalid configuration" << std::endl;
        return false;
    }
    
    motor_configs[motor_id] = *config;
    
    // Initialize motor state
    motor_state_t state;
    state.direction = MOTOR_DIR_STOP;
    state.speed = 0;
    state.position = 0.0f;
    state.is_moving = false;
    motor_states[motor_id] = state;
    
    // Configure PWM for servo motors
    if (config->type == MOTOR_TYPE_SERVO) {
        pwm_configure(config->pwm_channel, 50); // 50Hz for servos
    }
    
    const char* type_str = (config->type == MOTOR_TYPE_DC) ? "DC" :
                          (config->type == MOTOR_TYPE_SERVO) ? "SERVO" : "STEPPER";
    
    std::cout << "[MOTOR] Motor " << (int)motor_id << " configured: " << type_str 
              << ", PWM ch=" << (int)config->pwm_channel << std::endl;
    
    return true;
}

bool motor_set_speed(uint8_t motor_id, uint32_t speed) {
    if (!motor_initialized) motor_init();
    
    auto config_it = motor_configs.find(motor_id);
    if (config_it == motor_configs.end()) {
        std::cout << "[MOTOR] Motor " << (int)motor_id << " not configured" << std::endl;
        return false;
    }
    
    // Clamp speed to valid range
    speed = std::min(speed, config_it->second.max_speed);
    speed = std::max(speed, config_it->second.min_speed);
    
    auto& state = motor_states[motor_id];
    state.speed = speed;
    state.is_moving = (speed > 0 && state.direction != MOTOR_DIR_STOP);
    
    // For DC motors, set PWM duty cycle based on speed
    if (config_it->second.type == MOTOR_TYPE_DC) {
        uint8_t duty_cycle = (speed * 100) / config_it->second.max_speed;
        pwm_set_duty_cycle(config_it->second.pwm_channel, duty_cycle);
    }
    
    std::cout << "[MOTOR] Motor " << (int)motor_id << " speed: " << speed << std::endl;
    return true;
}

bool motor_set_direction(uint8_t motor_id, motor_direction_t direction) {
    if (!motor_initialized) motor_init();
    
    auto config_it = motor_configs.find(motor_id);
    if (config_it == motor_configs.end()) {
        std::cout << "[MOTOR] Motor " << (int)motor_id << " not configured" << std::endl;
        return false;
    }
    
    auto& state = motor_states[motor_id];
    state.direction = direction;
    state.is_moving = (state.speed > 0 && direction != MOTOR_DIR_STOP);
    
    // Set direction pin for DC motors
    if (config_it->second.type == MOTOR_TYPE_DC) {
        gpio_state_t dir_pin_state = (direction == MOTOR_DIR_FORWARD) ? GPIO_HIGH : GPIO_LOW;
        gpio_write(config_it->second.direction_pin, dir_pin_state);
    }
    
    const char* dir_str = (direction == MOTOR_DIR_FORWARD) ? "FORWARD" :
                         (direction == MOTOR_DIR_REVERSE) ? "REVERSE" : "STOP";
    
    std::cout << "[MOTOR] Motor " << (int)motor_id << " direction: " << dir_str << std::endl;
    return true;
}

bool motor_set_position(uint8_t motor_id, float position) {
    if (!motor_initialized) motor_init();
    
    auto config_it = motor_configs.find(motor_id);
    if (config_it == motor_configs.end()) {
        std::cout << "[MOTOR] Motor " << (int)motor_id << " not configured" << std::endl;
        return false;
    }
    
    if (config_it->second.type != MOTOR_TYPE_SERVO) {
        std::cout << "[MOTOR] Motor " << (int)motor_id << " is not a servo motor" << std::endl;
        return false;
    }
    
    // Clamp position to 0-180 degrees
    position = std::max(0.0f, std::min(180.0f, position));
    
    auto& state = motor_states[motor_id];
    state.position = position;
    state.is_moving = true;
    
    // Convert angle to PWM duty cycle
    uint8_t duty_cycle = pwm_angle_to_duty_cycle(position);
    pwm_set_duty_cycle(config_it->second.pwm_channel, duty_cycle);
    pwm_enable(config_it->second.pwm_channel, true);
    
    std::cout << "[MOTOR] Servo " << (int)motor_id << " position: " << position << "°" << std::endl;
    return true;
}

void motor_stop(uint8_t motor_id) {
    if (!motor_initialized) motor_init();
    
    auto config_it = motor_configs.find(motor_id);
    if (config_it == motor_configs.end()) {
        return;
    }
    
    auto& state = motor_states[motor_id];
    state.direction = MOTOR_DIR_STOP;
    state.speed = 0;
    state.is_moving = false;
    
    // Stop PWM output
    pwm_enable(config_it->second.pwm_channel, false);
    
    std::cout << "[MOTOR] Motor " << (int)motor_id << " stopped" << std::endl;
}

void motor_enable(uint8_t motor_id, bool enable) {
    if (!motor_initialized) motor_init();
    
    auto config_it = motor_configs.find(motor_id);
    if (config_it == motor_configs.end()) {
        return;
    }
    
    config_it->second.enabled = enable;
    
    if (!enable) {
        motor_stop(motor_id);
    }
    
    const char* state = enable ? "enabled" : "disabled";
    std::cout << "[MOTOR] Motor " << (int)motor_id << " " << state << std::endl;
}

motor_state_t motor_get_state(uint8_t motor_id) {
    if (!motor_initialized) motor_init();
    
    auto state_it = motor_states.find(motor_id);
    if (state_it == motor_states.end()) {
        motor_state_t empty;
        memset(&empty, 0, sizeof(empty));
        return empty;
    }
    
    return state_it->second;
}

void motor_emergency_stop(void) {
    if (!motor_initialized) motor_init();
    
    std::cout << "[MOTOR] EMERGENCY STOP - All motors stopped!" << std::endl;
    
    for (auto& pair : motor_states) {
        pair.second.direction = MOTOR_DIR_STOP;
        pair.second.speed = 0;
        pair.second.is_moving = false;
    }
    
    // Disable all PWM channels
    for (auto& pair : motor_configs) {
        pwm_enable(pair.second.pwm_channel, false);
    }
}
