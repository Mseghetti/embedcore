#include "pwm.h"
#include <iostream>
#include <map>
#include <cmath>
#include <cstring>

// Simulated PWM state
static std::map<uint8_t, pwm_config_t> pwm_configs;
static bool pwm_initialized = false;

void pwm_init(void) {
    if (pwm_initialized) return;
    
    std::cout << "[PWM] Initializing PWM system..." << std::endl;
    pwm_initialized = true;
}

bool pwm_configure(uint8_t channel, uint32_t frequency_hz) {
    if (!pwm_initialized) pwm_init();
    
    if (frequency_hz == 0 || frequency_hz > 1000000) {
        std::cout << "[PWM] Invalid frequency: " << frequency_hz << " Hz" << std::endl;
        return false;
    }
    
    pwm_config_t config;
    config.channel = channel;
    config.frequency_hz = frequency_hz;
    config.duty_cycle_percent = 0;
    config.enabled = false;
    
    pwm_configs[channel] = config;
    std::cout << "[PWM] Channel " << (int)channel << " configured: " 
              << frequency_hz << " Hz" << std::endl;
    
    return true;
}

bool pwm_set_duty_cycle(uint8_t channel, uint8_t duty_cycle_percent) {
    if (!pwm_initialized) pwm_init();
    
    if (duty_cycle_percent > 100) {
        std::cout << "[PWM] Invalid duty cycle: " << (int)duty_cycle_percent << "%" << std::endl;
        return false;
    }
    
    auto it = pwm_configs.find(channel);
    if (it == pwm_configs.end()) {
        std::cout << "[PWM] Channel " << (int)channel << " not configured" << std::endl;
        return false;
    }
    
    it->second.duty_cycle_percent = duty_cycle_percent;
    std::cout << "[PWM] Channel " << (int)channel << " duty cycle: " 
              << (int)duty_cycle_percent << "%" << std::endl;
    
    return true;
}

void pwm_enable(uint8_t channel, bool enable) {
    if (!pwm_initialized) pwm_init();
    
    auto it = pwm_configs.find(channel);
    if (it == pwm_configs.end()) {
        std::cout << "[PWM] Channel " << (int)channel << " not configured" << std::endl;
        return;
    }
    
    it->second.enabled = enable;
    const char* state = enable ? "enabled" : "disabled";
    std::cout << "[PWM] Channel " << (int)channel << " " << state << std::endl;
}

pwm_config_t pwm_get_config(uint8_t channel) {
    if (!pwm_initialized) pwm_init();
    
    auto it = pwm_configs.find(channel);
    if (it == pwm_configs.end()) {
        pwm_config_t empty;
        memset(&empty, 0, sizeof(empty));
        return empty;
    }
    
    return it->second;
}

uint8_t pwm_angle_to_duty_cycle(float angle_degrees) {
    // Clamp angle to 0-180 degrees
    if (angle_degrees < 0.0f) angle_degrees = 0.0f;
    if (angle_degrees > 180.0f) angle_degrees = 180.0f;
    
    // Convert to duty cycle (1ms = 0°, 2ms = 180°)
    // Assuming 20ms period (50Hz), 1ms = 5%, 2ms = 10%
    float duty_cycle = 5.0f + (angle_degrees / 180.0f) * 5.0f;
    
    return (uint8_t)std::round(duty_cycle);
}
