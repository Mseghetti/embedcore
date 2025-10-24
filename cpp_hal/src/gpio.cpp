#include "gpio.h"
#include <iostream>
#include <map>
#include <chrono>
#include <thread>

// Simulated GPIO state
static std::map<uint8_t, gpio_state_t> gpio_states;
static std::map<uint8_t, gpio_mode_t> gpio_modes;
static bool gpio_initialized = false;

void gpio_init(void) {
    if (gpio_initialized) return;
    
    std::cout << "[GPIO] Initializing GPIO system..." << std::endl;
    gpio_initialized = true;
}

void gpio_set_mode(uint8_t pin, gpio_mode_t mode) {
    if (!gpio_initialized) gpio_init();
    
    gpio_modes[pin] = mode;
    gpio_states[pin] = GPIO_LOW;
    
    const char* mode_str = (mode == GPIO_MODE_INPUT) ? "INPUT" :
                          (mode == GPIO_MODE_OUTPUT) ? "OUTPUT" :
                          (mode == GPIO_MODE_PULLUP) ? "PULLUP" : "PULLDOWN";
    
    std::cout << "[GPIO] Pin " << (int)pin << " set to " << mode_str << std::endl;
}

gpio_state_t gpio_read(uint8_t pin) {
    if (!gpio_initialized) gpio_init();
    
    auto it = gpio_states.find(pin);
    if (it == gpio_states.end()) {
        gpio_states[pin] = GPIO_LOW;
        return GPIO_LOW;
    }
    
    return it->second;
}

void gpio_write(uint8_t pin, gpio_state_t state) {
    if (!gpio_initialized) gpio_init();
    
    gpio_states[pin] = state;
    const char* state_str = (state == GPIO_HIGH) ? "HIGH" : "LOW";
    std::cout << "[GPIO] Pin " << (int)pin << " = " << state_str << std::endl;
}

void gpio_toggle(uint8_t pin) {
    gpio_state_t current = gpio_read(pin);
    gpio_write(pin, (current == GPIO_HIGH) ? GPIO_LOW : GPIO_HIGH);
}

void gpio_delay_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
