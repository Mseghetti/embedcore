#include "timer.h"
#include <iostream>
#include <map>
#include <chrono>
#include <thread>
#include <atomic>

// Simulated timer state
static std::map<uint8_t, timer_config_t> timer_configs;
static std::map<uint8_t, std::atomic<bool>> timer_running;
static std::atomic<uint32_t> system_time_ms{0};
static std::atomic<uint64_t> system_time_us{0};
static bool timer_initialized = false;

// Timer thread function
void timer_thread_function(uint8_t timer_id) {
    auto& config = timer_configs[timer_id];
    auto& running = timer_running[timer_id];
    
    while (running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(config.period_ms));
        
        if (running.load() && config.callback) {
            config.callback();
        }
        
        if (!config.auto_reload) {
            running.store(false);
            break;
        }
    }
}

void timer_init(void) {
    if (timer_initialized) return;
    
    std::cout << "[TIMER] Initializing timer system..." << std::endl;
    
    // Start system time counter
    std::thread([]() {
        auto start = std::chrono::high_resolution_clock::now();
        while (true) {
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
            system_time_ms.store(elapsed.count());
            
            auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
            system_time_us.store(elapsed_us.count());
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }).detach();
    
    timer_initialized = true;
}

bool timer_configure(uint8_t timer_id, const timer_config_t* config) {
    if (!timer_initialized) timer_init();
    
    if (!config) {
        std::cout << "[TIMER] Invalid configuration" << std::endl;
        return false;
    }
    
    if (config->period_ms == 0) {
        std::cout << "[TIMER] Invalid period: " << config->period_ms << " ms" << std::endl;
        return false;
    }
    
    timer_configs[timer_id] = *config;
    timer_running[timer_id] = false;
    
    std::cout << "[TIMER] Timer " << (int)timer_id << " configured: " 
              << config->period_ms << " ms, auto_reload=" 
              << (config->auto_reload ? "true" : "false") << std::endl;
    
    return true;
}

void timer_start(uint8_t timer_id) {
    if (!timer_initialized) timer_init();
    
    auto config_it = timer_configs.find(timer_id);
    if (config_it == timer_configs.end()) {
        std::cout << "[TIMER] Timer " << (int)timer_id << " not configured" << std::endl;
        return;
    }
    
    auto& running = timer_running[timer_id];
    if (running.load()) {
        std::cout << "[TIMER] Timer " << (int)timer_id << " already running" << std::endl;
        return;
    }
    
    running.store(true);
    std::thread(timer_thread_function, timer_id).detach();
    
    std::cout << "[TIMER] Timer " << (int)timer_id << " started" << std::endl;
}

void timer_stop(uint8_t timer_id) {
    if (!timer_initialized) timer_init();
    
    auto running_it = timer_running.find(timer_id);
    if (running_it == timer_running.end()) {
        return;
    }
    
    running_it->second.store(false);
    std::cout << "[TIMER] Timer " << (int)timer_id << " stopped" << std::endl;
}

uint32_t timer_get_ms(void) {
    if (!timer_initialized) timer_init();
    return system_time_ms.load();
}

uint64_t timer_get_us(void) {
    if (!timer_initialized) timer_init();
    return system_time_us.load();
}

void timer_delay_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void timer_delay_us(uint32_t us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

bool timer_is_running(uint8_t timer_id) {
    if (!timer_initialized) timer_init();
    
    auto running_it = timer_running.find(timer_id);
    if (running_it == timer_running.end()) {
        return false;
    }
    
    return running_it->second.load();
}
