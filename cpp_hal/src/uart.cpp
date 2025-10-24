#include "uart.h"
#include <iostream>
#include <map>
#include <vector>
#include <string>

// Simulated UART state
static std::map<uint8_t, uart_config_t> uart_configs;
static std::map<uint8_t, std::vector<uint8_t>> uart_tx_buffers;
static std::map<uint8_t, std::vector<uint8_t>> uart_rx_buffers;
static bool uart_initialized = false;

void uart_init(void) {
    if (uart_initialized) return;
    
    std::cout << "[UART] Initializing UART system..." << std::endl;
    uart_initialized = true;
}

bool uart_configure(uint8_t port, const uart_config_t* config) {
    if (!uart_initialized) uart_init();
    
    if (!config) {
        std::cout << "[UART] Invalid configuration" << std::endl;
        return false;
    }
    
    uart_configs[port] = *config;
    uart_tx_buffers[port].clear();
    uart_rx_buffers[port].clear();
    
    std::cout << "[UART] Port " << (int)port << " configured: " 
              << config->baud_rate << " baud" << std::endl;
    
    return true;
}

bool uart_send(uint8_t port, const uint8_t* data, uint16_t length) {
    if (!uart_initialized) uart_init();
    
    auto config_it = uart_configs.find(port);
    if (config_it == uart_configs.end()) {
        std::cout << "[UART] Port " << (int)port << " not configured" << std::endl;
        return false;
    }
    
    if (!data || length == 0) {
        return false;
    }
    
    // Simulate transmission
    std::cout << "[UART] Port " << (int)port << " TX: ";
    for (uint16_t i = 0; i < length; i++) {
        if (data[i] >= 32 && data[i] <= 126) {
            std::cout << (char)data[i];
        } else {
            std::cout << "\\x" << std::hex << (int)data[i] << std::dec;
        }
    }
    std::cout << std::endl;
    
    return true;
}

uint16_t uart_receive(uint8_t port, uint8_t* buffer, uint16_t max_length) {
    if (!uart_initialized) uart_init();
    
    auto config_it = uart_configs.find(port);
    if (config_it == uart_configs.end()) {
        return 0;
    }
    
    auto rx_it = uart_rx_buffers.find(port);
    if (rx_it == uart_rx_buffers.end() || rx_it->second.empty()) {
        return 0;
    }
    
    uint16_t bytes_to_read = std::min(max_length, (uint16_t)rx_it->second.size());
    for (uint16_t i = 0; i < bytes_to_read; i++) {
        buffer[i] = rx_it->second[i];
    }
    
    rx_it->second.erase(rx_it->second.begin(), rx_it->second.begin() + bytes_to_read);
    return bytes_to_read;
}

bool uart_data_available(uint8_t port) {
    if (!uart_initialized) uart_init();
    
    auto rx_it = uart_rx_buffers.find(port);
    return (rx_it != uart_rx_buffers.end() && !rx_it->second.empty());
}

bool uart_send_string(uint8_t port, const char* str) {
    if (!str) return false;
    
    size_t len = strlen(str);
    return uart_send(port, (const uint8_t*)str, (uint16_t)len);
}

void uart_flush(uint8_t port) {
    if (!uart_initialized) uart_init();
    
    uart_tx_buffers[port].clear();
    uart_rx_buffers[port].clear();
    std::cout << "[UART] Port " << (int)port << " buffers flushed" << std::endl;
}
