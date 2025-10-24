//! FFI bindings for EmbedCore C++ Hardware Abstraction Layer
//! 
//! This crate provides unsafe Rust bindings to the C++ HAL functions.
//! The safe wrappers are provided in the `embedcore` crate.

#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use libc::{c_char, c_void};

// GPIO bindings
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub enum gpio_mode_t {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_PULLUP = 2,
    GPIO_MODE_PULLDOWN = 3,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub enum gpio_state_t {
    GPIO_LOW = 0,
    GPIO_HIGH = 1,
}

extern "C" {
    pub fn gpio_init();
    pub fn gpio_set_mode(pin: u8, mode: gpio_mode_t);
    pub fn gpio_read(pin: u8) -> gpio_state_t;
    pub fn gpio_write(pin: u8, state: gpio_state_t);
    pub fn gpio_toggle(pin: u8);
    pub fn gpio_delay_ms(ms: u32);
}

// PWM bindings
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct pwm_config_t {
    pub channel: u8,
    pub frequency_hz: u32,
    pub duty_cycle_percent: u8,
    pub enabled: bool,
}

extern "C" {
    pub fn pwm_init();
    pub fn pwm_configure(channel: u8, frequency_hz: u32) -> bool;
    pub fn pwm_set_duty_cycle(channel: u8, duty_cycle_percent: u8) -> bool;
    pub fn pwm_enable(channel: u8, enable: bool);
    pub fn pwm_get_config(channel: u8) -> pwm_config_t;
    pub fn pwm_angle_to_duty_cycle(angle_degrees: f32) -> u8;
}

// UART bindings
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct uart_config_t {
    pub baud_rate: u32,
    pub data_bits: u8,
    pub stop_bits: u8,
    pub parity_enabled: bool,
    pub parity_odd: bool,
}

extern "C" {
    pub fn uart_init();
    pub fn uart_configure(port: u8, config: *const uart_config_t) -> bool;
    pub fn uart_send(port: u8, data: *const u8, length: u16) -> bool;
    pub fn uart_receive(port: u8, buffer: *mut u8, max_length: u16) -> u16;
    pub fn uart_data_available(port: u8) -> bool;
    pub fn uart_send_string(port: u8, str: *const c_char) -> bool;
    pub fn uart_flush(port: u8);
}

// Timer bindings
pub type timer_callback_t = Option<unsafe extern "C" fn()>;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct timer_config_t {
    pub period_ms: u32,
    pub auto_reload: bool,
    pub callback: timer_callback_t,
    pub enabled: bool,
}

extern "C" {
    pub fn timer_init();
    pub fn timer_configure(timer_id: u8, config: *const timer_config_t) -> bool;
    pub fn timer_start(timer_id: u8);
    pub fn timer_stop(timer_id: u8);
    pub fn timer_get_ms() -> u32;
    pub fn timer_get_us() -> u64;
    pub fn timer_delay_ms(ms: u32);
    pub fn timer_delay_us(us: u32);
    pub fn timer_is_running(timer_id: u8) -> bool;
}

// Motor bindings
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub enum motor_type_t {
    MOTOR_TYPE_DC = 0,
    MOTOR_TYPE_SERVO = 1,
    MOTOR_TYPE_STEPPER = 2,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub enum motor_direction_t {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD = 1,
    MOTOR_DIR_REVERSE = 2,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct motor_config_t {
    pub type_: motor_type_t,
    pub pwm_channel: u8,
    pub direction_pin: u8,
    pub enable_pin: u8,
    pub max_speed: u32,
    pub min_speed: u32,
    pub enabled: bool,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct motor_state_t {
    pub direction: motor_direction_t,
    pub speed: u32,
    pub position: f32,
    pub is_moving: bool,
}

extern "C" {
    pub fn motor_init();
    pub fn motor_configure(motor_id: u8, config: *const motor_config_t) -> bool;
    pub fn motor_set_speed(motor_id: u8, speed: u32) -> bool;
    pub fn motor_set_direction(motor_id: u8, direction: motor_direction_t) -> bool;
    pub fn motor_set_position(motor_id: u8, position: f32) -> bool;
    pub fn motor_stop(motor_id: u8);
    pub fn motor_enable(motor_id: u8, enable: bool);
    pub fn motor_get_state(motor_id: u8) -> motor_state_t;
    pub fn motor_emergency_stop();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gpio_init() {
        unsafe {
            gpio_init();
        }
    }

    #[test]
    fn test_pwm_init() {
        unsafe {
            pwm_init();
        }
    }

    #[test]
    fn test_uart_init() {
        unsafe {
            uart_init();
        }
    }

    #[test]
    fn test_timer_init() {
        unsafe {
            timer_init();
        }
    }

    #[test]
    fn test_motor_init() {
        unsafe {
            motor_init();
        }
    }
}
