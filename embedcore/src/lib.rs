//! EmbedCore - Safe Rust abstractions for embedded systems middleware
//! 
//! This crate provides safe, idiomatic Rust wrappers around the C++ Hardware
//! Abstraction Layer, including GPIO, PWM, UART, timer, and motor control.

pub mod devices;
pub mod scheduler;
pub mod task;

// Re-export commonly used types
pub use devices::{Gpio, Pwm, Uart, Motor, MotorType, MotorDirection};
pub use scheduler::Scheduler;
pub use task::{Task, TaskPriority, TaskState};

/// Initialize the EmbedCore system
/// 
/// This function initializes all hardware abstraction layers and should be
/// called once at the beginning of your application.
pub fn init() {
    unsafe {
        embedcore_sys::gpio_init();
        embedcore_sys::pwm_init();
        embedcore_sys::uart_init();
        embedcore_sys::timer_init();
        embedcore_sys::motor_init();
    }
}

/// Get the current system time in milliseconds
pub fn get_time_ms() -> u32 {
    unsafe { embedcore_sys::timer_get_ms() }
}

/// Get the current system time in microseconds
pub fn get_time_us() -> u64 {
    unsafe { embedcore_sys::timer_get_us() }
}

/// Delay execution for the specified number of milliseconds
pub fn delay_ms(ms: u32) {
    unsafe { embedcore_sys::timer_delay_ms(ms) }
}

/// Delay execution for the specified number of microseconds
pub fn delay_us(us: u32) {
    unsafe { embedcore_sys::timer_delay_us(us) }
}

/// Emergency stop all motors
pub fn emergency_stop() {
    unsafe { embedcore_sys::motor_emergency_stop() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_init() {
        init();
    }

    #[test]
    fn test_time_functions() {
        init();
        let start = get_time_ms();
        delay_ms(10);
        let elapsed = get_time_ms() - start;
        assert!(elapsed >= 10);
    }
}
