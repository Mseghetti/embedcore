//! EmbedCore - Safe Rust abstractions for embedded systems middleware
//! 
//! This crate provides safe, idiomatic Rust wrappers around the C++ Hardware
//! Abstraction Layer, including GPIO, PWM, UART, timer, and motor control.

pub mod devices;
pub mod scheduler;
pub mod task;
pub mod security;
pub mod web_dashboard;
pub mod control;

// Re-export commonly used types
pub use devices::{Gpio, Pwm, Uart, Motor, MotorType, MotorDirection};
pub use scheduler::Scheduler;
pub use task::{Task, TaskPriority, TaskState};
pub use security::{SecurityMonitor, SecurityConfig, SecurityError, AttackType, ThreatLevel, init_security, check_operation_security, get_security_monitor};
pub use web_dashboard::{WebDashboard, DashboardConfig, RobotState, JointInfo};
pub use control::{PidController, TrajectoryPlanner, CollisionDetector, ForwardKinematics, InverseKinematics, AdvancedControlSystem, Point3D, BoundingBox, JointConfig, TrajectoryPoint};

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

/// Initialize the EmbedCore system with security monitoring
/// 
/// This function initializes all hardware abstraction layers and security
/// monitoring. Use this instead of init() when you want security features.
pub fn init_with_security(config: SecurityConfig) {
    init();
    init_security(config);
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
