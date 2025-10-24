//! Device abstractions for EmbedCore
//! 
//! This module provides safe Rust wrappers for hardware devices including
//! GPIO, PWM, UART, and motor control.

pub mod gpio;
pub mod pwm;
pub mod uart;
pub mod motor;

// Re-export device types
pub use gpio::Gpio;
pub use pwm::Pwm;
pub use uart::Uart;
pub use motor::{Motor, MotorType, MotorDirection};
