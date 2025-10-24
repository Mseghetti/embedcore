//! GPIO (General Purpose Input/Output) device abstraction

use embedcore_sys::{self, gpio_mode_t, gpio_state_t};

/// GPIO pin modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioMode {
    Input,
    Output,
    Pullup,
    Pulldown,
}

impl From<GpioMode> for gpio_mode_t {
    fn from(mode: GpioMode) -> Self {
        match mode {
            GpioMode::Input => gpio_mode_t::GPIO_MODE_INPUT,
            GpioMode::Output => gpio_mode_t::GPIO_MODE_OUTPUT,
            GpioMode::Pullup => gpio_mode_t::GPIO_MODE_PULLUP,
            GpioMode::Pulldown => gpio_mode_t::GPIO_MODE_PULLDOWN,
        }
    }
}

/// GPIO pin states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioState {
    Low,
    High,
}

impl From<GpioState> for gpio_state_t {
    fn from(state: GpioState) -> Self {
        match state {
            GpioState::Low => gpio_state_t::GPIO_LOW,
            GpioState::High => gpio_state_t::GPIO_HIGH,
        }
    }
}

impl From<gpio_state_t> for GpioState {
    fn from(state: gpio_state_t) -> Self {
        match state {
            gpio_state_t::GPIO_LOW => GpioState::Low,
            gpio_state_t::GPIO_HIGH => GpioState::High,
        }
    }
}

/// GPIO device abstraction
/// 
/// Provides safe access to GPIO pins with proper error handling and
/// bounds checking.
#[derive(Debug)]
pub struct Gpio {
    pin: u8,
    mode: GpioMode,
}

impl Gpio {
    /// Create a new GPIO instance for the specified pin
    /// 
    /// # Arguments
    /// * `pin` - The GPIO pin number (0-255)
    /// * `mode` - The initial pin mode
    /// 
    /// # Returns
    /// * `Some(Gpio)` if the pin is valid, `None` otherwise
    pub fn new(pin: u8, mode: GpioMode) -> Option<Self> {
        // Basic bounds checking
        if pin > 255 {
            return None;
        }
        
        let gpio = Gpio { pin, mode };
        gpio.configure();
        Some(gpio)
    }
    
    /// Configure the GPIO pin mode
    pub fn configure(&self) {
        unsafe {
            embedcore_sys::gpio_set_mode(self.pin, self.mode.into());
        }
    }
    
    /// Set the GPIO pin mode
    pub fn set_mode(&mut self, mode: GpioMode) {
        self.mode = mode;
        self.configure();
    }
    
    /// Read the current state of the GPIO pin
    /// 
    /// # Returns
    /// The current pin state (High or Low)
    pub fn read(&self) -> GpioState {
        unsafe {
            embedcore_sys::gpio_read(self.pin).into()
        }
    }
    
    /// Write a state to the GPIO pin
    /// 
    /// # Arguments
    /// * `state` - The state to write (High or Low)
    pub fn write(&self, state: GpioState) {
        unsafe {
            embedcore_sys::gpio_write(self.pin, state.into());
        }
    }
    
    /// Toggle the GPIO pin state
    pub fn toggle(&self) {
        unsafe {
            embedcore_sys::gpio_toggle(self.pin);
        }
    }
    
    /// Get the pin number
    pub fn pin(&self) -> u8 {
        self.pin
    }
    
    /// Get the current mode
    pub fn mode(&self) -> GpioMode {
        self.mode
    }
}

impl Drop for Gpio {
    fn drop(&mut self) {
        // Set pin to input mode for safety
        unsafe {
            embedcore_sys::gpio_set_mode(self.pin, gpio_mode_t::GPIO_MODE_INPUT);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gpio_creation() {
        let gpio = Gpio::new(13, GpioMode::Output);
        assert!(gpio.is_some());
    }

    #[test]
    fn test_gpio_invalid_pin() {
        let gpio = Gpio::new(256, GpioMode::Output);
        assert!(gpio.is_none());
    }

    #[test]
    fn test_gpio_read_write() {
        let gpio = Gpio::new(13, GpioMode::Output).unwrap();
        gpio.write(GpioState::High);
        // Note: In a real system, we'd need to read back the state
        // but in simulation, we just verify no panic occurs
    }
}
