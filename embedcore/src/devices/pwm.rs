//! PWM (Pulse Width Modulation) device abstraction

use embedcore_sys::{self, pwm_config_t};

/// PWM configuration error
#[derive(Debug, thiserror::Error)]
pub enum PwmError {
    #[error("Invalid frequency: {0} Hz")]
    InvalidFrequency(u32),
    #[error("Invalid duty cycle: {0}%")]
    InvalidDutyCycle(u8),
    #[error("Channel not configured")]
    ChannelNotConfigured,
}

/// PWM device abstraction
/// 
/// Provides safe access to PWM channels with proper error handling and
/// bounds checking.
#[derive(Debug)]
pub struct Pwm {
    channel: u8,
    frequency_hz: u32,
    duty_cycle_percent: u8,
    enabled: bool,
}

impl Pwm {
    /// Create a new PWM instance for the specified channel
    /// 
    /// # Arguments
    /// * `channel` - The PWM channel number (0-255)
    /// * `frequency_hz` - The PWM frequency in Hz (1-1000000)
    /// 
    /// # Returns
    /// * `Result<Pwm, PwmError>` - Ok(Pwm) if successful, Err otherwise
    pub fn new(channel: u8, frequency_hz: u32) -> Result<Self, PwmError> {
        if frequency_hz == 0 || frequency_hz > 1_000_000 {
            return Err(PwmError::InvalidFrequency(frequency_hz));
        }
        
        let pwm = Pwm {
            channel,
            frequency_hz,
            duty_cycle_percent: 0,
            enabled: false,
        };
        
        pwm.configure()?;
        Ok(pwm)
    }
    
    /// Configure the PWM channel
    fn configure(&self) -> Result<(), PwmError> {
        let success = unsafe {
            embedcore_sys::pwm_configure(self.channel, self.frequency_hz)
        };
        
        if success {
            Ok(())
        } else {
            Err(PwmError::ChannelNotConfigured)
        }
    }
    
    /// Set the PWM duty cycle
    /// 
    /// # Arguments
    /// * `duty_cycle_percent` - Duty cycle as percentage (0-100)
    /// 
    /// # Returns
    /// * `Result<(), PwmError>` - Ok if successful, Err otherwise
    pub fn set_duty_cycle(&mut self, duty_cycle_percent: u8) -> Result<(), PwmError> {
        if duty_cycle_percent > 100 {
            return Err(PwmError::InvalidDutyCycle(duty_cycle_percent));
        }
        
        let success = unsafe {
            embedcore_sys::pwm_set_duty_cycle(self.channel, duty_cycle_percent)
        };
        
        if success {
            self.duty_cycle_percent = duty_cycle_percent;
            Ok(())
        } else {
            Err(PwmError::ChannelNotConfigured)
        }
    }
    
    /// Enable or disable the PWM channel
    /// 
    /// # Arguments
    /// * `enable` - true to enable, false to disable
    pub fn enable(&mut self, enable: bool) {
        unsafe {
            embedcore_sys::pwm_enable(self.channel, enable);
        }
        self.enabled = enable;
    }
    
    /// Get the current PWM configuration
    pub fn get_config(&self) -> pwm_config_t {
        unsafe {
            embedcore_sys::pwm_get_config(self.channel)
        }
    }
    
    /// Convert angle to servo duty cycle
    /// 
    /// # Arguments
    /// * `angle_degrees` - Angle in degrees (0-180)
    /// 
    /// # Returns
    /// * Duty cycle percentage for servo control
    pub fn angle_to_duty_cycle(angle_degrees: f32) -> u8 {
        unsafe {
            embedcore_sys::pwm_angle_to_duty_cycle(angle_degrees)
        }
    }
    
    /// Get the channel number
    pub fn channel(&self) -> u8 {
        self.channel
    }
    
    /// Get the frequency in Hz
    pub fn frequency_hz(&self) -> u32 {
        self.frequency_hz
    }
    
    /// Get the current duty cycle percentage
    pub fn duty_cycle_percent(&self) -> u8 {
        self.duty_cycle_percent
    }
    
    /// Check if the PWM channel is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}

impl Drop for Pwm {
    fn drop(&mut self) {
        // Disable PWM output for safety
        self.enable(false);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pwm_creation() {
        let pwm = Pwm::new(0, 1000);
        assert!(pwm.is_ok());
    }

    #[test]
    fn test_pwm_invalid_frequency() {
        let pwm = Pwm::new(0, 0);
        assert!(pwm.is_err());
        
        let pwm = Pwm::new(0, 2_000_000);
        assert!(pwm.is_err());
    }

    #[test]
    fn test_pwm_duty_cycle() {
        let mut pwm = Pwm::new(0, 1000).unwrap();
        assert!(pwm.set_duty_cycle(50).is_ok());
        assert_eq!(pwm.duty_cycle_percent(), 50);
    }

    #[test]
    fn test_pwm_invalid_duty_cycle() {
        let mut pwm = Pwm::new(0, 1000).unwrap();
        assert!(pwm.set_duty_cycle(101).is_err());
    }

    #[test]
    fn test_angle_to_duty_cycle() {
        let duty = Pwm::angle_to_duty_cycle(90.0);
        assert!(duty >= 5 && duty <= 10); // Should be around 7.5%
    }
}
