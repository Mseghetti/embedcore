//! Motor device abstraction

use embedcore_sys::{self, motor_config_t, motor_state_t, motor_type_t, motor_direction_t};

/// Motor types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorType {
    Dc,
    Servo,
    Stepper,
}

impl From<MotorType> for motor_type_t {
    fn from(motor_type: MotorType) -> Self {
        match motor_type {
            MotorType::Dc => motor_type_t::MOTOR_TYPE_DC,
            MotorType::Servo => motor_type_t::MOTOR_TYPE_SERVO,
            MotorType::Stepper => motor_type_t::MOTOR_TYPE_STEPPER,
        }
    }
}

/// Motor directions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorDirection {
    Stop,
    Forward,
    Reverse,
}

impl From<MotorDirection> for motor_direction_t {
    fn from(direction: MotorDirection) -> Self {
        match direction {
            MotorDirection::Stop => motor_direction_t::MOTOR_DIR_STOP,
            MotorDirection::Forward => motor_direction_t::MOTOR_DIR_FORWARD,
            MotorDirection::Reverse => motor_direction_t::MOTOR_DIR_REVERSE,
        }
    }
}

impl From<motor_direction_t> for MotorDirection {
    fn from(direction: motor_direction_t) -> Self {
        match direction {
            motor_direction_t::MOTOR_DIR_STOP => MotorDirection::Stop,
            motor_direction_t::MOTOR_DIR_FORWARD => MotorDirection::Forward,
            motor_direction_t::MOTOR_DIR_REVERSE => MotorDirection::Reverse,
        }
    }
}

/// Motor configuration error
#[derive(Debug, thiserror::Error)]
pub enum MotorError {
    #[error("Invalid configuration")]
    InvalidConfiguration,
    #[error("Motor not configured")]
    MotorNotConfigured,
    #[error("Invalid speed: {0}")]
    InvalidSpeed(u32),
    #[error("Invalid position: {0}")]
    InvalidPosition(f32),
    #[error("Not a servo motor")]
    NotServoMotor,
}

/// Motor configuration
#[derive(Debug, Clone, Copy)]
pub struct MotorConfig {
    pub motor_type: MotorType,
    pub pwm_channel: u8,
    pub direction_pin: u8,
    pub enable_pin: u8,
    pub max_speed: u32,
    pub min_speed: u32,
    pub enabled: bool,
}

impl Default for MotorConfig {
    fn default() -> Self {
        MotorConfig {
            motor_type: MotorType::Dc,
            pwm_channel: 0,
            direction_pin: 0,
            enable_pin: 0,
            max_speed: 100,
            min_speed: 0,
            enabled: true,
        }
    }
}

impl From<MotorConfig> for motor_config_t {
    fn from(config: MotorConfig) -> Self {
        motor_config_t {
            type_: config.motor_type.into(),
            pwm_channel: config.pwm_channel,
            direction_pin: config.direction_pin,
            enable_pin: config.enable_pin,
            max_speed: config.max_speed,
            min_speed: config.min_speed,
            enabled: config.enabled,
        }
    }
}

/// Motor state
#[derive(Debug, Clone, Copy)]
pub struct MotorState {
    pub direction: MotorDirection,
    pub speed: u32,
    pub position: f32,
    pub is_moving: bool,
}

impl From<motor_state_t> for MotorState {
    fn from(state: motor_state_t) -> Self {
        MotorState {
            direction: state.direction.into(),
            speed: state.speed,
            position: state.position,
            is_moving: state.is_moving,
        }
    }
}

/// Motor device abstraction
/// 
/// Provides safe access to motors with proper error handling and
/// bounds checking.
#[derive(Debug)]
pub struct Motor {
    motor_id: u8,
    config: MotorConfig,
}

impl Motor {
    /// Create a new motor instance
    /// 
    /// # Arguments
    /// * `motor_id` - The motor ID (0-255)
    /// * `config` - The motor configuration
    /// 
    /// # Returns
    /// * `Result<Motor, MotorError>` - Ok(Motor) if successful, Err otherwise
    pub fn new(motor_id: u8, config: MotorConfig) -> Result<Self, MotorError> {
        let motor = Motor { motor_id, config };
        motor.configure()?;
        Ok(motor)
    }
    
    /// Configure the motor
    fn configure(&self) -> Result<(), MotorError> {
        let config = self.config.into();
        let success = unsafe {
            embedcore_sys::motor_configure(self.motor_id, &config)
        };
        
        if success {
            Ok(())
        } else {
            Err(MotorError::MotorNotConfigured)
        }
    }
    
    /// Set motor speed
    /// 
    /// # Arguments
    /// * `speed` - Speed as percentage (0-100)
    /// 
    /// # Returns
    /// * `Result<(), MotorError>` - Ok if successful, Err otherwise
    pub fn set_speed(&self, speed: u32) -> Result<(), MotorError> {
        // Check security before operation
        if let Err(security_error) = crate::check_operation_security("motor_set_speed", Some(self.motor_id)) {
            eprintln!("🚨 SECURITY ALERT: {} - Motor {} speed change blocked", security_error, self.motor_id);
            return Err(MotorError::InvalidSpeed(speed)); // Convert security error to motor error
        }
        
        if speed > 100 {
            return Err(MotorError::InvalidSpeed(speed));
        }
        
        let success = unsafe {
            embedcore_sys::motor_set_speed(self.motor_id, speed)
        };
        
        if success {
            Ok(())
        } else {
            Err(MotorError::MotorNotConfigured)
        }
    }
    
    /// Set motor direction
    /// 
    /// # Arguments
    /// * `direction` - The motor direction
    /// 
    /// # Returns
    /// * `Result<(), MotorError>` - Ok if successful, Err otherwise
    pub fn set_direction(&self, direction: MotorDirection) -> Result<(), MotorError> {
        let success = unsafe {
            embedcore_sys::motor_set_direction(self.motor_id, direction.into())
        };
        
        if success {
            Ok(())
        } else {
            Err(MotorError::MotorNotConfigured)
        }
    }
    
    /// Set servo motor position
    /// 
    /// # Arguments
    /// * `position` - Position in degrees (0-180)
    /// 
    /// # Returns
    /// * `Result<(), MotorError>` - Ok if successful, Err otherwise
    pub fn set_position(&self, position: f32) -> Result<(), MotorError> {
        // Check security before operation
        if let Err(security_error) = crate::check_operation_security("motor_set_position", Some(self.motor_id)) {
            eprintln!("🚨 SECURITY ALERT: {} - Motor {} position change blocked", security_error, self.motor_id);
            return Err(MotorError::InvalidPosition(position)); // Convert security error to motor error
        }
        
        if self.config.motor_type != MotorType::Servo {
            return Err(MotorError::NotServoMotor);
        }
        
        if position < 0.0 || position > 180.0 {
            return Err(MotorError::InvalidPosition(position));
        }
        
        let success = unsafe {
            embedcore_sys::motor_set_position(self.motor_id, position)
        };
        
        if success {
            Ok(())
        } else {
            Err(MotorError::MotorNotConfigured)
        }
    }
    
    /// Stop the motor
    pub fn stop(&self) {
        unsafe {
            embedcore_sys::motor_stop(self.motor_id);
        }
    }
    
    /// Enable or disable the motor
    /// 
    /// # Arguments
    /// * `enable` - true to enable, false to disable
    pub fn enable(&self, enable: bool) {
        unsafe {
            embedcore_sys::motor_enable(self.motor_id, enable);
        }
    }
    
    /// Get the current motor state
    pub fn get_state(&self) -> MotorState {
        unsafe {
            embedcore_sys::motor_get_state(self.motor_id).into()
        }
    }
    
    /// Get the motor ID
    pub fn motor_id(&self) -> u8 {
        self.motor_id
    }
    
    /// Get the motor configuration
    pub fn config(&self) -> MotorConfig {
        self.config
    }
    
    /// Get the motor type
    pub fn motor_type(&self) -> MotorType {
        self.config.motor_type
    }
}

impl Drop for Motor {
    fn drop(&mut self) {
        // Stop motor for safety
        self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motor_creation() {
        let config = MotorConfig::default();
        let motor = Motor::new(0, config);
        assert!(motor.is_ok());
    }

    #[test]
    fn test_motor_speed() {
        let config = MotorConfig::default();
        let motor = Motor::new(0, config).unwrap();
        assert!(motor.set_speed(50).is_ok());
        assert!(motor.set_speed(101).is_err());
    }

    #[test]
    fn test_motor_direction() {
        let config = MotorConfig::default();
        let motor = Motor::new(0, config).unwrap();
        assert!(motor.set_direction(MotorDirection::Forward).is_ok());
        assert!(motor.set_direction(MotorDirection::Reverse).is_ok());
        assert!(motor.set_direction(MotorDirection::Stop).is_ok());
    }

    #[test]
    fn test_servo_position() {
        let mut config = MotorConfig::default();
        config.motor_type = MotorType::Servo;
        let motor = Motor::new(0, config).unwrap();
        assert!(motor.set_position(90.0).is_ok());
        assert!(motor.set_position(181.0).is_err());
        assert!(motor.set_position(-1.0).is_err());
    }

    #[test]
    fn test_dc_motor_position_error() {
        let config = MotorConfig::default(); // DC motor by default
        let motor = Motor::new(0, config).unwrap();
        assert!(motor.set_position(90.0).is_err());
    }
}
