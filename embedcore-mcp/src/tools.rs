//! MCP Tools for EmbedCore device control and operations

use serde::{Deserialize, Serialize};
use serde_json::json;
use embedcore::*;
use embedcore::devices::gpio::{GpioMode, GpioState};
use embedcore::devices::motor::{MotorConfig, MotorType, MotorDirection};
use embedcore::devices::uart::UartConfig;

/// Tool input for GPIO control
#[derive(Debug, Deserialize)]
pub struct GpioControlInput {
    pub pin: u8,
    pub mode: Option<String>, // "input", "output", "pullup", "pulldown"
    pub state: Option<String>, // "high", "low"
    pub action: String, // "read", "write", "toggle", "configure"
}

/// Tool input for PWM control
#[derive(Debug, Deserialize)]
pub struct PwmControlInput {
    pub channel: u8,
    pub action: String, // "configure", "set_duty", "set_angle", "enable", "disable"
    pub frequency_hz: Option<u32>,
    pub duty_cycle_percent: Option<u8>,
    pub angle_degrees: Option<f32>,
    pub enabled: Option<bool>,
}

/// Tool input for Motor control
#[derive(Debug, Deserialize)]
pub struct MotorControlInput {
    pub motor_id: u8,
    pub action: String, // "configure", "set_speed", "set_position", "set_direction", "stop", "emergency_stop"
    pub motor_type: Option<String>, // "dc", "servo", "stepper"
    pub pwm_channel: Option<u8>,
    pub speed: Option<u32>,
    pub position: Option<f32>,
    pub direction: Option<String>, // "forward", "reverse", "stop"
}

/// Tool input for UART control
#[derive(Debug, Deserialize)]
pub struct UartControlInput {
    pub port: u8,
    pub action: String, // "configure", "send", "receive", "check_available"
    pub baud_rate: Option<u32>,
    pub data: Option<String>,
}

/// Tool input for Task scheduling
#[derive(Debug, Deserialize)]
pub struct TaskControlInput {
    pub action: String, // "create", "suspend", "resume", "remove", "list", "start_scheduler", "stop_scheduler"
    pub task_name: Option<String>,
    pub priority: Option<String>, // "critical", "high", "normal", "low"
    pub period_ms: Option<u32>,
    pub task_id: Option<u32>,
}

/// Tool input for Security monitoring
#[derive(Debug, Deserialize)]
pub struct SecurityControlInput {
    pub action: String, // "init", "get_status", "check_operation", "get_threats", "configure"
    pub config: Option<serde_json::Value>,
    pub operation: Option<String>,
}

/// Tool input for System status
#[derive(Debug, Deserialize)]
pub struct SystemStatusInput {
    pub action: String, // "get_time", "get_all_status", "init", "init_with_security"
}

pub struct ToolHandler;

impl ToolHandler {
    pub fn handle_gpio_control(input: GpioControlInput) -> Result<serde_json::Value, String> {
        embedcore::init();
        
        match input.action.as_str() {
            "read" => {
                let gpio = Gpio::new(input.pin, GpioMode::Input)
                    .ok_or_else(|| "Failed to create GPIO".to_string())?;
                let state = gpio.read();
                Ok(json!({
                    "pin": input.pin,
                    "state": format!("{:?}", state),
                    "value": matches!(state, GpioState::High)
                }))
            }
            "write" => {
                let state_str = input.state.ok_or("State required for write action")?;
                let state = match state_str.as_str() {
                    "high" => GpioState::High,
                    "low" => GpioState::Low,
                    _ => return Err("Invalid state. Use 'high' or 'low'".to_string()),
                };
                let gpio = Gpio::new(input.pin, GpioMode::Output)
                    .ok_or_else(|| "Failed to create GPIO".to_string())?;
                gpio.write(state);
                Ok(json!({
                    "pin": input.pin,
                    "action": "write",
                    "state": state_str
                }))
            }
            "toggle" => {
                let gpio = Gpio::new(input.pin, GpioMode::Output)
                    .ok_or_else(|| "Failed to create GPIO".to_string())?;
                gpio.toggle();
                Ok(json!({
                    "pin": input.pin,
                    "action": "toggle"
                }))
            }
            "configure" => {
                let mode_str = input.mode.ok_or("Mode required for configure action")?;
                let mode = match mode_str.as_str() {
                    "input" => GpioMode::Input,
                    "output" => GpioMode::Output,
                    "pullup" => GpioMode::Pullup,
                    "pulldown" => GpioMode::Pulldown,
                    _ => return Err("Invalid mode. Use 'input', 'output', 'pullup', or 'pulldown'".to_string()),
                };
                let _gpio = Gpio::new(input.pin, mode)
                    .ok_or_else(|| "Failed to configure GPIO".to_string())?;
                Ok(json!({
                    "pin": input.pin,
                    "mode": mode_str,
                    "action": "configure"
                }))
            }
            _ => Err(format!("Unknown GPIO action: {}", input.action)),
        }
    }

    pub fn handle_pwm_control(input: PwmControlInput) -> Result<serde_json::Value, String> {
        embedcore::init();
        
        match input.action.as_str() {
            "configure" => {
                let freq = input.frequency_hz.ok_or("frequency_hz required for configure")?;
                let pwm = Pwm::new(input.channel, freq)
                    .map_err(|e| format!("Failed to create PWM: {:?}", e))?;
                Ok(json!({
                    "channel": input.channel,
                    "frequency_hz": freq,
                    "action": "configure"
                }))
            }
            "set_duty" => {
                let duty = input.duty_cycle_percent.ok_or("duty_cycle_percent required")?;
                let freq = input.frequency_hz.unwrap_or(1000);
                let pwm = Pwm::new(input.channel, freq)
                    .map_err(|e| format!("Failed to create PWM: {:?}", e))?;
                pwm.set_duty_cycle(duty)
                    .map_err(|e| format!("Failed to set duty cycle: {:?}", e))?;
                Ok(json!({
                    "channel": input.channel,
                    "duty_cycle_percent": duty,
                    "action": "set_duty"
                }))
            }
            "set_angle" => {
                let angle = input.angle_degrees.ok_or("angle_degrees required")?;
                if angle < 0.0 || angle > 180.0 {
                    return Err("Angle must be between 0 and 180 degrees".to_string());
                }
                let freq = input.frequency_hz.unwrap_or(50); // Servo frequency
                let mut pwm = Pwm::new(input.channel, freq)
                    .map_err(|e| format!("Failed to create PWM: {:?}", e))?;
                let duty_cycle = Pwm::angle_to_duty_cycle(angle);
                pwm.set_duty_cycle(duty_cycle)
                    .map_err(|e| format!("Failed to set duty cycle: {:?}", e))?;
                Ok(json!({
                    "channel": input.channel,
                    "angle_degrees": angle,
                    "duty_cycle_percent": duty_cycle,
                    "action": "set_angle"
                }))
            }
            "enable" | "disable" => {
                let enabled = input.action == "enable";
                let freq = input.frequency_hz.unwrap_or(1000);
                let pwm = Pwm::new(input.channel, freq)
                    .map_err(|e| format!("Failed to create PWM: {:?}", e))?;
                pwm.enable(enabled);
                Ok(json!({
                    "channel": input.channel,
                    "enabled": enabled,
                    "action": input.action
                }))
            }
            _ => Err(format!("Unknown PWM action: {}", input.action)),
        }
    }

    pub fn handle_motor_control(input: MotorControlInput) -> Result<serde_json::Value, String> {
        embedcore::init();
        
        match input.action.as_str() {
            "configure" => {
                let mut config = MotorConfig::default();
                
                if let Some(motor_type_str) = input.motor_type {
                    config.motor_type = match motor_type_str.as_str() {
                        "dc" => MotorType::Dc,
                        "servo" => MotorType::Servo,
                        "stepper" => MotorType::Stepper,
                        _ => return Err("Invalid motor type. Use 'dc', 'servo', or 'stepper'".to_string()),
                    };
                }
                
                if let Some(pwm_ch) = input.pwm_channel {
                    config.pwm_channel = pwm_ch;
                }
                
                let motor = Motor::new(input.motor_id, config)
                    .map_err(|e| format!("Failed to create motor: {:?}", e))?;
                
                Ok(json!({
                    "motor_id": input.motor_id,
                    "motor_type": input.motor_type,
                    "pwm_channel": input.pwm_channel,
                    "action": "configure"
                }))
            }
            "set_speed" => {
                let speed = input.speed.ok_or("speed required")?;
                let config = MotorConfig::default();
                let motor = Motor::new(input.motor_id, config)
                    .map_err(|e| format!("Failed to create motor: {:?}", e))?;
                motor.set_speed(speed)
                    .map_err(|e| format!("Failed to set speed: {:?}", e))?;
                Ok(json!({
                    "motor_id": input.motor_id,
                    "speed": speed,
                    "action": "set_speed"
                }))
            }
            "set_position" => {
                let position = input.position.ok_or("position required")?;
                let mut config = MotorConfig::default();
                config.motor_type = MotorType::Servo;
                let motor = Motor::new(input.motor_id, config)
                    .map_err(|e| format!("Failed to create motor: {:?}", e))?;
                motor.set_position(position)
                    .map_err(|e| format!("Failed to set position: {:?}", e))?;
                Ok(json!({
                    "motor_id": input.motor_id,
                    "position": position,
                    "action": "set_position"
                }))
            }
            "set_direction" => {
                let direction_str = input.direction.ok_or("direction required")?;
                let direction = match direction_str.as_str() {
                    "forward" => MotorDirection::Forward,
                    "reverse" => MotorDirection::Reverse,
                    "stop" => MotorDirection::Stop,
                    _ => return Err("Invalid direction. Use 'forward', 'reverse', or 'stop'".to_string()),
                };
                let config = MotorConfig::default();
                let motor = Motor::new(input.motor_id, config)
                    .map_err(|e| format!("Failed to create motor: {:?}", e))?;
                motor.set_direction(direction)
                    .map_err(|e| format!("Failed to set direction: {:?}", e))?;
                Ok(json!({
                    "motor_id": input.motor_id,
                    "direction": direction_str,
                    "action": "set_direction"
                }))
            }
            "stop" => {
                let config = MotorConfig::default();
                let motor = Motor::new(input.motor_id, config)
                    .map_err(|e| format!("Failed to create motor: {:?}", e))?;
                motor.stop();
                Ok(json!({
                    "motor_id": input.motor_id,
                    "action": "stop"
                }))
            }
            "emergency_stop" => {
                embedcore::emergency_stop();
                Ok(json!({
                    "action": "emergency_stop",
                    "message": "All motors stopped"
                }))
            }
            _ => Err(format!("Unknown motor action: {}", input.action)),
        }
    }

    pub fn handle_uart_control(input: UartControlInput) -> Result<serde_json::Value, String> {
        embedcore::init();
        
        match input.action.as_str() {
            "configure" => {
                let baud = input.baud_rate.ok_or("baud_rate required")?;
                let config = UartConfig {
                    baud_rate: baud,
                    data_bits: 8,
                    stop_bits: 1,
                    parity_enabled: false,
                    parity_odd: false,
                };
                let uart = Uart::new(input.port, config)
                    .map_err(|e| format!("Failed to create UART: {:?}", e))?;
                Ok(json!({
                    "port": input.port,
                    "baud_rate": baud,
                    "action": "configure"
                }))
            }
            "send" => {
                let data = input.data.ok_or("data required")?;
                let baud = input.baud_rate.unwrap_or(9600);
                let config = UartConfig {
                    baud_rate: baud,
                    data_bits: 8,
                    stop_bits: 1,
                    parity_enabled: false,
                    parity_odd: false,
                };
                let uart = Uart::new(input.port, config)
                    .map_err(|e| format!("Failed to create UART: {:?}", e))?;
                uart.send(data.as_bytes())
                    .map_err(|e| format!("Failed to send data: {:?}", e))?;
                Ok(json!({
                    "port": input.port,
                    "data": data,
                    "action": "send"
                }))
            }
            "check_available" => {
                let baud = input.baud_rate.unwrap_or(9600);
                let config = UartConfig {
                    baud_rate: baud,
                    data_bits: 8,
                    stop_bits: 1,
                    parity_enabled: false,
                    parity_odd: false,
                };
                let uart = Uart::new(input.port, config)
                    .map_err(|e| format!("Failed to create UART: {:?}", e))?;
                let available = uart.data_available();
                Ok(json!({
                    "port": input.port,
                    "data_available": available,
                    "action": "check_available"
                }))
            }
            _ => Err(format!("Unknown UART action: {}", input.action)),
        }
    }

    pub fn handle_system_status(input: SystemStatusInput) -> Result<serde_json::Value, String> {
        match input.action.as_str() {
            "init" => {
                embedcore::init();
                Ok(json!({
                    "action": "init",
                    "status": "initialized"
                }))
            }
            "init_with_security" => {
                use embedcore::SecurityConfig;
                let config = SecurityConfig::default();
                embedcore::init_with_security(config);
                Ok(json!({
                    "action": "init_with_security",
                    "status": "initialized_with_security"
                }))
            }
            "get_time" => {
                embedcore::init();
                let time_ms = embedcore::get_time_ms();
                let time_us = embedcore::get_time_us();
                Ok(json!({
                    "time_ms": time_ms,
                    "time_us": time_us
                }))
            }
            "get_all_status" => {
                embedcore::init();
                Ok(json!({
                    "time_ms": embedcore::get_time_ms(),
                    "time_us": embedcore::get_time_us(),
                    "system": "running"
                }))
            }
            _ => Err(format!("Unknown system action: {}", input.action)),
        }
    }
}

