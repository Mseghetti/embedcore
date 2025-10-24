//! Servo Sweep example - Servo motor control
//! 
//! This example demonstrates servo motor control by sweeping a servo
//! motor from 0 to 180 degrees and back. The servo is connected to
//! PWM channel 0.

use embedcore::{init, delay_ms};
use embedcore::devices::motor::{Motor, MotorConfig, MotorType};

fn main() {
    println!("EmbedCore Servo Sweep Example");
    println!("=============================");
    
    // Initialize the EmbedCore system
    init();
    
    // Configure servo motor
    let mut servo_config = MotorConfig::default();
    servo_config.motor_type = MotorType::Servo;
    servo_config.pwm_channel = 0;
    servo_config.enabled = true;
    
    let servo = Motor::new(0, servo_config).expect("Failed to create servo motor");
    
    println!("Servo motor connected to PWM channel 0");
    println!("Sweeping servo from 0° to 180° and back...");
    println!("Press Ctrl+C to stop");
    
    // Sweep the servo
    loop {
        // Sweep from 0 to 180 degrees
        for angle in 0..=180 {
            servo.set_position(angle as f32).expect("Failed to set servo position");
            println!("Servo position: {}°", angle);
            delay_ms(20); // 20ms delay for smooth movement
        }
        
        // Sweep from 180 to 0 degrees
        for angle in (0..=180).rev() {
            servo.set_position(angle as f32).expect("Failed to set servo position");
            println!("Servo position: {}°", angle);
            delay_ms(20); // 20ms delay for smooth movement
        }
    }
}
