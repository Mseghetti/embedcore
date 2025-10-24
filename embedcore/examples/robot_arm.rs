//! Robot Arm example - Multi-motor coordination
//! 
//! This example demonstrates coordinated control of multiple motors
//! simulating a 2-DOF (2 degrees of freedom) robot arm with:
//! - Base rotation servo (0-180°)
//! - Shoulder servo (0-90°)
//! - Elbow servo (0-90°)
//! - Task scheduler for coordinated movement

use embedcore::{
    init, delay_ms, get_time_ms
};
use embedcore::devices::motor::{Motor, MotorConfig, MotorType, MotorDirection};
use embedcore::{Scheduler, TaskPriority, TaskResult};
use std::sync::{Arc, Mutex};
use std::time::Duration;

// Robot arm configuration
struct RobotArm {
    base_servo: Motor,
    shoulder_servo: Motor,
    elbow_servo: Motor,
    current_base_angle: Arc<Mutex<f32>>,
    current_shoulder_angle: Arc<Mutex<f32>>,
    current_elbow_angle: Arc<Mutex<f32>>,
}

impl RobotArm {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        // Configure base servo (0-180°)
        let mut base_config = MotorConfig::default();
        base_config.motor_type = MotorType::Servo;
        base_config.pwm_channel = 0;
        base_config.enabled = true;
        let base_servo = Motor::new(0, base_config)?;
        
        // Configure shoulder servo (0-90°)
        let mut shoulder_config = MotorConfig::default();
        shoulder_config.motor_type = MotorType::Servo;
        shoulder_config.pwm_channel = 1;
        shoulder_config.enabled = true;
        let shoulder_servo = Motor::new(1, shoulder_config)?;
        
        // Configure elbow servo (0-90°)
        let mut elbow_config = MotorConfig::default();
        elbow_config.motor_type = MotorType::Servo;
        elbow_config.pwm_channel = 2;
        elbow_config.enabled = true;
        let elbow_servo = Motor::new(2, elbow_config)?;
        
        Ok(RobotArm {
            base_servo,
            shoulder_servo,
            elbow_servo,
            current_base_angle: Arc::new(Mutex::new(90.0)),
            current_shoulder_angle: Arc::new(Mutex::new(45.0)),
            current_elbow_angle: Arc::new(Mutex::new(45.0)),
        })
    }
    
    fn move_to_position(&self, base: f32, shoulder: f32, elbow: f32) -> Result<(), Box<dyn std::error::Error>> {
        // Clamp angles to valid ranges
        let base = base.clamp(0.0, 180.0);
        let shoulder = shoulder.clamp(0.0, 90.0);
        let elbow = elbow.clamp(0.0, 90.0);
        
        // Move servos to target positions
        self.base_servo.set_position(base)?;
        self.shoulder_servo.set_position(shoulder)?;
        self.elbow_servo.set_position(elbow)?;
        
        // Update current positions
        *self.current_base_angle.lock().unwrap() = base;
        *self.current_shoulder_angle.lock().unwrap() = shoulder;
        *self.current_elbow_angle.lock().unwrap() = elbow;
        
        println!("Robot arm moved to: Base={:.1}°, Shoulder={:.1}°, Elbow={:.1}°", 
                base, shoulder, elbow);
        
        Ok(())
    }
    
    fn get_current_position(&self) -> (f32, f32, f32) {
        let base = *self.current_base_angle.lock().unwrap();
        let shoulder = *self.current_shoulder_angle.lock().unwrap();
        let elbow = *self.current_elbow_angle.lock().unwrap();
        (base, shoulder, elbow)
    }
}

fn main() {
    println!("EmbedCore Robot Arm Example");
    println!("===========================");
    
    // Initialize the EmbedCore system
    init();
    
    // Create robot arm
    let robot_arm = Arc::new(RobotArm::new().expect("Failed to create robot arm"));
    
    // Create task scheduler
    let scheduler = Arc::new(Scheduler::new());
    
    // Move to home position
    robot_arm.move_to_position(90.0, 45.0, 45.0).expect("Failed to move to home position");
    delay_ms(1000);
    
    println!("Robot arm initialized at home position");
    println!("Starting coordinated movement sequence...");
    
    // Define movement sequence
    let movements = vec![
        (90.0, 45.0, 45.0),  // Home
        (0.0, 30.0, 60.0),   // Left reach
        (180.0, 30.0, 60.0), // Right reach
        (90.0, 60.0, 30.0),  // High reach
        (90.0, 20.0, 70.0),  // Low reach
        (45.0, 45.0, 45.0),  // Diagonal
        (135.0, 45.0, 45.0), // Other diagonal
        (90.0, 45.0, 45.0),  // Back to home
    ];
    
    let robot_arm_clone = Arc::clone(&robot_arm);
    let movements_clone = movements.clone();
    
    // Add movement task to scheduler
    let _task_id = scheduler.add_task(
        "robot_arm_movement".to_string(),
        TaskPriority::High,
        move || {
            static mut MOVEMENT_INDEX: usize = 0;
            static mut LAST_MOVEMENT_TIME: u32 = 0;
            
            let current_time = get_time_ms();
            
            // Move to next position every 2 seconds
            if current_time - unsafe { LAST_MOVEMENT_TIME } >= 2000 {
                let movements = &movements_clone;
                let index = unsafe { MOVEMENT_INDEX };
                
                if index < movements.len() {
                    let (base, shoulder, elbow) = movements[index];
                    if let Err(e) = robot_arm_clone.move_to_position(base, shoulder, elbow) {
                        eprintln!("Movement error: {}", e);
                        return TaskResult::Error;
                    }
                    
                    unsafe {
                        MOVEMENT_INDEX += 1;
                        LAST_MOVEMENT_TIME = current_time;
                    }
                } else {
                    // Reset to beginning
                    unsafe {
                        MOVEMENT_INDEX = 0;
                        LAST_MOVEMENT_TIME = current_time;
                    }
                }
            }
            
            TaskResult::Reschedule
        },
        Some(100), // Run every 100ms
    ).expect("Failed to add movement task");
    
    // Add telemetry task
    let robot_arm_telemetry = Arc::clone(&robot_arm);
    let _telemetry_task_id = scheduler.add_task(
        "telemetry".to_string(),
        TaskPriority::Normal,
        move || {
            let (base, shoulder, elbow) = robot_arm_telemetry.get_current_position();
            println!("Telemetry: Base={:.1}°, Shoulder={:.1}°, Elbow={:.1}°", 
                    base, shoulder, elbow);
            TaskResult::Reschedule
        },
        Some(5000), // Run every 5 seconds
    ).expect("Failed to add telemetry task");
    
    // Start the scheduler
    scheduler.start().expect("Failed to start scheduler");
    
    println!("Scheduler started with {} tasks", scheduler.get_stats().total_tasks);
    println!("Press Ctrl+C to stop");
    
    // Main loop - just keep the program running
    loop {
        delay_ms(1000);
        
        // Print scheduler statistics every 10 seconds
        static mut LAST_STATS_TIME: u32 = 0;
        let current_time = get_time_ms();
        if current_time - unsafe { LAST_STATS_TIME } >= 10000 {
            let stats = scheduler.get_stats();
            println!("Scheduler Stats: Total={}, Ready={}, Running={}, Missed Deadlines={}", 
                    stats.total_tasks, stats.ready_tasks, stats.running_tasks, stats.missed_deadlines);
            unsafe { LAST_STATS_TIME = current_time; }
        }
    }
}
