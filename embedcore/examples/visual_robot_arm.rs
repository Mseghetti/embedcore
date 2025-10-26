//! Visual Robot Arm Example with Web Dashboard
//! 
//! This example demonstrates a robot arm with real-time web visualization.
//! It includes a web dashboard that shows the robot's movements in real-time
//! and allows interactive control through a web interface.

use embedcore::{
    init_with_security, delay_ms, get_time_ms,
    SecurityConfig, get_security_monitor,
    WebDashboard, DashboardConfig
};
use embedcore::devices::motor::{Motor, MotorConfig, MotorType};
use embedcore::{Scheduler, TaskPriority};
use embedcore::task::TaskResult;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::time::sleep;

// Robot arm configuration
struct VisualRobotArm {
    base_servo: Motor,
    shoulder_servo: Motor,
    elbow_servo: Motor,
    current_base_angle: Arc<Mutex<f32>>,
    current_shoulder_angle: Arc<Mutex<f32>>,
    current_elbow_angle: Arc<Mutex<f32>>,
    dashboard: Arc<WebDashboard>,
}

impl VisualRobotArm {
    fn new() -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        // Configure dashboard
        let dashboard_config = DashboardConfig {
            port: 8080,
            update_interval_ms: 50, // 20 FPS
            enable_controls: true,
        };
        let dashboard = Arc::new(WebDashboard::new(dashboard_config));
        
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
        
        Ok(VisualRobotArm {
            base_servo,
            shoulder_servo,
            elbow_servo,
            current_base_angle: Arc::new(Mutex::new(90.0)),
            current_shoulder_angle: Arc::new(Mutex::new(45.0)),
            current_elbow_angle: Arc::new(Mutex::new(45.0)),
            dashboard,
        })
    }
    
    fn move_to_position(&self, base: f32, shoulder: f32, elbow: f32) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
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
        
        // Update dashboard
        self.update_dashboard();
        
        println!("🤖 Robot arm moved to: Base={:.1}°, Shoulder={:.1}°, Elbow={:.1}°", 
                base, shoulder, elbow);
        
        Ok(())
    }
    
    fn update_dashboard(&self) {
        let base_angle = *self.current_base_angle.lock().unwrap();
        let shoulder_angle = *self.current_shoulder_angle.lock().unwrap();
        let elbow_angle = *self.current_elbow_angle.lock().unwrap();
        
        // Update joint information on dashboard
        self.dashboard.update_joint(0, "Base".to_string(), base_angle, base_angle, 0.0, 180.0, false);
        self.dashboard.update_joint(1, "Shoulder".to_string(), shoulder_angle, shoulder_angle, 0.0, 90.0, false);
        self.dashboard.update_joint(2, "Elbow".to_string(), elbow_angle, elbow_angle, 0.0, 90.0, false);
        
        // Update system status
        self.dashboard.update_system_status("Running".to_string());
        
        // Update security events if available
        if let Some(monitor) = get_security_monitor() {
            let report = monitor.generate_security_report();
            self.dashboard.update_security_events(report.total_attacks);
            self.dashboard.set_emergency_stop(report.system_compromised);
        }
    }
    
    fn get_current_position(&self) -> (f32, f32, f32) {
        let base = *self.current_base_angle.lock().unwrap();
        let shoulder = *self.current_shoulder_angle.lock().unwrap();
        let elbow = *self.current_elbow_angle.lock().unwrap();
        (base, shoulder, elbow)
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    println!("🌐 EmbedCore Visual Robot Arm Example");
    println!("====================================");
    
    // Configure security
    let security_config = SecurityConfig {
        enable_attack_simulation: true,
        attack_probability: 0.1, // 10% chance for demo
        max_events_per_second: 5,
        enable_intrusion_detection: true,
        enable_rate_limiting: true,
        emergency_stop_on_critical: true,
    };
    
    // Initialize the EmbedCore system with security
    init_with_security(security_config);
    
    // Create robot arm
    let robot_arm = Arc::new(VisualRobotArm::new().expect("Failed to create robot arm"));
    
    // Start the web dashboard in a separate task
    let dashboard = Arc::clone(&robot_arm.dashboard);
    let dashboard_task = tokio::spawn(async move {
        if let Err(e) = dashboard.start().await {
            eprintln!("Dashboard error: {}", e);
        }
    });
    
    // Give the dashboard time to start
    sleep(Duration::from_millis(1000)).await;
    
    // Move to home position
    robot_arm.move_to_position(90.0, 45.0, 45.0).expect("Failed to move to home position");
    delay_ms(1000);
    
    println!("✅ Robot arm initialized at home position");
    println!("🌐 Web dashboard available at: http://localhost:8080");
    println!("🎮 Use the web interface to control the robot!");
    println!("Press Ctrl+C to stop");
    
    // Create task scheduler for automated movements
    let scheduler = Arc::new(Scheduler::new());
    
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
        "visual_robot_movement".to_string(),
        TaskPriority::High,
        move || {
            static mut MOVEMENT_INDEX: usize = 0;
            static mut LAST_MOVEMENT_TIME: u32 = 0;
            
            let current_time = get_time_ms();
            
            // Move to next position every 3 seconds
            if current_time - unsafe { LAST_MOVEMENT_TIME } >= 3000 {
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
            println!("📊 Telemetry: Base={:.1}°, Shoulder={:.1}°, Elbow={:.1}°", 
                    base, shoulder, elbow);
            TaskResult::Reschedule
        },
        Some(5000), // Run every 5 seconds
    ).expect("Failed to add telemetry task");
    
    // Start the scheduler
    scheduler.start().expect("Failed to start scheduler");
    
    println!("🚀 Scheduler started with {} tasks", scheduler.get_stats().total_tasks);
    
    // Main loop - keep the program running
    loop {
        sleep(Duration::from_millis(1000)).await;
        
        // Print scheduler statistics every 10 seconds
        static mut LAST_STATS_TIME: u32 = 0;
        let current_time = get_time_ms();
        if current_time - unsafe { LAST_STATS_TIME } >= 10000 {
            let stats = scheduler.get_stats();
            println!("📈 Scheduler Stats: Total={}, Ready={}, Running={}, Missed Deadlines={}", 
                    stats.total_tasks, stats.ready_tasks, stats.running_tasks, stats.missed_deadlines);
            unsafe { LAST_STATS_TIME = current_time; }
        }
        
        // Check if system is compromised
        if let Some(monitor) = get_security_monitor() {
            if monitor.is_system_compromised() {
                println!("🚨 SYSTEM COMPROMISED - Emergency stop activated!");
                robot_arm.dashboard.set_emergency_stop(true);
                break;
            }
        }
    }
    
    // Cleanup
    robot_arm.dashboard.stop();
    let _ = dashboard_task.await;
    
    println!("🔒 Visual robot arm example completed");
    Ok(())
}
