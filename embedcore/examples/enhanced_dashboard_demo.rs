//! Enhanced Dashboard Demo - Demonstrates the new web dashboard features
//! 
//! This example showcases the enhanced web dashboard with:
//! - Real-time data logging and analytics
//! - Performance metrics and charts
//! - 3D robot visualization
//! - Data export capabilities
//! - Theme switching and mobile responsiveness

use embedcore::{
    init_with_security, delay_ms, get_time_ms,
    SecurityConfig, AttackType, ThreatLevel,
    get_security_monitor
};
use embedcore::devices::motor::{Motor, MotorConfig, MotorType};
use embedcore::web_dashboard::{WebDashboard, DashboardConfig};
use std::thread;
use std::time::Duration;

#[tokio::main]
async fn main() {
    println!("🚀 EmbedCore Enhanced Dashboard Demo");
    println!("====================================");
    
    // Configure security with moderate attack probability
    let security_config = SecurityConfig {
        enable_attack_simulation: true,
        attack_probability: 0.1, // 10% chance of attack per operation
        max_events_per_second: 10,
        enable_intrusion_detection: true,
        enable_rate_limiting: true,
        emergency_stop_on_critical: true,
    };
    
    // Initialize the EmbedCore system with security
    init_with_security(security_config);
    
    // Configure enhanced dashboard
    let dashboard_config = DashboardConfig {
        port: 8080,
        update_interval_ms: 50, // 20 FPS
        enable_controls: true,
        enable_data_logging: true,
        max_log_entries: 5000,
        enable_analytics: true,
    };
    
    // Create and start the web dashboard
    let dashboard = WebDashboard::new(dashboard_config);
    
    // Start dashboard in a separate task
    let dashboard_handle = {
        let dashboard = dashboard.clone();
        tokio::spawn(async move {
            if let Err(e) = dashboard.start().await {
                eprintln!("❌ Dashboard error: {}", e);
            }
        })
    };
    
    // Give the dashboard time to start
    thread::sleep(Duration::from_millis(1000));
    
    // Configure servo motors
    let mut servo_configs = Vec::new();
    let mut servos = Vec::new();
    
    for i in 0..3 {
        let mut config = MotorConfig::default();
        config.motor_type = MotorType::Servo;
        config.pwm_channel = i;
        config.enabled = true;
        
        let servo = Motor::new(i, config).expect(&format!("Failed to create servo {}", i));
        servos.push(servo);
        servo_configs.push(config);
    }
    
    println!("✅ Created 3 servo motors");
    println!("🌐 Web Dashboard available at http://localhost:8080");
    println!("📊 Features: Real-time analytics, 3D visualization, data export");
    println!("🎨 Controls: Theme toggle, 2D/3D view switching, data logging");
    println!();
    
    // Get security monitor for reporting
    let security_monitor = get_security_monitor().expect("Security monitor not initialized");
    
    let mut demo_phase = 0;
    let mut last_update = get_time_ms();
    let mut last_analytics_update = get_time_ms();
    
    println!("🔄 Starting enhanced demo sequence...");
    println!("Press Ctrl+C to stop");
    println!();
    
    // Main demo loop
    loop {
        let current_time = get_time_ms();
        
        // Update dashboard with current robot state
        update_dashboard_state(&dashboard, &servos, &security_monitor, current_time);
        
        // Log data points for analytics
        for (i, servo) in servos.iter().enumerate() {
            // Simulate some realistic data
            let position = (current_time as f32 / 1000.0).sin() * 45.0 + 90.0;
            let velocity = (current_time as f32 / 1000.0).cos() * 30.0;
            let acceleration = -(current_time as f32 / 1000.0).sin() * 20.0;
            let power_consumption = 0.5 + (velocity.abs() / 100.0) * 2.0;
            
            dashboard.log_data_point(i as u8, position, velocity, acceleration, power_consumption);
        }
        
        // Update performance metrics every 5 seconds
        if current_time - last_analytics_update >= 5000 {
            dashboard.update_performance_metrics();
            last_analytics_update = current_time;
        }
        
        // Demo different phases
        match demo_phase {
            0 => {
                // Phase 1: Gentle sweep demonstration
                if current_time - last_update >= 50 {
                    for (i, servo) in servos.iter().enumerate() {
                        let angle = 90.0 + (current_time as f32 / 2000.0 + i as f32).sin() * 30.0;
                        let _ = servo.set_position(angle);
                    }
                    last_update = current_time;
                }
                
                if current_time > 10000 {
                    demo_phase = 1;
                    println!("🎭 Phase 2: Rapid movement demonstration");
                }
            }
            1 => {
                // Phase 2: Rapid movement
                if current_time - last_update >= 20 {
                    for (i, servo) in servos.iter().enumerate() {
                        let angle = 90.0 + (current_time as f32 / 500.0 + i as f32).sin() * 60.0;
                        let _ = servo.set_position(angle);
                    }
                    last_update = current_time;
                }
                
                if current_time > 20000 {
                    demo_phase = 2;
                    println!("🎭 Phase 3: Security demonstration");
                }
            }
            2 => {
                // Phase 3: Security demonstration
                if current_time - last_update >= 100 {
                    for (i, servo) in servos.iter().enumerate() {
                        let angle = 90.0 + (current_time as f32 / 1000.0 + i as f32).sin() * 20.0;
                        let _ = servo.set_position(angle);
                    }
                    last_update = current_time;
                }
                
                if current_time > 30000 {
                    demo_phase = 0;
                    println!("🔄 Restarting demo sequence...");
                }
            }
            _ => demo_phase = 0,
        }
        
        // Print status every 10 seconds
        if current_time % 10000 < 50 {
            print_demo_status(&security_monitor, current_time);
        }
        
        delay_ms(10);
    }
}

/// Update the dashboard with current robot state
fn update_dashboard_state(
    dashboard: &WebDashboard,
    servos: &[Motor],
    security_monitor: &embedcore::SecurityMonitor,
    current_time: u32,
) {
    // Update joint information
    for (i, servo) in servos.iter().enumerate() {
        let joint_names = ["Base", "Shoulder", "Elbow"];
        let min_angles = [0.0, 0.0, 0.0];
        let max_angles = [180.0, 90.0, 90.0];
        
        // Simulate realistic joint data
        let position = 90.0 + (current_time as f32 / 1000.0 + i as f32).sin() * 30.0;
        let target = position;
        let is_moving = (current_time % 100) < 50; // Simulate movement
        let velocity = (current_time as f32 / 1000.0 + i as f32).cos() * 20.0;
        let acceleration = -(current_time as f32 / 1000.0 + i as f32).sin() * 10.0;
        let power_consumption = 0.5 + velocity.abs() / 50.0;
        
        dashboard.update_joint(
            i as u8,
            joint_names[i].to_string(),
            position,
            target,
            min_angles[i],
            max_angles[i],
            is_moving,
        );
    }
    
    // Update system status
    let system_status = if security_monitor.is_system_compromised() {
        "Compromised"
    } else {
        "Running"
    };
    dashboard.update_system_status(system_status.to_string());
    
    // Update security events
    let report = security_monitor.generate_security_report();
    dashboard.update_security_events(report.total_attacks);
    
    // Update emergency stop status
    dashboard.set_emergency_stop(report.system_compromised);
}

/// Print demo status information
fn print_demo_status(security_monitor: &embedcore::SecurityMonitor, current_time: u32) {
    let report = security_monitor.generate_security_report();
    
    println!("📊 Demo Status - Time: {}s", current_time / 1000);
    println!("  🛡️  Security Events: {} (Blocked: {})", report.total_attacks, report.blocked_attacks);
    println!("  ⚠️  System Compromised: {}", if report.system_compromised { "YES" } else { "NO" });
    println!("  📈 Attack Breakdown:");
    
    for (attack_type, count) in &report.attack_breakdown {
        let emoji = get_attack_emoji(*attack_type);
        println!("    {} {:?}: {}", emoji, attack_type, count);
    }
    
    println!("  🌐 Dashboard: http://localhost:8080");
    println!("  📊 Analytics: http://localhost:8080/api/analytics");
    println!("  📋 Export CSV: http://localhost:8080/api/export/csv");
    println!("  📄 Export JSON: http://localhost:8080/api/export/json");
    println!();
}

/// Get emoji for attack type
fn get_attack_emoji(attack_type: AttackType) -> &'static str {
    match attack_type {
        AttackType::DenialOfService => "🚫",
        AttackType::CommandInjection => "💉",
        AttackType::PrivilegeEscalation => "🔓",
        AttackType::TimingAttack => "⏱️",
        AttackType::BufferOverflow => "💥",
        AttackType::ReplayAttack => "🔄",
        AttackType::ManInTheMiddle => "👤",
        AttackType::PhysicalTampering => "🔧",
    }
}

