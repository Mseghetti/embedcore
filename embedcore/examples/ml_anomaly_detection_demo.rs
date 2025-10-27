//! ML Anomaly Detection Demo - Demonstrates machine learning-based anomaly detection
//! 
//! This example showcases the new ML-based anomaly detection system that:
//! - Learns normal behavior patterns over time
//! - Detects unusual patterns in motor operations
//! - Identifies power consumption anomalies
//! - Monitors system load and error rate patterns
//! - Provides detailed anomaly reports and statistics

use embedcore::{
    init_with_security, delay_ms, get_time_ms,
    SecurityConfig, AnomalyDetectionConfig, ThreatLevel, AnomalyType,
    get_security_monitor
};
use embedcore::devices::motor::{Motor, MotorConfig, MotorType};
use std::time::Duration;

fn main() {
    println!("🤖 EmbedCore ML Anomaly Detection Demo");
    println!("=====================================");
    
    // Configure security with ML anomaly detection
    let anomaly_config = AnomalyDetectionConfig {
        enable_ml_anomaly_detection: true,
        learning_period_seconds: 30, // 30 seconds learning period for demo
        anomaly_threshold: 0.6, // 60% confidence threshold
        max_data_points: 500,
        frequency_window_seconds: 10, // 10 second window for frequency calculation
        power_anomaly_threshold: 1.8, // 1.8x normal power
        load_anomaly_threshold: 1.3, // 1.3x normal load
        error_rate_threshold: 0.05, // 5% error rate
    };
    
    let security_config = SecurityConfig {
        enable_attack_simulation: false, // Disable random attacks for cleaner demo
        attack_probability: 0.0,
        max_events_per_second: 100,
        enable_intrusion_detection: true,
        enable_rate_limiting: false, // Disable rate limiting for demo
        emergency_stop_on_critical: true,
        anomaly_detection: anomaly_config,
    };
    
    // Initialize the EmbedCore system with ML security
    init_with_security(security_config);
    
    // Configure servo motor
    let mut servo_config = MotorConfig::default();
    servo_config.motor_type = MotorType::Servo;
    servo_config.pwm_channel = 0;
    servo_config.enabled = true;
    
    let servo = Motor::new(0, servo_config).expect("Failed to create servo motor");
    
    println!("✅ Servo motor connected to PWM channel 0");
    println!("🧠 ML Anomaly Detection enabled");
    println!("📚 Learning period: 30 seconds");
    println!("🎯 Anomaly threshold: 60% confidence");
    println!();
    
    // Get security monitor for ML operations
    let security_monitor = get_security_monitor().expect("Security monitor not initialized");
    
    println!("🔄 Starting ML Anomaly Detection Demo...");
    println!("Press Ctrl+C to stop");
    println!();
    
    let mut demo_phase = 0;
    let mut last_report_time = get_time_ms();
    let mut operation_count = 0;
    
    // Main demo loop
    loop {
        let current_time = get_time_ms();
        operation_count += 1;
        
        // Simulate different behavior patterns
        match demo_phase {
            0 => {
                // Phase 1: Normal behavior (learning period)
                if current_time < 30000 {
                    simulate_normal_behavior(&servo, &security_monitor, operation_count);
                    if current_time % 5000 < 100 {
                        println!("📚 Learning normal patterns... (Phase 1)");
                    }
                } else {
                    demo_phase = 1;
                    println!("🎯 Learning complete! Starting anomaly detection...");
                }
            }
            1 => {
                // Phase 2: Introduce anomalies
                if current_time < 60000 {
                    simulate_anomalous_behavior(&servo, &security_monitor, operation_count, current_time);
                    if current_time % 5000 < 100 {
                        println!("🚨 Detecting anomalies... (Phase 2)");
                    }
                } else {
                    demo_phase = 2;
                    println!("📊 Moving to extreme anomaly testing...");
                }
            }
            2 => {
                // Phase 3: Extreme anomalies
                simulate_extreme_anomalies(&servo, &security_monitor, operation_count, current_time);
                if current_time % 5000 < 100 {
                    println!("⚠️  Testing extreme anomaly detection... (Phase 3)");
                }
            }
            _ => demo_phase = 0,
        }
        
        // Print detailed report every 10 seconds
        if current_time - last_report_time >= 10000 {
            print_ml_security_report(&security_monitor, current_time);
            last_report_time = current_time;
        }
        
        delay_ms(100); // 100ms delay between operations
    }
}

/// Simulate normal behavior patterns for learning
fn simulate_normal_behavior(servo: &Motor, security_monitor: &embedcore::SecurityMonitor, operation_count: u32) {
    // Normal servo movements (0-180 degrees)
    let angle = (operation_count as f32 * 0.1).sin() * 45.0 + 90.0;
    let _ = servo.set_position(angle);
    
    // Record normal behavior data
    let power_consumption = 0.5 + (angle - 90.0).abs() / 180.0 * 1.0; // Normal power range
    let system_load = 0.3 + (operation_count % 10) as f32 / 100.0; // Normal load range
    let error_count = if operation_count % 50 == 0 { 1 } else { 0 }; // Occasional errors
    
    security_monitor.record_behavior_data(
        "servo_position".to_string(),
        Some(0),
        angle,
        power_consumption,
        system_load,
        error_count,
    );
}

/// Simulate anomalous behavior patterns
fn simulate_anomalous_behavior(servo: &Motor, security_monitor: &embedcore::SecurityMonitor, operation_count: u32, current_time: u32) {
    let angle = if operation_count % 20 < 5 {
        // Anomalous: Rapid movements
        (current_time as f32 / 100.0).sin() * 90.0 + 90.0
    } else if operation_count % 20 < 10 {
        // Anomalous: Extreme angles
        if operation_count % 2 == 0 { 0.0 } else { 180.0 }
    } else {
        // Normal behavior
        (operation_count as f32 * 0.1).sin() * 45.0 + 90.0
    };
    
    let _ = servo.set_position(angle);
    
    // Record behavior data with anomalies
    let power_consumption = if operation_count % 20 < 5 {
        3.0 + (angle - 90.0).abs() / 180.0 * 2.0 // High power consumption
    } else {
        0.5 + (angle - 90.0).abs() / 180.0 * 1.0 // Normal power
    };
    
    let system_load = if operation_count % 20 < 10 {
        0.8 + (operation_count % 10) as f32 / 50.0 // High system load
    } else {
        0.3 + (operation_count % 10) as f32 / 100.0 // Normal load
    };
    
    let error_count = if operation_count % 15 == 0 {
        3 // High error rate
    } else {
        0
    };
    
    security_monitor.record_behavior_data(
        "servo_position".to_string(),
        Some(0),
        angle,
        power_consumption,
        system_load,
        error_count,
    );
}

/// Simulate extreme anomalies
fn simulate_extreme_anomalies(servo: &Motor, security_monitor: &embedcore::SecurityMonitor, operation_count: u32, current_time: u32) {
    let angle = if operation_count % 10 < 3 {
        // Extreme: Impossible angles
        if operation_count % 2 == 0 { -50.0 } else { 250.0 }
    } else if operation_count % 10 < 6 {
        // Extreme: Chaotic movements
        (current_time as f32 / 50.0).sin() * 180.0
    } else {
        // Normal behavior
        (operation_count as f32 * 0.1).sin() * 45.0 + 90.0
    };
    
    let _ = servo.set_position(angle);
    
    // Record extreme behavior data
    let power_consumption = if operation_count % 10 < 3 {
        5.0 + (angle - 90.0).abs() / 180.0 * 3.0 // Extreme power consumption
    } else if operation_count % 10 < 6 {
        2.0 + (current_time as f32 / 100.0).sin() * 2.0 // Variable high power
    } else {
        0.5 + (angle - 90.0).abs() / 180.0 * 1.0 // Normal power
    };
    
    let system_load = if operation_count % 10 < 6 {
        1.0 + (operation_count % 10) as f32 / 20.0 // Very high system load
    } else {
        0.3 + (operation_count % 10) as f32 / 100.0 // Normal load
    };
    
    let error_count = if operation_count % 8 == 0 {
        5 // Very high error rate
    } else {
        0
    };
    
    security_monitor.record_behavior_data(
        "servo_position".to_string(),
        Some(0),
        angle,
        power_consumption,
        system_load,
        error_count,
    );
}

/// Print detailed ML security report
fn print_ml_security_report(security_monitor: &embedcore::SecurityMonitor, current_time: u32) {
    let report = security_monitor.generate_security_report();
    let anomaly_stats = security_monitor.get_anomaly_stats();
    let normal_patterns = security_monitor.get_normal_patterns();
    
    println!();
    println!("📊 ML ANOMALY DETECTION REPORT - Time: {}s", current_time / 1000);
    println!("================================================");
    println!("🧠 Learning Status: {}", if report.learning_active { "ACTIVE" } else { "COMPLETE" });
    println!("🚨 Total Anomalies Detected: {}", report.total_anomalies);
    println!("⚠️  System Compromised: {}", if report.system_compromised { "YES" } else { "NO" });
    println!();
    
    if !anomaly_stats.is_empty() {
        println!("📈 Anomaly Breakdown:");
        for (anomaly_type, count) in &anomaly_stats {
            let emoji = get_anomaly_emoji(*anomaly_type);
            println!("  {} {:?}: {}", emoji, anomaly_type, count);
        }
        println!();
    }
    
    if !normal_patterns.is_empty() {
        println!("📚 Learned Normal Patterns:");
        for (pattern_key, (mean, std_dev, count)) in &normal_patterns {
            if count > &10.0 { // Only show patterns with enough data
                println!("  {}: {:.2}±{:.2} (n={})", pattern_key, mean, std_dev, count as u32);
            }
        }
        println!();
    }
    
    if !report.attack_breakdown.is_empty() {
        println!("🛡️  Security Events:");
        for (attack_type, count) in &report.attack_breakdown {
            let emoji = get_attack_emoji(*attack_type);
            println!("  {} {:?}: {}", emoji, attack_type, count);
        }
        println!();
    }
    
    println!("💡 Tips:");
    println!("  - Watch for anomaly detection during Phase 2 & 3");
    println!("  - ML system learns normal patterns in Phase 1");
    println!("  - Anomalies are detected using statistical analysis");
    println!("  - System can automatically block suspicious operations");
    println!();
}

/// Get emoji for anomaly type
fn get_anomaly_emoji(anomaly_type: AnomalyType) -> &'static str {
    match anomaly_type {
        AnomalyType::FrequencyAnomaly => "⚡",
        AnomalyType::MovementAnomaly => "🤖",
        AnomalyType::TimingAnomaly => "⏱️",
        AnomalyType::PowerAnomaly => "⚡",
        AnomalyType::LoadAnomaly => "📊",
        AnomalyType::ErrorAnomaly => "❌",
    }
}

/// Get emoji for attack type
fn get_attack_emoji(attack_type: embedcore::AttackType) -> &'static str {
    match attack_type {
        embedcore::AttackType::DenialOfService => "🚫",
        embedcore::AttackType::CommandInjection => "💉",
        embedcore::AttackType::PrivilegeEscalation => "🔓",
        embedcore::AttackType::TimingAttack => "⏱️",
        embedcore::AttackType::BufferOverflow => "💥",
        embedcore::AttackType::ReplayAttack => "🔄",
        embedcore::AttackType::ManInTheMiddle => "👤",
        embedcore::AttackType::PhysicalTampering => "🔧",
        embedcore::AttackType::AnomalyDetected => "🧠",
    }
}
