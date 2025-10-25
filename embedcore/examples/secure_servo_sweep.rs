//! Secure Servo Sweep example - Servo motor control with security monitoring
//! 
//! This example demonstrates servo motor control with integrated security monitoring.
//! It simulates various types of attacks and shows how the security system
//! detects and responds to them.

use embedcore::{
    init_with_security, delay_ms, get_time_ms,
    SecurityConfig, AttackType, ThreatLevel,
    get_security_monitor
};
use embedcore::devices::motor::{Motor, MotorConfig, MotorType};

fn main() {
    println!("🔒 EmbedCore Secure Servo Sweep Example");
    println!("=======================================");
    
    // Configure security with high attack probability for demonstration
    let security_config = SecurityConfig {
        enable_attack_simulation: true,
        attack_probability: 0.3, // 30% chance of attack per operation
        max_events_per_second: 5,
        enable_intrusion_detection: true,
        enable_rate_limiting: true,
        emergency_stop_on_critical: true,
    };
    
    // Initialize the EmbedCore system with security
    init_with_security(security_config);
    
    // Configure servo motor
    let mut servo_config = MotorConfig::default();
    servo_config.motor_type = MotorType::Servo;
    servo_config.pwm_channel = 0;
    servo_config.enabled = true;
    
    let servo = Motor::new(0, servo_config).expect("Failed to create servo motor");
    
    println!("✅ Servo motor connected to PWM channel 0");
    println!("🛡️  Security monitoring enabled");
    println!("🎯 Attack simulation: 30% probability per operation");
    println!("📊 Rate limiting: 5 operations per second");
    println!();
    
    // Get security monitor for reporting
    let security_monitor = get_security_monitor().expect("Security monitor not initialized");
    
    println!("🔄 Starting secure servo sweep...");
    println!("Press Ctrl+C to stop");
    println!();
    
    let mut sweep_count = 0;
    let mut last_report_time = get_time_ms();
    
    // Main servo sweep loop with security monitoring
    loop {
        sweep_count += 1;
        println!("🔄 Sweep #{} starting...", sweep_count);
        
        // Sweep from 0 to 180 degrees with security checks
        for angle in 0..=180 {
            match servo.set_position(angle as f32) {
                Ok(()) => {
                    // Operation succeeded
                    if angle % 30 == 0 { // Print every 30 degrees
                        println!("  📍 Servo position: {}°", angle);
                    }
                }
                Err(e) => {
                    println!("  ❌ Failed to set position {}°: {}", angle, e);
                    
                    // Check if system is compromised
                    if security_monitor.is_system_compromised() {
                        println!("🚨 SYSTEM COMPROMISED - Emergency stop activated!");
                        return;
                    }
                }
            }
            
            delay_ms(20); // 20ms delay for smooth movement
            
            // Print security report every 5 seconds
            let current_time = get_time_ms();
            if current_time - last_report_time >= 5000 {
                print_security_report(&security_monitor);
                last_report_time = current_time;
            }
        }
        
        // Brief pause between sweeps
        delay_ms(500);
        
        // Sweep from 180 to 0 degrees
        for angle in (0..=180).rev() {
            match servo.set_position(angle as f32) {
                Ok(()) => {
                    if angle % 30 == 0 {
                        println!("  📍 Servo position: {}°", angle);
                    }
                }
                Err(e) => {
                    println!("  ❌ Failed to set position {}°: {}", angle, e);
                    
                    if security_monitor.is_system_compromised() {
                        println!("🚨 SYSTEM COMPROMISED - Emergency stop activated!");
                        return;
                    }
                }
            }
            
            delay_ms(20);
            
            // Print security report every 5 seconds
            let current_time = get_time_ms();
            if current_time - last_report_time >= 5000 {
                print_security_report(&security_monitor);
                last_report_time = current_time;
            }
        }
        
        println!("✅ Sweep #{} completed", sweep_count);
        delay_ms(1000); // 1 second pause between sweeps
        
        // Demonstrate different attack scenarios
        if sweep_count % 3 == 0 {
            demonstrate_attack_scenarios(&servo);
        }
    }
}

/// Print a comprehensive security report
fn print_security_report(monitor: &embedcore::SecurityMonitor) {
    let report = monitor.generate_security_report();
    
    println!();
    println!("📊 SECURITY REPORT");
    println!("==================");
    println!("🔢 Total attacks detected: {}", report.total_attacks);
    println!("🛡️  Attacks blocked: {}", report.blocked_attacks);
    println!("🚨 Critical attacks: {}", report.critical_attacks);
    println!("⚠️  System compromised: {}", if report.system_compromised { "YES" } else { "NO" });
    
    if !report.attack_breakdown.is_empty() {
        println!("📈 Attack breakdown:");
        for (attack_type, count) in &report.attack_breakdown {
            let emoji = get_attack_emoji(*attack_type);
            println!("  {} {:?}: {}", emoji, attack_type, count);
        }
    }
    
    if let Some(last_attack) = report.last_attack {
        let elapsed = last_attack.elapsed();
        println!("⏰ Last attack: {:.1}s ago", elapsed.as_secs_f32());
    }
    
    // Show recent events
    let recent_events = monitor.get_recent_events(3);
    if !recent_events.is_empty() {
        println!("📋 Recent security events:");
        for event in recent_events {
            let emoji = if event.blocked { "🛡️" } else { "⚠️" };
            let level_emoji = get_threat_level_emoji(event.threat_level);
            println!("  {} {} {:?} - {}", 
                emoji, level_emoji, event.attack_type, event.description);
        }
    }
    
    println!();
}

/// Demonstrate various attack scenarios
fn demonstrate_attack_scenarios(servo: &Motor) {
    println!("🎭 Demonstrating attack scenarios...");
    
    // Scenario 1: Rapid commands (potential DoS)
    println!("  🚀 Testing rapid command scenario...");
    for i in 0..10 {
        let _ = servo.set_position((i * 18) as f32);
        delay_ms(10); // Very fast commands
    }
    
    // Scenario 2: Invalid positions (potential injection)
    println!("  💉 Testing invalid position scenario...");
    let invalid_positions = [181.0, -1.0, 999.0, -999.0];
    for pos in &invalid_positions {
        let _ = servo.set_position(*pos);
        delay_ms(50);
    }
    
    // Scenario 3: Extreme movements (potential physical tampering)
    println!("  🔧 Testing extreme movement scenario...");
    let extreme_positions = [0.0, 180.0, 0.0, 180.0, 90.0];
    for pos in &extreme_positions {
        let _ = servo.set_position(*pos);
        delay_ms(100);
    }
    
    println!("  ✅ Attack scenario demonstration completed");
    delay_ms(1000);
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

/// Get emoji for threat level
fn get_threat_level_emoji(level: ThreatLevel) -> &'static str {
    match level {
        ThreatLevel::Low => "🟢",
        ThreatLevel::Medium => "🟡",
        ThreatLevel::High => "🟠",
        ThreatLevel::Critical => "🔴",
    }
}
