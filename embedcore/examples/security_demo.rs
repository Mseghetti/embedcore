//! Security Demonstration Example
//! 
//! This example demonstrates the comprehensive security features of EmbedCore,
//! including attack simulation, intrusion detection, and system protection.

use embedcore::{
    init_with_security, delay_ms, get_time_ms,
    SecurityConfig, AttackType, ThreatLevel,
    get_security_monitor, emergency_stop
};
use embedcore::devices::motor::{Motor, MotorConfig, MotorType};
use embedcore::devices::gpio::{Gpio, GpioMode, GpioState};
use std::thread;
use std::time::Duration;

fn main() {
    println!("🛡️  EmbedCore Security Demonstration");
    println!("====================================");
    println!();
    
    // Configure security for demonstration
    let security_config = SecurityConfig {
        enable_attack_simulation: true,
        attack_probability: 0.4, // 40% chance for demo
        max_events_per_second: 3,
        enable_intrusion_detection: true,
        enable_rate_limiting: true,
        emergency_stop_on_critical: true,
    };
    
    // Initialize system with security
    init_with_security(security_config);
    
    let security_monitor = get_security_monitor().expect("Security monitor not initialized");
    
    println!("🔧 Initializing devices...");
    
    // Create devices
    let mut servo_config = MotorConfig::default();
    servo_config.motor_type = MotorType::Servo;
    servo_config.pwm_channel = 0;
    let servo = Motor::new(0, servo_config).expect("Failed to create servo");
    
    let led = Gpio::new(13, GpioMode::Output).expect("Failed to create LED");
    
    println!("✅ Devices initialized");
    println!("🛡️  Security monitoring active");
    println!();
    
    // Run different security scenarios
    run_scenario_1_normal_operation(&servo, &led, &security_monitor);
    run_scenario_2_dos_attack(&servo, &security_monitor);
    run_scenario_3_injection_attack(&servo, &security_monitor);
    run_scenario_4_privilege_escalation(&security_monitor);
    run_scenario_5_physical_tampering(&servo, &security_monitor);
    
    // Final security report
    println!("📊 FINAL SECURITY REPORT");
    println!("========================");
    print_comprehensive_report(&security_monitor);
    
    println!("🔒 Security demonstration completed");
}

/// Scenario 1: Normal operation with occasional security events
fn run_scenario_1_normal_operation(
    servo: &Motor, 
    led: &Gpio, 
    monitor: &embedcore::SecurityMonitor
) {
    println!("📋 SCENARIO 1: Normal Operation");
    println!("================================");
    println!("Demonstrating normal servo operation with security monitoring...");
    
    for i in 0..20 {
        // Normal servo movement
        let angle = (i * 9) as f32; // 0, 9, 18, 27, ... 171
        match servo.set_position(angle) {
            Ok(()) => {
                println!("  ✅ Servo moved to {:.0}°", angle);
            }
            Err(e) => {
                println!("  ❌ Failed to move servo: {}", e);
            }
        }
        
        // Blink LED
        led.write(GpioState::High);
        delay_ms(100);
        led.write(GpioState::Low);
        delay_ms(100);
        
        // Check for security events
        if i % 5 == 0 {
            let events = monitor.get_recent_events(2);
            if !events.is_empty() {
                println!("  🚨 Security event detected during normal operation");
                for event in events {
                    println!("    {} {:?}: {}", 
                        get_threat_level_emoji(event.threat_level),
                        event.attack_type, 
                        event.description);
                }
            }
        }
        
        delay_ms(200);
    }
    
    println!("✅ Normal operation scenario completed\n");
    delay_ms(1000);
}

/// Scenario 2: Denial of Service attack simulation
fn run_scenario_2_dos_attack(servo: &Motor, monitor: &embedcore::SecurityMonitor) {
    println!("📋 SCENARIO 2: Denial of Service Attack");
    println!("=======================================");
    println!("Simulating rapid-fire commands to overwhelm the system...");
    
    for i in 0..15 {
        let angle = (i * 12) as f32;
        match servo.set_position(angle) {
            Ok(()) => {
                println!("  📍 Command {}: Servo to {:.0}°", i + 1, angle);
            }
            Err(e) => {
                println!("  🚫 Command {} BLOCKED: {}", i + 1, e);
            }
        }
        delay_ms(50); // Very fast commands
    }
    
    // Check for DoS attacks in recent events
    let events = monitor.get_recent_events(10);
    let dos_attacks = events.iter().filter(|e| e.attack_type == AttackType::DenialOfService).count();
    println!("  📊 DoS attacks detected: {}", dos_attacks);
    
    println!("✅ DoS attack scenario completed\n");
    delay_ms(1000);
}

/// Scenario 3: Command injection attack simulation
fn run_scenario_3_injection_attack(servo: &Motor, monitor: &embedcore::SecurityMonitor) {
    println!("📋 SCENARIO 3: Command Injection Attack");
    println!("=======================================");
    println!("Simulating malicious command injection attempts...");
    
    // Simulate various injection attempts
    let malicious_commands = [
        ("Normal command", 90.0),
        ("Injection attempt", 999.0),
        ("Another normal", 45.0),
        ("Buffer overflow", -999.0),
        ("Normal again", 135.0),
        ("Privilege escalation", 181.0),
    ];
    
    for (desc, angle) in &malicious_commands {
        println!("  🔍 Testing: {}", desc);
        match servo.set_position(*angle) {
            Ok(()) => {
                println!("    ✅ Command accepted: {:.0}°", angle);
            }
            Err(e) => {
                println!("    🛡️  Command BLOCKED: {} (Error: {})", desc, e);
            }
        }
        delay_ms(300);
    }
    
    // Check for injection attacks
    let events = monitor.get_recent_events(10);
    let injection_attacks = events.iter().filter(|e| e.attack_type == AttackType::CommandInjection).count();
    println!("  📊 Injection attacks detected: {}", injection_attacks);
    
    println!("✅ Command injection scenario completed\n");
    delay_ms(1000);
}

/// Scenario 4: Privilege escalation simulation
fn run_scenario_4_privilege_escalation(monitor: &embedcore::SecurityMonitor) {
    println!("📋 SCENARIO 4: Privilege Escalation");
    println!("===================================");
    println!("Simulating unauthorized access attempts...");
    
    // Simulate privilege escalation attempts
    for i in 0..8 {
        println!("  🔓 Attempting privilege escalation #{}", i + 1);
        
        // Simulate various escalation attempts
        let escalation_commands = [
            "admin_access",
            "root_privileges", 
            "system_override",
            "emergency_bypass",
            "debug_mode",
            "maintenance_mode",
            "factory_reset",
            "firmware_update"
        ];
        
        if i < escalation_commands.len() {
            println!("    🎯 Command: {}", escalation_commands[i]);
            
            // Simulate the escalation attempt
            match embedcore::check_operation_security(escalation_commands[i], None) {
                Ok(()) => {
                    println!("    ✅ Escalation attempt processed");
                }
                Err(e) => {
                    println!("    🛡️  Escalation BLOCKED: {}", e);
                }
            }
        }
        
        delay_ms(400);
    }
    
    // Check for privilege escalation attacks
    let events = monitor.get_recent_events(10);
    let escalation_attacks = events.iter().filter(|e| e.attack_type == AttackType::PrivilegeEscalation).count();
    println!("  📊 Privilege escalation attempts: {}", escalation_attacks);
    
    println!("✅ Privilege escalation scenario completed\n");
    delay_ms(1000);
}

/// Scenario 5: Physical tampering simulation
fn run_scenario_5_physical_tampering(servo: &Motor, monitor: &embedcore::SecurityMonitor) {
    println!("📋 SCENARIO 5: Physical Tampering");
    println!("=================================");
    println!("Simulating physical security breaches...");
    
    // Simulate physical tampering scenarios
    let tampering_scenarios = [
        ("Normal operation", 90.0),
        ("Physical access detected", 0.0),
        ("Hardware modification", 180.0),
        ("Sensor bypass", 45.0),
        ("Emergency override", 135.0),
        ("System reset", 90.0),
    ];
    
    for (scenario, angle) in &tampering_scenarios {
        println!("  🔧 Scenario: {}", scenario);
        match servo.set_position(*angle) {
            Ok(()) => {
                println!("    📍 Servo moved to {:.0}°", angle);
            }
            Err(e) => {
                println!("    🚨 Physical tampering detected: {}", e);
            }
        }
        delay_ms(500);
    }
    
    // Check for physical tampering events
    let events = monitor.get_recent_events(10);
    let tampering_events = events.iter().filter(|e| e.attack_type == AttackType::PhysicalTampering).count();
    println!("  📊 Physical tampering events: {}", tampering_events);
    
    // Check if system is compromised
    if monitor.is_system_compromised() {
        println!("  🚨 SYSTEM COMPROMISED - Emergency stop required!");
        emergency_stop();
    }
    
    println!("✅ Physical tampering scenario completed\n");
    delay_ms(1000);
}

/// Print comprehensive security report
fn print_comprehensive_report(monitor: &embedcore::SecurityMonitor) {
    let report = monitor.generate_security_report();
    
    println!("🔢 Total Security Events: {}", report.total_attacks);
    println!("🛡️  Successfully Blocked: {}", report.blocked_attacks);
    println!("🚨 Critical Threats: {}", report.critical_attacks);
    println!("⚠️  System Status: {}", 
        if report.system_compromised { "COMPROMISED" } else { "SECURE" });
    
    if !report.attack_breakdown.is_empty() {
        println!("\n📈 Attack Type Breakdown:");
        for (attack_type, count) in &report.attack_breakdown {
            let emoji = get_attack_emoji(*attack_type);
            let level = get_threat_level_for_attack(*attack_type);
            let level_emoji = get_threat_level_emoji(level);
            println!("  {} {} {:?}: {} attacks", emoji, level_emoji, attack_type, count);
        }
    }
    
    // Show recent critical events
    let recent_events = monitor.get_recent_events(5);
    let critical_events: Vec<_> = recent_events.iter()
        .filter(|e| e.threat_level >= ThreatLevel::High)
        .collect();
    
    if !critical_events.is_empty() {
        println!("\n🚨 Recent Critical Events:");
        for event in critical_events {
            let emoji = if event.blocked { "🛡️" } else { "⚠️" };
            println!("  {} {:?} - {}", emoji, event.attack_type, event.description);
        }
    }
    
    println!("\n🔒 Security demonstration completed successfully!");
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

/// Get threat level for attack type
fn get_threat_level_for_attack(attack_type: AttackType) -> ThreatLevel {
    match attack_type {
        AttackType::DenialOfService => ThreatLevel::Medium,
        AttackType::CommandInjection => ThreatLevel::High,
        AttackType::PrivilegeEscalation => ThreatLevel::Critical,
        AttackType::TimingAttack => ThreatLevel::Medium,
        AttackType::BufferOverflow => ThreatLevel::High,
        AttackType::ReplayAttack => ThreatLevel::Medium,
        AttackType::ManInTheMiddle => ThreatLevel::High,
        AttackType::PhysicalTampering => ThreatLevel::Critical,
    }
}
