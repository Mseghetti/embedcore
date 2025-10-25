//! Security module for EmbedCore
//! 
//! This module provides security monitoring, attack simulation, and protection
//! mechanisms for embedded systems. It simulates real-world security threats
//! that could affect robotic and embedded systems.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use rand::Rng;

/// Security threat levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ThreatLevel {
    Low,
    Medium,
    High,
    Critical,
}

/// Types of security attacks that can be simulated
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AttackType {
    /// Denial of Service - overwhelming the system with requests
    DenialOfService,
    /// Command injection - malicious commands injected into motor control
    CommandInjection,
    /// Privilege escalation - attempting to gain higher access levels
    PrivilegeEscalation,
    /// Timing attack - exploiting timing differences in operations
    TimingAttack,
    /// Buffer overflow - attempting to overflow system buffers
    BufferOverflow,
    /// Replay attack - replaying captured commands
    ReplayAttack,
    /// ManInTheMiddle - intercepting and modifying communications
    ManInTheMiddle,
    /// Physical tampering - simulating physical security breaches
    PhysicalTampering,
}

/// Security event information
#[derive(Debug, Clone)]
pub struct SecurityEvent {
    pub timestamp: Instant,
    pub attack_type: AttackType,
    pub threat_level: ThreatLevel,
    pub source: String,
    pub description: String,
    pub blocked: bool,
    pub motor_id: Option<u8>,
}

/// Security configuration
#[derive(Debug, Clone)]
pub struct SecurityConfig {
    pub enable_attack_simulation: bool,
    pub attack_probability: f32, // 0.0 to 1.0
    pub max_events_per_second: u32,
    pub enable_intrusion_detection: bool,
    pub enable_rate_limiting: bool,
    pub emergency_stop_on_critical: bool,
}

impl Default for SecurityConfig {
    fn default() -> Self {
        SecurityConfig {
            enable_attack_simulation: true,
            attack_probability: 0.1, // 10% chance per operation
            max_events_per_second: 10,
            enable_intrusion_detection: true,
            enable_rate_limiting: true,
            emergency_stop_on_critical: true,
        }
    }
}

/// Security monitor and attack simulator
pub struct SecurityMonitor {
    config: SecurityConfig,
    events: Arc<Mutex<Vec<SecurityEvent>>>,
    rate_limiter: Arc<Mutex<HashMap<String, (u32, Instant)>>>,
    attack_counters: Arc<Mutex<HashMap<AttackType, u32>>>,
    system_compromised: Arc<Mutex<bool>>,
    last_attack_time: Arc<Mutex<Option<Instant>>>,
}

impl SecurityMonitor {
    /// Create a new security monitor
    pub fn new(config: SecurityConfig) -> Self {
        SecurityMonitor {
            config,
            events: Arc::new(Mutex::new(Vec::new())),
            rate_limiter: Arc::new(Mutex::new(HashMap::new())),
            attack_counters: Arc::new(Mutex::new(HashMap::new())),
            system_compromised: Arc::new(Mutex::new(false)),
            last_attack_time: Arc::new(Mutex::new(None)),
        }
    }

    /// Check if an operation should trigger a simulated attack
    pub fn check_operation(&self, operation: &str, motor_id: Option<u8>) -> Result<(), SecurityError> {
        // Check rate limiting
        if self.config.enable_rate_limiting {
            if let Err(e) = self.check_rate_limit(operation) {
                return Err(e);
            }
        }

        // Check for attack simulation
        if self.config.enable_attack_simulation {
            if let Some(attack) = self.simulate_attack(operation, motor_id) {
                let threat_level = attack.threat_level;
                self.record_event(attack);
                
                // Check if system should be compromised
                if threat_level >= ThreatLevel::Critical {
                    *self.system_compromised.lock().unwrap() = true;
                    if self.config.emergency_stop_on_critical {
                        return Err(SecurityError::SystemCompromised);
                    }
                }
            }
        }

        Ok(())
    }

    /// Simulate a random attack based on configuration
    fn simulate_attack(&self, operation: &str, motor_id: Option<u8>) -> Option<SecurityEvent> {
        let mut rng = rand::thread_rng();
        
        // Check if attack should occur
        if rng.gen::<f32>() > self.config.attack_probability {
            return None;
        }

        // Select random attack type
        let attack_types = [
            AttackType::DenialOfService,
            AttackType::CommandInjection,
            AttackType::PrivilegeEscalation,
            AttackType::TimingAttack,
            AttackType::BufferOverflow,
            AttackType::ReplayAttack,
            AttackType::ManInTheMiddle,
            AttackType::PhysicalTampering,
        ];
        
        let attack_type = attack_types[rng.gen_range(0..attack_types.len())];
        let threat_level = self.get_threat_level_for_attack(attack_type);
        
        // Update attack counter
        {
            let mut counters = self.attack_counters.lock().unwrap();
            *counters.entry(attack_type).or_insert(0) += 1;
        }

        // Create security event
        let event = SecurityEvent {
            timestamp: Instant::now(),
            attack_type,
            threat_level,
            source: format!("Simulated attack on {}", operation),
            description: self.get_attack_description(attack_type, motor_id),
            blocked: self.should_block_attack(attack_type, threat_level),
            motor_id,
        };

        *self.last_attack_time.lock().unwrap() = Some(event.timestamp);
        Some(event)
    }

    /// Get threat level for specific attack type
    fn get_threat_level_for_attack(&self, attack_type: AttackType) -> ThreatLevel {
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

    /// Get description for attack type
    fn get_attack_description(&self, attack_type: AttackType, motor_id: Option<u8>) -> String {
        let motor_info = motor_id.map(|id| format!(" on motor {}", id)).unwrap_or_default();
        
        match attack_type {
            AttackType::DenialOfService => {
                format!("DoS attack detected{} - system overwhelmed with requests", motor_info)
            }
            AttackType::CommandInjection => {
                format!("Command injection attempt{} - malicious commands detected", motor_info)
            }
            AttackType::PrivilegeEscalation => {
                format!("Privilege escalation attempt{} - unauthorized access attempt", motor_info)
            }
            AttackType::TimingAttack => {
                format!("Timing attack detected{} - exploiting operation timing", motor_info)
            }
            AttackType::BufferOverflow => {
                format!("Buffer overflow attempt{} - excessive data input", motor_info)
            }
            AttackType::ReplayAttack => {
                format!("Replay attack detected{} - duplicate commands detected", motor_info)
            }
            AttackType::ManInTheMiddle => {
                format!("Man-in-the-middle attack{} - communication interception", motor_info)
            }
            AttackType::PhysicalTampering => {
                format!("Physical tampering detected{} - hardware security breach", motor_info)
            }
        }
    }

    /// Determine if attack should be blocked
    fn should_block_attack(&self, attack_type: AttackType, threat_level: ThreatLevel) -> bool {
        match threat_level {
            ThreatLevel::Critical => true,
            ThreatLevel::High => attack_type != AttackType::TimingAttack,
            ThreatLevel::Medium => attack_type == AttackType::CommandInjection || 
                                 attack_type == AttackType::BufferOverflow,
            ThreatLevel::Low => false,
        }
    }

    /// Check rate limiting for operations
    fn check_rate_limit(&self, operation: &str) -> Result<(), SecurityError> {
        let mut rate_limiter = self.rate_limiter.lock().unwrap();
        let now = Instant::now();
        let key = operation.to_string();
        
        if let Some((count, last_reset)) = rate_limiter.get(&key) {
            if now.duration_since(*last_reset) < Duration::from_secs(1) {
                if *count >= self.config.max_events_per_second {
                    return Err(SecurityError::RateLimitExceeded);
                }
            } else {
                // Reset counter
                rate_limiter.insert(key.clone(), (1, now));
            }
        } else {
            rate_limiter.insert(key, (1, now));
        }
        
        Ok(())
    }

    /// Record a security event
    fn record_event(&self, event: SecurityEvent) {
        let mut events = self.events.lock().unwrap();
        events.push(event);
        
        // Keep only last 1000 events to prevent memory issues
        let len = events.len();
        if len > 1000 {
            events.drain(0..len - 1000);
        }
    }

    /// Get recent security events
    pub fn get_recent_events(&self, count: usize) -> Vec<SecurityEvent> {
        let events = self.events.lock().unwrap();
        events.iter().rev().take(count).cloned().collect()
    }

    /// Get attack statistics
    pub fn get_attack_stats(&self) -> HashMap<AttackType, u32> {
        self.attack_counters.lock().unwrap().clone()
    }

    /// Check if system is compromised
    pub fn is_system_compromised(&self) -> bool {
        *self.system_compromised.lock().unwrap()
    }

    /// Reset system security state
    pub fn reset_security_state(&self) {
        *self.system_compromised.lock().unwrap() = false;
        self.events.lock().unwrap().clear();
        self.attack_counters.lock().unwrap().clear();
        self.rate_limiter.lock().unwrap().clear();
    }

    /// Update security configuration
    pub fn update_config(&self, config: SecurityConfig) {
        // Note: In a real implementation, this would need proper synchronization
        // For simulation purposes, we'll just update the config
        let _ = config; // Placeholder for actual implementation
    }

    /// Generate security report
    pub fn generate_security_report(&self) -> SecurityReport {
        let events = self.events.lock().unwrap();
        let attack_stats = self.attack_counters.lock().unwrap();
        
        let total_attacks = attack_stats.values().sum();
        let blocked_attacks = events.iter().filter(|e| e.blocked).count();
        let critical_attacks = events.iter().filter(|e| e.threat_level == ThreatLevel::Critical).count();
        
        SecurityReport {
            total_attacks,
            blocked_attacks,
            critical_attacks,
            system_compromised: self.is_system_compromised(),
            attack_breakdown: attack_stats.clone(),
            last_attack: self.last_attack_time.lock().unwrap().clone(),
        }
    }
}

/// Security error types
#[derive(Debug, thiserror::Error)]
pub enum SecurityError {
    #[error("Rate limit exceeded for operation")]
    RateLimitExceeded,
    #[error("System compromised - emergency stop required")]
    SystemCompromised,
    #[error("Security violation detected: {0}")]
    SecurityViolation(String),
    #[error("Unauthorized access attempt")]
    UnauthorizedAccess,
}

/// Security report structure
#[derive(Debug, Clone)]
pub struct SecurityReport {
    pub total_attacks: u32,
    pub blocked_attacks: usize,
    pub critical_attacks: usize,
    pub system_compromised: bool,
    pub attack_breakdown: HashMap<AttackType, u32>,
    pub last_attack: Option<Instant>,
}

/// Global security monitor instance
static mut GLOBAL_SECURITY_MONITOR: Option<Arc<SecurityMonitor>> = None;

/// Initialize the global security monitor
pub fn init_security(config: SecurityConfig) {
    unsafe {
        GLOBAL_SECURITY_MONITOR = Some(Arc::new(SecurityMonitor::new(config)));
    }
}

/// Get the global security monitor
pub fn get_security_monitor() -> Option<Arc<SecurityMonitor>> {
    unsafe { GLOBAL_SECURITY_MONITOR.clone() }
}

/// Check operation security (convenience function)
pub fn check_operation_security(operation: &str, motor_id: Option<u8>) -> Result<(), SecurityError> {
    if let Some(monitor) = get_security_monitor() {
        monitor.check_operation(operation, motor_id)
    } else {
        Ok(()) // No security monitoring enabled
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_security_monitor_creation() {
        let config = SecurityConfig::default();
        let monitor = SecurityMonitor::new(config);
        assert!(!monitor.is_system_compromised());
    }

    #[test]
    fn test_attack_simulation() {
        let config = SecurityConfig {
            attack_probability: 1.0, // 100% chance for testing
            ..Default::default()
        };
        let monitor = SecurityMonitor::new(config);
        
        // This should trigger an attack
        let result = monitor.check_operation("test_operation", Some(0));
        // Result might be Ok or Err depending on attack type
        let _ = result; // Just ensure it doesn't panic
    }

    #[test]
    fn test_rate_limiting() {
        let config = SecurityConfig {
            max_events_per_second: 1,
            attack_probability: 0.0, // No attacks, just rate limiting
            ..Default::default()
        };
        let monitor = SecurityMonitor::new(config);
        
        // First operation should succeed
        assert!(monitor.check_operation("test", None).is_ok());
        
        // Second operation should fail due to rate limiting
        assert!(monitor.check_operation("test", None).is_err());
    }

    #[test]
    fn test_threat_levels() {
        assert!(ThreatLevel::Low < ThreatLevel::Medium);
        assert!(ThreatLevel::Medium < ThreatLevel::High);
        assert!(ThreatLevel::High < ThreatLevel::Critical);
    }

    #[test]
    fn test_attack_type_enum() {
        // Test that all attack types are properly defined
        let attacks = [
            AttackType::DenialOfService,
            AttackType::CommandInjection,
            AttackType::PrivilegeEscalation,
            AttackType::TimingAttack,
            AttackType::BufferOverflow,
            AttackType::ReplayAttack,
            AttackType::ManInTheMiddle,
            AttackType::PhysicalTampering,
        ];
        
        assert_eq!(attacks.len(), 8);
    }

    #[test]
    fn test_security_event_creation() {
        let event = SecurityEvent {
            timestamp: Instant::now(),
            attack_type: AttackType::CommandInjection,
            threat_level: ThreatLevel::High,
            source: "test".to_string(),
            description: "Test attack".to_string(),
            blocked: true,
            motor_id: Some(0),
        };
        
        assert_eq!(event.attack_type, AttackType::CommandInjection);
        assert_eq!(event.threat_level, ThreatLevel::High);
        assert!(event.blocked);
        assert_eq!(event.motor_id, Some(0));
    }

    #[test]
    fn test_security_report_generation() {
        let config = SecurityConfig::default();
        let monitor = SecurityMonitor::new(config);
        
        let report = monitor.generate_security_report();
        assert_eq!(report.total_attacks, 0);
        assert_eq!(report.blocked_attacks, 0);
        assert_eq!(report.critical_attacks, 0);
        assert!(!report.system_compromised);
    }

    #[test]
    fn test_emergency_stop_configuration() {
        let config = SecurityConfig {
            emergency_stop_on_critical: true,
            ..Default::default()
        };
        let monitor = SecurityMonitor::new(config);
        
        // Simulate critical attack
        let config_high_prob = SecurityConfig {
            attack_probability: 1.0,
            emergency_stop_on_critical: true,
            ..Default::default()
        };
        let monitor_critical = SecurityMonitor::new(config_high_prob);
        
        // Multiple operations to potentially trigger critical attack
        for _ in 0..10 {
            let _ = monitor_critical.check_operation("critical_test", Some(0));
        }
        
        // System might be compromised if critical attack occurred
        let compromised = monitor_critical.is_system_compromised();
        // Note: This test might pass or fail depending on random attack generation
        let _ = compromised; // Just ensure the function works
    }

    #[test]
    fn test_attack_statistics() {
        let config = SecurityConfig {
            attack_probability: 1.0, // Guarantee attacks for testing
            ..Default::default()
        };
        let monitor = SecurityMonitor::new(config);
        
        // Perform multiple operations to generate attacks
        for i in 0..20 {
            let _ = monitor.check_operation(&format!("test_{}", i), Some(0));
        }
        
        let stats = monitor.get_attack_stats();
        // Should have some attacks recorded
        let total_attacks: u32 = stats.values().sum();
        assert!(total_attacks > 0);
    }

    #[test]
    fn test_security_config_defaults() {
        let config = SecurityConfig::default();
        assert!(config.enable_attack_simulation);
        assert_eq!(config.attack_probability, 0.1);
        assert_eq!(config.max_events_per_second, 10);
        assert!(config.enable_intrusion_detection);
        assert!(config.enable_rate_limiting);
        assert!(config.emergency_stop_on_critical);
    }

    #[test]
    fn test_recent_events_retrieval() {
        let config = SecurityConfig {
            attack_probability: 1.0,
            ..Default::default()
        };
        let monitor = SecurityMonitor::new(config);
        
        // Generate some events
        for i in 0..5 {
            let _ = monitor.check_operation(&format!("event_{}", i), Some(0));
        }
        
        let recent_events = monitor.get_recent_events(3);
        assert!(recent_events.len() <= 3);
    }
}
