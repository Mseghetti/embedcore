//! PID Control Demo - Demonstrates PID controller for smooth motor movements
//! 
//! This example shows how to use PID controllers for precise motor control:
//! - Tuning PID parameters (Kp, Ki, Kd)
//! - Smooth position control
//! - Real-time control loop implementation
//! - Performance monitoring and analysis

use embedcore::{
    init, delay_ms, get_time_ms,
    Motor, MotorConfig, MotorType,
    PidController
};

fn main() {
    println!("🎛️  EmbedCore PID Control Demo");
    println!("=============================");
    
    // Initialize the EmbedCore system
    init();
    
    // Configure servo motor
    let mut servo_config = MotorConfig::default();
    servo_config.motor_type = MotorType::Servo;
    servo_config.pwm_channel = 0;
    servo_config.enabled = true;
    
    let servo = Motor::new(0, servo_config).expect("Failed to create servo motor");
    
    println!("✅ Servo motor connected to PWM channel 0");
    println!();
    
    // Demo 1: Basic PID Control
    demo_basic_pid(&servo);
    
    // Demo 2: PID Tuning
    demo_pid_tuning(&servo);
    
    // Demo 3: Multi-target Control
    demo_multi_target_control(&servo);
    
    // Demo 4: Performance Analysis
    demo_performance_analysis(&servo);
    
    println!("🎉 PID Control Demo Complete!");
}

/// Demo 1: Basic PID Control
fn demo_basic_pid(servo: &Motor) {
    println!("📊 Demo 1: Basic PID Control");
    println!("----------------------------");
    
    // Create PID controller
    let mut pid = PidController::new(1.0, 0.1, 0.05); // Kp=1.0, Ki=0.1, Kd=0.05
    pid.set_setpoint(90.0); // Target: 90 degrees
    pid.set_limits(-1.0, 1.0); // Output limits
    
    println!("🎯 Target: 90° | Kp=1.0, Ki=0.1, Kd=0.05");
    println!("Time(ms) | Current | Target | Error | Output | Motor Pos");
    println!("--------------------------------------------------------");
    
    let mut current_angle = 0.0;
    let start_time = get_time_ms();
    
    // PID control loop
    for iteration in 0..100 {
        let current_time = get_time_ms() - start_time;
        let error = pid.setpoint - current_angle;
        let output = pid.update(current_angle);
        
        // Simulate motor response (simplified model)
        current_angle += output * 0.8; // Motor response factor
        
        // Apply to real motor
        let _ = servo.set_position(current_angle);
        
        if iteration % 10 == 0 {
            println!("{:8} | {:7.1}° | {:6.1}° | {:5.1}° | {:6.3} | {:8.1}°", 
                    current_time, current_angle, pid.setpoint, error, output, current_angle);
        }
        
        delay_ms(50); // 50ms control loop
    }
    
    println!("✅ Basic PID control complete - Final position: {:.1}°", current_angle);
    println!();
}

/// Demo 2: PID Tuning Comparison
fn demo_pid_tuning(servo: &Motor) {
    println!("🔧 Demo 2: PID Tuning Comparison");
    println!("--------------------------------");
    
    let tuning_configs = vec![
        ("Aggressive", (3.0, 0.5, 0.2)),
        ("Moderate", (1.5, 0.3, 0.1)),
        ("Conservative", (0.8, 0.1, 0.05)),
    ];
    
    for (name, (kp, ki, kd)) in tuning_configs {
        println!("🎛️  Testing {} tuning: Kp={}, Ki={}, Kd={}", name, kp, ki, kd);
        
        let mut pid = PidController::new(kp, ki, kd);
        pid.set_setpoint(45.0); // Target: 45 degrees
        pid.set_limits(-1.0, 1.0);
        
        let mut current_angle = 0.0;
        let mut overshoot = 0.0;
        let mut settling_time = 0;
        let mut steady_state_error = 0.0;
        
        // Control loop
        for iteration in 0..80 {
            let error = pid.setpoint - current_angle;
            let output = pid.update(current_angle);
            
            // Track overshoot
            if current_angle > pid.setpoint {
                overshoot = overshoot.max(current_angle - pid.setpoint);
            }
            
            // Track settling time (within 2% of target)
            if error.abs() <= pid.setpoint * 0.02 {
                if settling_time == 0 {
                    settling_time = iteration;
                }
            } else {
                settling_time = 0;
            }
            
            // Simulate motor response
            current_angle += output * 0.6;
            
            // Apply to motor
            let _ = servo.set_position(current_angle);
            
            delay_ms(50);
        }
        
        // Calculate final steady-state error
        steady_state_error = (pid.setpoint - current_angle).abs();
        
        println!("   Results: Overshoot={:.1}°, Settling={}ms, SSE={:.2}°", 
                overshoot, settling_time * 50, steady_state_error);
        println!();
        
        // Reset for next test
        delay_ms(500);
    }
}

/// Demo 3: Multi-target Control
fn demo_multi_target_control(servo: &Motor) {
    println!("🎯 Demo 3: Multi-target Control");
    println!("-------------------------------");
    
    let mut pid = PidController::new(2.0, 0.4, 0.1);
    pid.set_limits(-1.0, 1.0);
    
    let targets = vec![0.0, 45.0, 90.0, 135.0, 180.0, 90.0, 0.0];
    
    println!("🎯 Following target sequence: {:?}", targets);
    println!("Time(ms) | Target | Current | Error | Output");
    println!("--------------------------------------------");
    
    let mut current_angle = 0.0;
    let start_time = get_time_ms();
    let mut target_index = 0;
    let mut target_reached = false;
    let mut target_start_time = 0;
    
    // Control loop
    for iteration in 0..200 {
        let current_time = get_time_ms() - start_time;
        
        // Check if current target is reached
        if target_index < targets.len() {
            let target = targets[target_index];
            pid.set_setpoint(target);
            
            let error = target - current_angle;
            
            if error.abs() < 2.0 && !target_reached {
                target_reached = true;
                target_start_time = current_time;
                println!("✅ Target {:.1}° reached at {:.1}s", target, current_time as f32 / 1000.0);
            }
            
            // Move to next target after 1 second at current target
            if target_reached && current_time - target_start_time > 1000 {
                target_index += 1;
                target_reached = false;
            }
        }
        
        let error = pid.setpoint - current_angle;
        let output = pid.update(current_angle);
        
        // Simulate motor response
        current_angle += output * 0.7;
        
        // Apply to motor
        let _ = servo.set_position(current_angle);
        
        if iteration % 15 == 0 {
            println!("{:8} | {:6.1}° | {:7.1}° | {:5.1}° | {:6.3}", 
                    current_time, pid.setpoint, current_angle, error, output);
        }
        
        delay_ms(50);
    }
    
    println!("✅ Multi-target control complete");
    println!();
}

/// Demo 4: Performance Analysis
fn demo_performance_analysis(servo: &Motor) {
    println!("📈 Demo 4: Performance Analysis");
    println!("------------------------------");
    
    let mut pid = PidController::new(1.8, 0.35, 0.08);
    pid.set_setpoint(120.0); // Target: 120 degrees
    pid.set_limits(-1.0, 1.0);
    
    let mut current_angle = 0.0;
    let mut max_error = 0.0;
    let mut total_error = 0.0;
    let mut max_overshoot = 0.0;
    let mut iterations = 0;
    let mut steady_state_reached = false;
    let mut settling_time = 0;
    
    println!("🎯 Analyzing PID performance for target: 120°");
    println!("Iteration | Current | Error | Output | Performance");
    println!("--------------------------------------------------");
    
    // Performance analysis loop
    for iteration in 0..150 {
        let error = pid.setpoint - current_angle;
        let output = pid.update(current_angle);
        
        // Track performance metrics
        max_error = max_error.max(error.abs());
        total_error += error.abs();
        
        // Track overshoot
        if current_angle > pid.setpoint {
            max_overshoot = max_overshoot.max(current_angle - pid.setpoint);
        }
        
        // Check for steady state (error < 1% for 10 iterations)
        if error.abs() < pid.setpoint * 0.01 {
            if !steady_state_reached {
                settling_time = iteration;
                steady_state_reached = true;
            }
        } else {
            steady_state_reached = false;
        }
        
        // Simulate motor response
        current_angle += output * 0.65;
        
        // Apply to motor
        let _ = servo.set_position(current_angle);
        
        if iteration % 20 == 0 {
            let performance = if error.abs() < 2.0 { "Good" } else if error.abs() < 5.0 { "Fair" } else { "Poor" };
            println!("{:9} | {:7.1}° | {:5.1}° | {:6.3} | {:10}", 
                    iteration, current_angle, error, output, performance);
        }
        
        delay_ms(50);
        iterations += 1;
    }
    
    // Calculate performance metrics
    let average_error = total_error / iterations as f32;
    let final_error = (pid.setpoint - current_angle).abs();
    let settling_time_ms = settling_time * 50;
    
    println!();
    println!("📊 Performance Analysis Results:");
    println!("   Maximum Error: {:.2}°", max_error);
    println!("   Average Error: {:.2}°", average_error);
    println!("   Final Error: {:.2}°", final_error);
    println!("   Maximum Overshoot: {:.2}°", max_overshoot);
    println!("   Settling Time: {}ms", settling_time_ms);
    println!("   Steady State Reached: {}", steady_state_reached);
    
    // Performance rating
    let performance_score = if final_error < 1.0 && max_overshoot < 5.0 && settling_time_ms < 2000 {
        "Excellent"
    } else if final_error < 2.0 && max_overshoot < 10.0 && settling_time_ms < 3000 {
        "Good"
    } else if final_error < 5.0 && settling_time_ms < 5000 {
        "Fair"
    } else {
        "Needs Tuning"
    };
    
    println!("   Overall Performance: {}", performance_score);
    println!();
}

/// Helper function to print PID parameters
fn print_pid_parameters(pid: &PidController) {
    println!("   Kp: {:.2}, Ki: {:.2}, Kd: {:.2}", pid.kp, pid.ki, pid.kd);
    println!("   Setpoint: {:.1}°, Limits: [{:.1}, {:.1}]", 
             pid.setpoint, pid.output_min, pid.output_max);
}
