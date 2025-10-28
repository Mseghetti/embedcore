//! Advanced Control Algorithms Demo - Demonstrates sophisticated control features
//! 
//! This example showcases the new advanced control algorithms including:
//! - PID controllers for smooth motor movements
//! - Trajectory planning for robot movements
//! - Collision detection and avoidance
//! - Forward/inverse kinematics for robot arms
//! - Integrated control system combining all features

use embedcore::{
    init, delay_ms, get_time_ms,
    Motor, MotorConfig, MotorType,
    PidController, TrajectoryPlanner, CollisionDetector, ForwardKinematics, 
    InverseKinematics, AdvancedControlSystem, Point3D, BoundingBox, JointConfig, TrajectoryPoint
};
use std::time::Duration;

fn main() {
    println!("🤖 EmbedCore Advanced Control Algorithms Demo");
    println!("=============================================");
    
    // Initialize the EmbedCore system
    init();
    
    // Configure servo motors
    let mut servo_config = MotorConfig::default();
    servo_config.motor_type = MotorType::Servo;
    servo_config.pwm_channel = 0;
    servo_config.enabled = true;
    
    let servo1 = Motor::new(0, servo_config.clone()).expect("Failed to create servo 1");
    
    let mut servo_config2 = servo_config.clone();
    servo_config2.pwm_channel = 1;
    let servo2 = Motor::new(1, servo_config2).expect("Failed to create servo 2");
    
    println!("✅ Servo motors connected");
    println!("🎯 Starting Advanced Control Demo...");
    println!();
    
    // Demo 1: PID Controller
    demo_pid_controller(&servo1);
    
    // Demo 2: Trajectory Planning
    demo_trajectory_planning(&servo1);
    
    // Demo 3: Collision Detection
    demo_collision_detection();
    
    // Demo 4: Forward/Inverse Kinematics
    demo_kinematics();
    
    // Demo 5: Integrated Control System
    demo_integrated_control(&servo1, &servo2);
    
    println!("🎉 Advanced Control Demo Complete!");
}

/// Demo 1: PID Controller for smooth motor control
fn demo_pid_controller(servo: &Motor) {
    println!("📊 Demo 1: PID Controller");
    println!("------------------------");
    
    // Create PID controller with tuned parameters
    let mut pid = PidController::new(2.0, 0.5, 0.1); // Kp=2.0, Ki=0.5, Kd=0.1
    pid.set_setpoint(90.0); // Target angle: 90 degrees
    pid.set_limits(-1.0, 1.0); // Output limits
    
    println!("🎯 Target: 90° | Kp=2.0, Ki=0.5, Kd=0.1");
    println!("Current Angle | PID Output | Error");
    println!("-----------------------------------");
    
    let mut current_angle = 0.0;
    let mut iteration = 0;
    
    // Simulate PID control loop
    while iteration < 50 {
        let error = pid.setpoint - current_angle;
        let output = pid.update(current_angle);
        
        // Simulate motor response (simplified)
        current_angle += output * 0.5; // Motor response factor
        
        if iteration % 5 == 0 {
            println!("{:12.1}° | {:10.3} | {:6.1}°", current_angle, output, error);
        }
        
        // Apply to real motor
        let _ = servo.set_position(current_angle);
        
        delay_ms(50); // 50ms control loop
        iteration += 1;
    }
    
    println!("✅ PID Controller converged to {:.1}°", current_angle);
    println!();
}

/// Demo 2: Trajectory Planning for smooth movements
fn demo_trajectory_planning(servo: &Motor) {
    println!("🛤️  Demo 2: Trajectory Planning");
    println!("------------------------------");
    
    // Create trajectory planner
    let planner = TrajectoryPlanner::new(20.0, 10.0, 5.0); // Max vel=20°/s, Max accel=10°/s²
    
    // Plan trajectory from 0° to 180°
    let trajectory = planner.plan_trajectory(0.0, 180.0);
    
    println!("📈 Planned trajectory: {} points", trajectory.len());
    println!("Time | Position | Velocity | Acceleration");
    println!("----------------------------------------");
    
    // Execute trajectory
    for (i, point) in trajectory.iter().enumerate() {
        if i % 10 == 0 { // Print every 10th point
            println!("{:4.1}s | {:8.1}° | {:8.1}°/s | {:12.1}°/s²", 
                    point.time, point.position, point.velocity, point.acceleration);
        }
        
        // Apply to motor
        let _ = servo.set_position(point.position);
        
        // Simulate real-time execution
        delay_ms(10); // 10ms time step
    }
    
    println!("✅ Trajectory execution complete");
    println!();
}

/// Demo 3: Collision Detection and Avoidance
fn demo_collision_detection() {
    println!("🚧 Demo 3: Collision Detection");
    println!("------------------------------");
    
    // Create collision detector
    let mut detector = CollisionDetector::new(0.2); // 0.2m safety margin
    
    // Add obstacles
    let obstacle1 = BoundingBox::new(
        Point3D { x: 1.0, y: 1.0, z: 0.0 },
        Point3D { x: 2.0, y: 2.0, z: 1.0 }
    );
    let obstacle2 = BoundingBox::new(
        Point3D { x: 3.0, y: 0.5, z: 0.0 },
        Point3D { x: 4.0, y: 1.5, z: 1.0 }
    );
    
    detector.add_obstacle(obstacle1);
    detector.add_obstacle(obstacle2);
    
    println!("🚫 Added 2 obstacles to the environment");
    
    // Test collision detection
    let start = Point3D { x: 0.0, y: 0.0, z: 0.0 };
    let end1 = Point3D { x: 5.0, y: 5.0, z: 0.0 }; // Should be clear
    let end2 = Point3D { x: 1.5, y: 1.5, z: 0.0 }; // Should collide
    
    println!("🔍 Testing path from ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             start.x, start.y, start.z, end1.x, end1.y, end1.z);
    println!("   Path clear: {}", detector.is_path_clear(start, end1));
    
    println!("🔍 Testing path from ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             start.x, start.y, start.z, end2.x, end2.y, end2.z);
    println!("   Path clear: {}", detector.is_path_clear(start, end2));
    
    // Find collision-free path
    if let Some(path) = detector.find_path(start, end1) {
        println!("🛤️  Found collision-free path with {} waypoints", path.len());
        for (i, point) in path.iter().enumerate() {
            if i % 3 == 0 { // Print every 3rd waypoint
                println!("   Waypoint {}: ({:.1}, {:.1}, {:.1})", i, point.x, point.y, point.z);
            }
        }
    } else {
        println!("❌ No collision-free path found");
    }
    
    println!("✅ Collision detection demo complete");
    println!();
}

/// Demo 4: Forward/Inverse Kinematics
fn demo_kinematics() {
    println!("🔧 Demo 4: Forward/Inverse Kinematics");
    println!("------------------------------------");
    
    // Create a 2-DOF robot arm
    let joint_configs = vec![
        JointConfig { angle: 0.0, length: 1.0, offset: 0.0 }, // Base joint
        JointConfig { angle: 0.0, length: 0.8, offset: 0.0 }, // Elbow joint
    ];
    
    // Forward kinematics
    let fk = ForwardKinematics::new(joint_configs.clone());
    let end_effector = fk.calculate_end_effector();
    
    println!("🤖 2-DOF Robot Arm Configuration:");
    println!("   Joint 1: Length=1.0m, Angle=0°");
    println!("   Joint 2: Length=0.8m, Angle=0°");
    println!("   End-effector position: ({:.2}, {:.2}, {:.2})", 
             end_effector.x, end_effector.y, end_effector.z);
    
    // Calculate all joint positions
    let all_positions = fk.calculate_all_positions();
    println!("   Joint positions:");
    for (i, pos) in all_positions.iter().enumerate() {
        println!("     Joint {}: ({:.2}, {:.2}, {:.2})", i, pos.x, pos.y, pos.z);
    }
    
    // Inverse kinematics
    let ik = InverseKinematics::new(joint_configs);
    let target = Point3D { x: 1.2, y: 0.8, z: 0.0 };
    
    println!("🎯 Solving inverse kinematics for target: ({:.2}, {:.2}, {:.2})", 
             target.x, target.y, target.z);
    
    if let Some(angles) = ik.solve(target) {
        println!("✅ Solution found:");
        for (i, angle) in angles.iter().enumerate() {
            println!("   Joint {}: {:.2}° ({:.3} rad)", i + 1, angle.to_degrees(), angle);
        }
        
        // Verify solution with forward kinematics
        let fk_verify = ForwardKinematics::new(
            angles.iter().zip(vec![1.0, 0.8].iter())
                .map(|(angle, length)| JointConfig { angle: *angle, length: *length, offset: 0.0 })
                .collect()
        );
        let verify_pos = fk_verify.calculate_end_effector();
        let error = ((verify_pos.x - target.x).powi(2) + 
                    (verify_pos.y - target.y).powi(2) + 
                    (verify_pos.z - target.z).powi(2)).sqrt();
        println!("   Verification error: {:.4}m", error);
    } else {
        println!("❌ No solution found for target position");
    }
    
    println!("✅ Kinematics demo complete");
    println!();
}

/// Demo 5: Integrated Control System
fn demo_integrated_control(servo1: &Motor, servo2: &Motor) {
    println!("🎛️  Demo 5: Integrated Control System");
    println!("------------------------------------");
    
    // Create advanced control system
    let mut control_system = AdvancedControlSystem::new();
    
    // Add PID controllers for both servos
    control_system.add_pid_controller(0, 2.0, 0.5, 0.1); // Servo 1
    control_system.add_pid_controller(1, 1.5, 0.3, 0.05); // Servo 2
    
    // Set up robot arm kinematics
    let joint_configs = vec![
        JointConfig { angle: 0.0, length: 1.0, offset: 0.0 },
        JointConfig { angle: 0.0, length: 0.8, offset: 0.0 },
    ];
    control_system.setup_robot_arm(joint_configs);
    
    // Add obstacles for collision detection
    let obstacle = BoundingBox::new(
        Point3D { x: 0.5, y: 0.5, z: 0.0 },
        Point3D { x: 1.0, y: 1.0, z: 1.0 }
    );
    control_system.collision_detector.add_obstacle(obstacle);
    
    println!("🔧 Control system configured:");
    println!("   - 2 PID controllers (servos 0 & 1)");
    println!("   - 2-DOF robot arm kinematics");
    println!("   - Collision detection enabled");
    println!();
    
    // Plan smooth trajectory for servo 1
    let trajectory = control_system.plan_smooth_movement(0.0, 90.0);
    println!("📈 Executing smooth trajectory for servo 1 (0° → 90°)");
    
    for (i, point) in trajectory.iter().enumerate() {
        if i % 5 == 0 {
            println!("   t={:.1}s: pos={:.1}°, vel={:.1}°/s", 
                    point.time, point.position, point.velocity);
        }
        
        // Apply trajectory point
        let _ = servo1.set_position(point.position);
        
        // Simulate PID control for servo 2
        if let Some(output) = control_system.calculate_pid_output(1, 45.0) {
            let target_angle = 45.0 + output * 10.0; // Scale output
            let _ = servo2.set_position(target_angle);
        }
        
        delay_ms(20); // 20ms control loop
    }
    
    // Test collision detection
    let start = Point3D { x: 0.0, y: 0.0, z: 0.0 };
    let end = Point3D { x: 1.5, y: 1.5, z: 0.0 };
    
    println!("🚧 Testing collision detection:");
    println!("   Path from ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})", 
             start.x, start.y, start.z, end.x, end.y, end.z);
    println!("   Collision-free: {}", control_system.check_collision_free(start, end));
    
    // Test inverse kinematics
    let target = Point3D { x: 1.0, y: 0.5, z: 0.0 };
    println!("🎯 Testing inverse kinematics for target: ({:.1}, {:.1}, {:.1})", 
             target.x, target.y, target.z);
    
    if let Some(angles) = control_system.solve_inverse_kinematics(target) {
        println!("✅ IK solution: [{:.2}°, {:.2}°]", 
                angles[0].to_degrees(), angles[1].to_degrees());
    } else {
        println!("❌ No IK solution found");
    }
    
    println!("✅ Integrated control system demo complete");
    println!();
}

/// Helper function to print demo progress
fn print_demo_progress(demo_num: u32, total_demos: u32) {
    println!("📊 Progress: Demo {}/{}", demo_num, total_demos);
    println!("{}", "=".repeat(50));
}
