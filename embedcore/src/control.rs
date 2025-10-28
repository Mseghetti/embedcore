//! Advanced Control Algorithms for EmbedCore
//! 
//! This module provides sophisticated control algorithms including:
//! - PID controllers for smooth motor movements
//! - Trajectory planning for robot movements
//! - Collision detection and avoidance
//! - Forward/inverse kinematics for robot arms

use std::collections::HashMap;
use std::time::Instant;
use serde::{Deserialize, Serialize};

/// PID Controller for smooth motor control
#[derive(Debug, Clone)]
pub struct PidController {
    pub kp: f32, // Proportional gain
    pub ki: f32, // Integral gain
    pub kd: f32, // Derivative gain
    pub setpoint: f32,
    pub output_min: f32,
    pub output_max: f32,
    pub integral: f32,
    pub last_error: f32,
    pub last_time: Option<Instant>,
    pub integral_windup_limit: f32,
}

impl PidController {
    /// Create a new PID controller
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        PidController {
            kp,
            ki,
            kd,
            setpoint: 0.0,
            output_min: -1.0,
            output_max: 1.0,
            integral: 0.0,
            last_error: 0.0,
            last_time: None,
            integral_windup_limit: 100.0,
        }
    }

    /// Set the target setpoint
    pub fn set_setpoint(&mut self, setpoint: f32) {
        self.setpoint = setpoint;
    }

    /// Set output limits
    pub fn set_limits(&mut self, min: f32, max: f32) {
        self.output_min = min;
        self.output_max = max;
    }

    /// Update the PID controller and return the output
    pub fn update(&mut self, current_value: f32) -> f32 {
        let now = Instant::now();
        let dt = if let Some(last_time) = self.last_time {
            now.duration_since(last_time).as_secs_f32()
        } else {
            0.0
        };

        if dt <= 0.0 {
            self.last_time = Some(now);
            return 0.0;
        }

        let error = self.setpoint - current_value;
        
        // Proportional term
        let p_term = self.kp * error;
        
        // Integral term with windup protection
        self.integral += error * dt;
        if self.integral > self.integral_windup_limit {
            self.integral = self.integral_windup_limit;
        } else if self.integral < -self.integral_windup_limit {
            self.integral = -self.integral_windup_limit;
        }
        let i_term = self.ki * self.integral;
        
        // Derivative term
        let d_error = (error - self.last_error) / dt;
        let d_term = self.kd * d_error;
        
        // Calculate output
        let output = p_term + i_term + d_term;
        
        // Apply output limits
        let output = output.max(self.output_min).min(self.output_max);
        
        // Update state
        self.last_error = error;
        self.last_time = Some(now);
        
        output
    }

    /// Reset the PID controller
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
        self.last_time = None;
    }
}

/// Trajectory point for path planning
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    pub position: f32,
    pub velocity: f32,
    pub acceleration: f32,
    pub time: f32, // Time from start of trajectory
}

/// Trajectory planner for smooth robot movements
#[derive(Debug, Clone)]
pub struct TrajectoryPlanner {
    pub max_velocity: f32,
    pub max_acceleration: f32,
    pub max_jerk: f32,
    pub time_step: f32,
}

impl TrajectoryPlanner {
    /// Create a new trajectory planner
    pub fn new(max_velocity: f32, max_acceleration: f32, max_jerk: f32) -> Self {
        TrajectoryPlanner {
            max_velocity,
            max_acceleration,
            max_jerk,
            time_step: 0.01, // 10ms time step
        }
    }

    /// Generate a smooth trajectory from start to end position
    pub fn plan_trajectory(&self, start_pos: f32, end_pos: f32) -> Vec<TrajectoryPoint> {
        let distance = (end_pos - start_pos).abs();
        let direction = if end_pos > start_pos { 1.0 } else { -1.0 };
        
        // Calculate time to reach max velocity
        let t_accel = self.max_velocity / self.max_acceleration;
        let d_accel = 0.5 * self.max_acceleration * t_accel * t_accel;
        
        let mut trajectory = Vec::new();
        let mut time = 0.0;
        let mut position = start_pos;
        let mut velocity: f32 = 0.0;
        let mut acceleration = 0.0;
        
        // Phase 1: Acceleration
        while velocity.abs() < self.max_velocity && (end_pos - position).abs() > 0.1 {
            acceleration = self.max_acceleration * direction;
            velocity += acceleration * self.time_step;
            position += velocity * self.time_step;
            
            trajectory.push(TrajectoryPoint {
                position,
                velocity,
                acceleration,
                time,
            });
            
            time += self.time_step;
        }
        
        // Phase 2: Constant velocity
        while (end_pos - position).abs() > 0.1 {
            acceleration = 0.0;
            velocity = self.max_velocity * direction;
            position += velocity * self.time_step;
            
            trajectory.push(TrajectoryPoint {
                position,
                velocity,
                acceleration,
                time,
            });
            
            time += self.time_step;
        }
        
        // Phase 3: Deceleration
        while velocity.abs() > 0.01 {
            acceleration = -self.max_acceleration * direction;
            velocity += acceleration * self.time_step;
            position += velocity * self.time_step;
            
            trajectory.push(TrajectoryPoint {
                position,
                velocity,
                acceleration,
                time,
            });
            
            time += self.time_step;
        }
        
        // Final point
        trajectory.push(TrajectoryPoint {
            position: end_pos,
            velocity: 0.0,
            acceleration: 0.0,
            time,
        });
        
        trajectory
    }

    /// Generate a smooth circular trajectory
    pub fn plan_circular_trajectory(&self, center_x: f32, center_y: f32, radius: f32, 
                                   start_angle: f32, end_angle: f32) -> Vec<(f32, f32)> {
        let mut trajectory = Vec::new();
        let angle_step = 0.1; // 0.1 radians per step
        let mut angle = start_angle;
        
        while angle <= end_angle {
            let x = center_x + radius * angle.cos();
            let y = center_y + radius * angle.sin();
            trajectory.push((x, y));
            angle += angle_step;
        }
        
        trajectory
    }
}

/// 3D Point for collision detection
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Bounding box for collision detection
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min: Point3D,
    pub max: Point3D,
}

impl BoundingBox {
    /// Create a new bounding box
    pub fn new(min: Point3D, max: Point3D) -> Self {
        BoundingBox { min, max }
    }

    /// Check if this bounding box intersects with another
    pub fn intersects(&self, other: &BoundingBox) -> bool {
        self.min.x <= other.max.x && self.max.x >= other.min.x &&
        self.min.y <= other.max.y && self.max.y >= other.min.y &&
        self.min.z <= other.max.z && self.max.z >= other.min.z
    }

    /// Check if a point is inside this bounding box
    pub fn contains_point(&self, point: Point3D) -> bool {
        point.x >= self.min.x && point.x <= self.max.x &&
        point.y >= self.min.y && point.y <= self.max.y &&
        point.z >= self.min.z && point.z <= self.max.z
    }
}

/// Collision detection system
#[derive(Debug, Clone)]
pub struct CollisionDetector {
    pub obstacles: Vec<BoundingBox>,
    pub safety_margin: f32,
}

impl CollisionDetector {
    /// Create a new collision detector
    pub fn new(safety_margin: f32) -> Self {
        CollisionDetector {
            obstacles: Vec::new(),
            safety_margin,
        }
    }

    /// Add an obstacle to the collision detection system
    pub fn add_obstacle(&mut self, obstacle: BoundingBox) {
        self.obstacles.push(obstacle);
    }

    /// Check if a path from start to end is collision-free
    pub fn is_path_clear(&self, start: Point3D, end: Point3D) -> bool {
        // Simple line-segment collision detection
        let steps = 10; // Number of points to check along the path
        for i in 0..=steps {
            let t = i as f32 / steps as f32;
            let point = Point3D {
                x: start.x + t * (end.x - start.x),
                y: start.y + t * (end.y - start.y),
                z: start.z + t * (end.z - start.z),
            };
            
            if self.check_point_collision(point) {
                return false;
            }
        }
        true
    }

    /// Check if a specific point collides with any obstacle
    pub fn check_point_collision(&self, point: Point3D) -> bool {
        for obstacle in &self.obstacles {
            if obstacle.contains_point(point) {
                return true;
            }
        }
        false
    }

    /// Find a collision-free path using simple pathfinding
    pub fn find_path(&self, start: Point3D, end: Point3D) -> Option<Vec<Point3D>> {
        // Simple A* pathfinding implementation
        let mut path = Vec::new();
        let mut current = start;
        
        // Add start point
        path.push(current);
        
        // Simple straight-line path with obstacle avoidance
        let steps = 20;
        for i in 1..=steps {
            let t = i as f32 / steps as f32;
            let target = Point3D {
                x: start.x + t * (end.x - start.x),
                y: start.y + t * (end.y - start.y),
                z: start.z + t * (end.z - start.z),
            };
            
            // Check if we can move to the target
            if self.is_path_clear(current, target) {
                current = target;
                path.push(current);
            } else {
                // Try to find an alternative path around obstacles
                if let Some(alternative) = self.find_alternative_path(current, target) {
                    path.extend(alternative);
                    current = target;
                } else {
                    return None; // No path found
                }
            }
        }
        
        Some(path)
    }

    /// Find an alternative path around obstacles
    fn find_alternative_path(&self, start: Point3D, end: Point3D) -> Option<Vec<Point3D>> {
        // Simple obstacle avoidance by moving perpendicular to the line
        let dx = end.x - start.x;
        let dy = end.y - start.y;
        let dz = end.z - start.z;
        let length = (dx * dx + dy * dy + dz * dz).sqrt();
        
        if length == 0.0 {
            return None;
        }
        
        // Try moving around the obstacle
        let offset = self.safety_margin * 2.0;
        let perpendicular_x = -dy / length * offset;
        let perpendicular_y = dx / length * offset;
        
        let alternative = Point3D {
            x: start.x + perpendicular_x,
            y: start.y + perpendicular_y,
            z: start.z,
        };
        
        if self.is_path_clear(start, alternative) && self.is_path_clear(alternative, end) {
            Some(vec![alternative])
        } else {
            None
        }
    }
}

/// Robot arm joint configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointConfig {
    pub angle: f32, // Joint angle in radians
    pub length: f32, // Link length
    pub offset: f32, // Joint offset
}

/// Forward kinematics for robot arm
#[derive(Debug, Clone)]
pub struct ForwardKinematics {
    pub joints: Vec<JointConfig>,
}

impl ForwardKinematics {
    /// Create a new forward kinematics solver
    pub fn new(joints: Vec<JointConfig>) -> Self {
        ForwardKinematics { joints }
    }

    /// Calculate end-effector position from joint angles
    pub fn calculate_end_effector(&self) -> Point3D {
        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;
        let mut angle_sum = 0.0;
        
        for joint in &self.joints {
            angle_sum += joint.angle;
            x += joint.length * angle_sum.cos();
            y += joint.length * angle_sum.sin();
            z += joint.offset;
        }
        
        Point3D { x, y, z }
    }

    /// Calculate all joint positions
    pub fn calculate_all_positions(&self) -> Vec<Point3D> {
        let mut positions = Vec::new();
        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;
        let mut angle_sum = 0.0;
        
        // Add base position
        positions.push(Point3D { x, y, z });
        
        for joint in &self.joints {
            angle_sum += joint.angle;
            x += joint.length * angle_sum.cos();
            y += joint.length * angle_sum.sin();
            z += joint.offset;
            positions.push(Point3D { x, y, z });
        }
        
        positions
    }
}

/// Inverse kinematics for robot arm
#[derive(Debug, Clone)]
pub struct InverseKinematics {
    pub joint_configs: Vec<JointConfig>,
    pub max_iterations: u32,
    pub tolerance: f32,
}

impl InverseKinematics {
    /// Create a new inverse kinematics solver
    pub fn new(joint_configs: Vec<JointConfig>) -> Self {
        InverseKinematics {
            joint_configs,
            max_iterations: 100,
            tolerance: 0.01,
        }
    }

    /// Calculate joint angles to reach target position
    pub fn solve(&self, target: Point3D) -> Option<Vec<f32>> {
        let mut angles = vec![0.0; self.joint_configs.len()];
        
        for _ in 0..self.max_iterations {
            let fk = ForwardKinematics::new(
                self.joint_configs.iter()
                    .zip(angles.iter())
                    .map(|(config, angle)| JointConfig {
                        angle: *angle,
                        length: config.length,
                        offset: config.offset,
                    })
                    .collect()
            );
            
            let current_pos = fk.calculate_end_effector();
            let error = self.calculate_error(current_pos, target);
            
            if error < self.tolerance {
                return Some(angles);
            }
            
            // Simple gradient descent for inverse kinematics
            angles = self.update_angles(angles, current_pos, target);
        }
        
        None // Failed to converge
    }

    /// Calculate position error
    fn calculate_error(&self, current: Point3D, target: Point3D) -> f32 {
        let dx = current.x - target.x;
        let dy = current.y - target.y;
        let dz = current.z - target.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Update joint angles using gradient descent
    fn update_angles(&self, mut angles: Vec<f32>, current: Point3D, target: Point3D) -> Vec<f32> {
        let learning_rate = 0.01;
        
        for i in 0..angles.len() {
            let fk_plus = ForwardKinematics::new(
                self.joint_configs.iter()
                    .zip(angles.iter())
                    .enumerate()
                    .map(|(j, (config, angle))| JointConfig {
                        angle: if j == i { *angle + 0.01 } else { *angle },
                        length: config.length,
                        offset: config.offset,
                    })
                    .collect()
            );
            
            let pos_plus = fk_plus.calculate_end_effector();
            let error_plus = self.calculate_error(pos_plus, target);
            let error_current = self.calculate_error(current, target);
            
            let gradient = (error_plus - error_current) / 0.01;
            angles[i] -= learning_rate * gradient;
            
            // Keep angles in reasonable range
            angles[i] = angles[i].max(-3.14).min(3.14);
        }
        
        angles
    }
}

/// Advanced control system that combines all control algorithms
#[derive(Debug, Clone)]
pub struct AdvancedControlSystem {
    pub pid_controllers: HashMap<u8, PidController>,
    pub trajectory_planner: TrajectoryPlanner,
    pub collision_detector: CollisionDetector,
    pub forward_kinematics: ForwardKinematics,
    pub inverse_kinematics: InverseKinematics,
}

impl AdvancedControlSystem {
    /// Create a new advanced control system
    pub fn new() -> Self {
        AdvancedControlSystem {
            pid_controllers: HashMap::new(),
            trajectory_planner: TrajectoryPlanner::new(10.0, 5.0, 2.0),
            collision_detector: CollisionDetector::new(0.1),
            forward_kinematics: ForwardKinematics::new(Vec::new()),
            inverse_kinematics: InverseKinematics::new(Vec::new()),
        }
    }

    /// Add a PID controller for a motor
    pub fn add_pid_controller(&mut self, motor_id: u8, kp: f32, ki: f32, kd: f32) {
        self.pid_controllers.insert(motor_id, PidController::new(kp, ki, kd));
    }

    /// Set up robot arm kinematics
    pub fn setup_robot_arm(&mut self, joint_configs: Vec<JointConfig>) {
        self.forward_kinematics = ForwardKinematics::new(joint_configs.clone());
        self.inverse_kinematics = InverseKinematics::new(joint_configs);
    }

    /// Plan a smooth trajectory to target position
    pub fn plan_smooth_movement(&self, start_pos: f32, end_pos: f32) -> Vec<TrajectoryPoint> {
        self.trajectory_planner.plan_trajectory(start_pos, end_pos)
    }

    /// Check if a movement is collision-free
    pub fn check_collision_free(&self, start: Point3D, end: Point3D) -> bool {
        self.collision_detector.is_path_clear(start, end)
    }

    /// Calculate PID output for a motor
    pub fn calculate_pid_output(&mut self, motor_id: u8, current_value: f32) -> Option<f32> {
        if let Some(pid) = self.pid_controllers.get_mut(&motor_id) {
            Some(pid.update(current_value))
        } else {
            None
        }
    }

    /// Solve inverse kinematics for target position
    pub fn solve_inverse_kinematics(&self, target: Point3D) -> Option<Vec<f32>> {
        self.inverse_kinematics.solve(target)
    }

    /// Calculate forward kinematics for current joint angles
    pub fn calculate_forward_kinematics(&self, joint_angles: Vec<f32>) -> Point3D {
        let fk = ForwardKinematics::new(
            self.forward_kinematics.joints.iter()
                .zip(joint_angles.iter())
                .map(|(config, angle)| JointConfig {
                    angle: *angle,
                    length: config.length,
                    offset: config.offset,
                })
                .collect()
        );
        fk.calculate_end_effector()
    }
}

impl Default for AdvancedControlSystem {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_controller() {
        let mut pid = PidController::new(1.0, 0.1, 0.01);
        pid.set_setpoint(100.0);
        
        let output = pid.update(50.0);
        assert!(output > 0.0); // Should output positive value to reach setpoint
    }

    #[test]
    fn test_trajectory_planner() {
        let planner = TrajectoryPlanner::new(10.0, 5.0, 2.0);
        let trajectory = planner.plan_trajectory(0.0, 100.0);
        
        assert!(!trajectory.is_empty());
        assert_eq!(trajectory[0].position, 0.0);
        assert_eq!(trajectory.last().unwrap().position, 100.0);
    }

    #[test]
    fn test_collision_detection() {
        let mut detector = CollisionDetector::new(0.1);
        let obstacle = BoundingBox::new(
            Point3D { x: 1.0, y: 1.0, z: 1.0 },
            Point3D { x: 2.0, y: 2.0, z: 2.0 }
        );
        detector.add_obstacle(obstacle);
        
        let start = Point3D { x: 0.0, y: 0.0, z: 0.0 };
        let end = Point3D { x: 3.0, y: 3.0, z: 3.0 };
        
        assert!(detector.is_path_clear(start, end));
    }

    #[test]
    fn test_forward_kinematics() {
        let joints = vec![
            JointConfig { angle: 0.0, length: 1.0, offset: 0.0 },
            JointConfig { angle: 1.57, length: 1.0, offset: 0.0 }, // 90 degrees
        ];
        let fk = ForwardKinematics::new(joints);
        let pos = fk.calculate_end_effector();
        
        assert!((pos.x - 1.0).abs() < 0.01);
        assert!((pos.y - 1.0).abs() < 0.01);
    }
}
