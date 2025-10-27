//! Web Dashboard for EmbedCore Robot Visualization
//! 
//! This module provides a web-based interface for visualizing and controlling
//! the embedded robotics system in real-time.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use std::collections::VecDeque;
use tokio::sync::broadcast;
use warp::Filter;
use serde::{Deserialize, Serialize};
use futures_util::{SinkExt, StreamExt};

/// Robot joint information for visualization
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointInfo {
    pub id: u8,
    pub name: String,
    pub position: f32,
    pub target_position: f32,
    pub min_angle: f32,
    pub max_angle: f32,
    pub is_moving: bool,
    pub velocity: f32,
    pub acceleration: f32,
    pub power_consumption: f32,
}

/// Data point for logging and analytics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataPoint {
    pub timestamp: u64,
    pub joint_id: u8,
    pub position: f32,
    pub velocity: f32,
    pub acceleration: f32,
    pub power_consumption: f32,
    pub temperature: f32,
    pub security_events: u32,
    pub system_load: f32,
}

/// Performance metrics for analytics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceMetrics {
    pub avg_response_time: f32,
    pub max_velocity: f32,
    pub total_power_consumed: f32,
    pub efficiency_score: f32,
    pub movement_smoothness: f32,
    pub security_incidents: u32,
}

/// Analytics data for charts
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalyticsData {
    pub time_series: Vec<DataPoint>,
    pub performance_metrics: PerformanceMetrics,
    pub joint_efficiency: Vec<(String, f32)>,
    pub power_consumption_history: Vec<(u64, f32)>,
}

/// Robot state for the dashboard
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotState {
    pub timestamp: u64,
    pub joints: Vec<JointInfo>,
    pub system_status: String,
    pub security_events: u32,
    pub emergency_stop: bool,
    pub system_temperature: f32,
    pub system_load: f32,
    pub uptime: u64,
}

/// Dashboard configuration
#[derive(Debug, Clone)]
pub struct DashboardConfig {
    pub port: u16,
    pub update_interval_ms: u64,
    pub enable_controls: bool,
    pub enable_data_logging: bool,
    pub max_log_entries: usize,
    pub enable_analytics: bool,
}

impl Default for DashboardConfig {
    fn default() -> Self {
        DashboardConfig {
            port: 8080,
            update_interval_ms: 50, // 20 FPS
            enable_controls: true,
            enable_data_logging: true,
            max_log_entries: 10000,
            enable_analytics: true,
        }
    }
}

/// Web dashboard server
#[derive(Clone)]
pub struct WebDashboard {
    config: DashboardConfig,
    robot_state: Arc<Mutex<RobotState>>,
    state_sender: broadcast::Sender<RobotState>,
    is_running: Arc<Mutex<bool>>,
    data_log: Arc<Mutex<VecDeque<DataPoint>>>,
    performance_metrics: Arc<Mutex<PerformanceMetrics>>,
    start_time: Instant,
}

impl WebDashboard {
    /// Create a new web dashboard
    pub fn new(config: DashboardConfig) -> Self {
        let (state_sender, _) = broadcast::channel(100);
        
        let initial_state = RobotState {
            timestamp: 0,
            joints: vec![],
            system_status: "Initializing".to_string(),
            security_events: 0,
            emergency_stop: false,
            system_temperature: 25.0,
            system_load: 0.0,
            uptime: 0,
        };
        
        let initial_metrics = PerformanceMetrics {
            avg_response_time: 0.0,
            max_velocity: 0.0,
            total_power_consumed: 0.0,
            efficiency_score: 100.0,
            movement_smoothness: 100.0,
            security_incidents: 0,
        };
        
        WebDashboard {
            config,
            robot_state: Arc::new(Mutex::new(initial_state)),
            state_sender,
            is_running: Arc::new(Mutex::new(false)),
            data_log: Arc::new(Mutex::new(VecDeque::new())),
            performance_metrics: Arc::new(Mutex::new(initial_metrics)),
            start_time: Instant::now(),
        }
    }
    
    /// Start the web dashboard server
    pub async fn start(&self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let robot_state = Arc::clone(&self.robot_state);
        let state_sender = self.state_sender.clone();
        let is_running = Arc::clone(&self.is_running);
        let config = self.config.clone();
        
        // Mark as running
        *is_running.lock().unwrap() = true;
        
        // Start the state update task
        let _update_task = {
            let robot_state = Arc::clone(&robot_state);
            let state_sender = state_sender.clone();
            let is_running = Arc::clone(&is_running);
            
            tokio::spawn(async move {
                let mut last_update = Instant::now();
                
                while *is_running.lock().unwrap() {
                    let now = Instant::now();
                    if now.duration_since(last_update) >= Duration::from_millis(config.update_interval_ms) {
                        // Update robot state
                        let current_state = robot_state.lock().unwrap().clone();
                        let _ = state_sender.send(current_state);
                        last_update = now;
                    }
                    
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            })
        };
        
        // Create warp filters
        let robot_state = Arc::clone(&self.robot_state);
        let state_sender = self.state_sender.clone();
        
        // Static files
        let static_files = warp::path("static")
            .and(warp::fs::dir("web/static"));
        
        // API routes
        let api = warp::path("api");
        
        // Get current robot state
        let get_state = api
            .and(warp::path("state"))
            .and(warp::get())
            .and(with_robot_state(robot_state))
            .and_then(get_robot_state);
        
        // WebSocket for real-time updates
        let websocket = api
            .and(warp::path("ws"))
            .and(warp::ws())
            .and(with_state_sender(state_sender))
            .and_then(websocket_handler);
        
        // Update robot state (for testing)
        let update_state = api
            .and(warp::path("update"))
            .and(warp::post())
            .and(warp::body::json())
            .and(with_robot_state(Arc::clone(&self.robot_state)))
            .and_then(update_robot_state);
        
        // Analytics routes
        let analytics_data = api
            .and(warp::path("analytics"))
            .and(warp::get())
            .and(with_dashboard(Arc::new(self.clone())))
            .and_then(get_analytics_data);
        
        let export_csv = api
            .and(warp::path("export"))
            .and(warp::path("csv"))
            .and(warp::get())
            .and(with_dashboard(Arc::new(self.clone())))
            .and_then(export_csv_data);
        
        let export_json = api
            .and(warp::path("export"))
            .and(warp::path("json"))
            .and(warp::get())
            .and(with_dashboard(Arc::new(self.clone())))
            .and_then(export_json_data);
        
        // Serve the main dashboard
        let dashboard = warp::path::end()
            .and(warp::get())
            .map(|| {
                warp::reply::html(include_str!("../web/index.html"))
            });
        
        // Combine all routes
        let routes = dashboard
            .or(static_files)
            .or(get_state)
            .or(websocket)
            .or(update_state)
            .or(analytics_data)
            .or(export_csv)
            .or(export_json)
            .with(warp::cors()
                .allow_any_origin()
                .allow_headers(vec!["content-type"])
                .allow_methods(vec!["GET", "POST", "OPTIONS"]));
        
        println!("🌐 Web Dashboard starting on http://localhost:{}", self.config.port);
        println!("📊 Robot visualization available at http://localhost:{}", self.config.port);
        
        // Start the server
        warp::serve(routes)
            .run(([127, 0, 0, 1], self.config.port))
            .await;
        
        Ok(())
    }
    
    /// Update robot joint information
    pub fn update_joint(&self, joint_id: u8, name: String, position: f32, target: f32, min_angle: f32, max_angle: f32, is_moving: bool) {
        let mut state = self.robot_state.lock().unwrap();
        
        // Find existing joint or create new one
        if let Some(joint) = state.joints.iter_mut().find(|j| j.id == joint_id) {
            joint.position = position;
            joint.target_position = target;
            joint.is_moving = is_moving;
        } else {
            state.joints.push(JointInfo {
                id: joint_id,
                name,
                position,
                target_position: target,
                min_angle,
                max_angle,
                is_moving,
                velocity: 0.0,
                acceleration: 0.0,
                power_consumption: 0.0,
            });
        }
        
        state.timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
    }
    
    /// Update system status
    pub fn update_system_status(&self, status: String) {
        let mut state = self.robot_state.lock().unwrap();
        state.system_status = status;
    }
    
    /// Update security events count
    pub fn update_security_events(&self, count: u32) {
        let mut state = self.robot_state.lock().unwrap();
        state.security_events = count;
    }
    
    /// Set emergency stop status
    pub fn set_emergency_stop(&self, emergency: bool) {
        let mut state = self.robot_state.lock().unwrap();
        state.emergency_stop = emergency;
    }
    
    /// Stop the dashboard
    pub fn stop(&self) {
        *self.is_running.lock().unwrap() = false;
    }
    
    /// Log a data point for analytics
    pub fn log_data_point(&self, joint_id: u8, position: f32, velocity: f32, acceleration: f32, power_consumption: f32) {
        if !self.config.enable_data_logging {
            return;
        }
        
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        
        let state = self.robot_state.lock().unwrap();
        let data_point = DataPoint {
            timestamp,
            joint_id,
            position,
            velocity,
            acceleration,
            power_consumption,
            temperature: state.system_temperature,
            security_events: state.security_events,
            system_load: state.system_load,
        };
        
        let mut log = self.data_log.lock().unwrap();
        log.push_back(data_point);
        
        // Maintain max log size
        while log.len() > self.config.max_log_entries {
            log.pop_front();
        }
    }
    
    /// Get analytics data for charts
    pub fn get_analytics_data(&self) -> AnalyticsData {
        let log = self.data_log.lock().unwrap();
        let metrics = self.performance_metrics.lock().unwrap();
        let state = self.robot_state.lock().unwrap();
        
        // Calculate joint efficiency
        let joint_efficiency: Vec<(String, f32)> = state.joints.iter()
            .map(|joint| {
                let efficiency = if joint.power_consumption > 0.0 {
                    (joint.velocity.abs() / joint.power_consumption * 100.0).min(100.0)
                } else {
                    100.0
                };
                (joint.name.clone(), efficiency)
            })
            .collect();
        
        // Calculate power consumption history (last 100 points)
        let power_history: Vec<(u64, f32)> = log.iter()
            .rev()
            .take(100)
            .map(|dp| (dp.timestamp, dp.power_consumption))
            .collect();
        
        AnalyticsData {
            time_series: log.iter().cloned().collect(),
            performance_metrics: metrics.clone(),
            joint_efficiency,
            power_consumption_history: power_history,
        }
    }
    
    /// Export data as CSV
    pub fn export_csv(&self) -> String {
        let log = self.data_log.lock().unwrap();
        let mut csv = String::from("timestamp,joint_id,position,velocity,acceleration,power_consumption,temperature,security_events,system_load\n");
        
        for data_point in log.iter() {
            csv.push_str(&format!(
                "{},{},{:.2},{:.2},{:.2},{:.2},{:.2},{},{:.2}\n",
                data_point.timestamp,
                data_point.joint_id,
                data_point.position,
                data_point.velocity,
                data_point.acceleration,
                data_point.power_consumption,
                data_point.temperature,
                data_point.security_events,
                data_point.system_load
            ));
        }
        
        csv
    }
    
    /// Export data as JSON
    pub fn export_json(&self) -> String {
        let analytics = self.get_analytics_data();
        serde_json::to_string_pretty(&analytics).unwrap_or_else(|_| "{}".to_string())
    }
    
    /// Update performance metrics
    pub fn update_performance_metrics(&self) {
        let log = self.data_log.lock().unwrap();
        let mut metrics = self.performance_metrics.lock().unwrap();
        
        if log.is_empty() {
            return;
        }
        
        // Calculate average response time (simplified)
        let mut total_velocity = 0.0;
        let mut max_vel: f32 = 0.0;
        let mut total_power = 0.0;
        let mut smoothness_sum = 0.0;
        let mut smoothness_count = 0;
        
        for data_point in log.iter() {
            total_velocity += data_point.velocity.abs();
            max_vel = max_vel.max(data_point.velocity.abs());
            total_power += data_point.power_consumption;
            
            // Calculate smoothness (inverse of acceleration variance)
            if data_point.acceleration.abs() < 10.0 { // Reasonable threshold
                smoothness_sum += 1.0;
            }
            smoothness_count += 1;
        }
        
        let count = log.len() as f32;
        metrics.avg_response_time = if count > 0.0 { total_velocity / count } else { 0.0 };
        metrics.max_velocity = max_vel;
        metrics.total_power_consumed = total_power;
        metrics.efficiency_score = if total_power > 0.0 { (total_velocity / total_power * 100.0).min(100.0) } else { 100.0 };
        metrics.movement_smoothness = if smoothness_count > 0 { (smoothness_sum / smoothness_count as f32) * 100.0 } else { 100.0 };
    }
}

// Helper functions for warp filters
fn with_robot_state(
    robot_state: Arc<Mutex<RobotState>>,
) -> impl Filter<Extract = (Arc<Mutex<RobotState>>,), Error = std::convert::Infallible> + Clone {
    warp::any().map(move || Arc::clone(&robot_state))
}

fn with_state_sender(
    sender: broadcast::Sender<RobotState>,
) -> impl Filter<Extract = (broadcast::Sender<RobotState>,), Error = std::convert::Infallible> + Clone {
    warp::any().map(move || sender.clone())
}

fn with_dashboard(
    dashboard: Arc<WebDashboard>,
) -> impl Filter<Extract = (Arc<WebDashboard>,), Error = std::convert::Infallible> + Clone {
    warp::any().map(move || Arc::clone(&dashboard))
}

// API handlers
async fn get_robot_state(
    robot_state: Arc<Mutex<RobotState>>,
) -> Result<impl warp::Reply, warp::Rejection> {
    let state = robot_state.lock().unwrap().clone();
    Ok(warp::reply::json(&state))
}

async fn update_robot_state(
    update: RobotState,
    robot_state: Arc<Mutex<RobotState>>,
) -> Result<impl warp::Reply, warp::Rejection> {
    *robot_state.lock().unwrap() = update;
    Ok(warp::reply::json(&serde_json::json!({"status": "ok"})))
}

async fn websocket_handler(
    ws: warp::ws::Ws,
    state_sender: broadcast::Sender<RobotState>,
) -> Result<impl warp::Reply, warp::Rejection> {
    Ok(ws.on_upgrade(move |websocket| {
        handle_websocket(websocket, state_sender)
    }))
}

async fn handle_websocket(
    websocket: warp::ws::WebSocket,
    state_sender: broadcast::Sender<RobotState>,
) {
    let (mut ws_sender, _ws_receiver) = websocket.split();
    let mut state_receiver = state_sender.subscribe();
    
    while let Ok(state) = state_receiver.recv().await {
        let message = warp::ws::Message::text(serde_json::to_string(&state).unwrap());
        if ws_sender.send(message).await.is_err() {
            break;
        }
    }
}

// New API handlers for analytics and data export
async fn get_analytics_data(
    dashboard: Arc<WebDashboard>,
) -> Result<impl warp::Reply, warp::Rejection> {
    let analytics = dashboard.get_analytics_data();
    Ok(warp::reply::json(&analytics))
}

async fn export_csv_data(
    dashboard: Arc<WebDashboard>,
) -> Result<impl warp::Reply, warp::Rejection> {
    let csv_data = dashboard.export_csv();
        Ok(warp::reply::with_header(
            warp::reply::with_header(
                csv_data,
                "Content-Type",
                "text/csv",
            ),
            "Content-Disposition",
            "attachment; filename=\"robot_data.csv\"",
        ))
}

async fn export_json_data(
    dashboard: Arc<WebDashboard>,
) -> Result<impl warp::Reply, warp::Rejection> {
    let json_data = dashboard.export_json();
        Ok(warp::reply::with_header(
            warp::reply::with_header(
                json_data,
                "Content-Type",
                "application/json",
            ),
            "Content-Disposition",
            "attachment; filename=\"robot_analytics.json\"",
        ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dashboard_creation() {
        let config = DashboardConfig::default();
        let dashboard = WebDashboard::new(config);
        assert!(!*dashboard.is_running.lock().unwrap());
    }

    #[test]
    fn test_joint_update() {
        let config = DashboardConfig::default();
        let dashboard = WebDashboard::new(config);
        
        dashboard.update_joint(0, "Base".to_string(), 90.0, 90.0, 0.0, 180.0, false);
        
        let state = dashboard.robot_state.lock().unwrap();
        assert_eq!(state.joints.len(), 1);
        assert_eq!(state.joints[0].name, "Base");
        assert_eq!(state.joints[0].position, 90.0);
    }
}
