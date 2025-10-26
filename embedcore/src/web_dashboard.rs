//! Web Dashboard for EmbedCore Robot Visualization
//! 
//! This module provides a web-based interface for visualizing and controlling
//! the embedded robotics system in real-time.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
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
}

/// Robot state for the dashboard
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotState {
    pub timestamp: u64,
    pub joints: Vec<JointInfo>,
    pub system_status: String,
    pub security_events: u32,
    pub emergency_stop: bool,
}

/// Dashboard configuration
#[derive(Debug, Clone)]
pub struct DashboardConfig {
    pub port: u16,
    pub update_interval_ms: u64,
    pub enable_controls: bool,
}

impl Default for DashboardConfig {
    fn default() -> Self {
        DashboardConfig {
            port: 8080,
            update_interval_ms: 50, // 20 FPS
            enable_controls: true,
        }
    }
}

/// Web dashboard server
pub struct WebDashboard {
    config: DashboardConfig,
    robot_state: Arc<Mutex<RobotState>>,
    state_sender: broadcast::Sender<RobotState>,
    is_running: Arc<Mutex<bool>>,
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
        };
        
        WebDashboard {
            config,
            robot_state: Arc::new(Mutex::new(initial_state)),
            state_sender,
            is_running: Arc::new(Mutex::new(false)),
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
