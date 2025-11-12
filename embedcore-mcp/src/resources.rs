//! MCP Resources for EmbedCore documentation and examples

use serde_json::json;
use std::fs;
use std::path::PathBuf;

/// Resource handler for EmbedCore documentation and examples
pub struct ResourceHandler;

impl ResourceHandler {
    /// Get available resources
    pub fn list_resources() -> Vec<serde_json::Value> {
        vec![
            json!({
                "uri": "embedcore://readme",
                "name": "EmbedCore README",
                "description": "Main project documentation",
                "mimeType": "text/markdown"
            }),
            json!({
                "uri": "embedcore://code-context",
                "name": "Code Context",
                "description": "Project architecture and component overview",
                "mimeType": "text/markdown"
            }),
            json!({
                "uri": "embedcore://example/blinky",
                "name": "Blinky Example",
                "description": "Simple GPIO LED blinking example",
                "mimeType": "text/rust"
            }),
            json!({
                "uri": "embedcore://example/servo-sweep",
                "name": "Servo Sweep Example",
                "description": "Servo motor control with angle sweeping",
                "mimeType": "text/rust"
            }),
            json!({
                "uri": "embedcore://example/robot-arm",
                "name": "Robot Arm Example",
                "description": "Multi-motor coordination example",
                "mimeType": "text/rust"
            }),
            json!({
                "uri": "embedcore://example/pid-control",
                "name": "PID Control Example",
                "description": "PID controller demonstration",
                "mimeType": "text/rust"
            }),
            json!({
                "uri": "embedcore://example/security-demo",
                "name": "Security Demo Example",
                "description": "Security monitoring demonstration",
                "mimeType": "text/rust"
            }),
        ]
    }

    /// Read a resource by URI
    pub fn read_resource(uri: &str) -> Result<String, String> {
        let project_root = std::env::var("EMBEDCORE_ROOT")
            .ok()
            .map(PathBuf::from)
            .or_else(|| {
                // Try to find project root by looking for Cargo.toml
                // Start from current directory
                let mut path = std::env::current_dir().ok()?;
                loop {
                    if path.join("Cargo.toml").exists() && path.join("embedcore").exists() {
                        return Some(path);
                    }
                    let parent = path.parent()?;
                    if parent == &path {
                        break; // Reached root
                    }
                    path = parent.to_path_buf();
                }
                None
            })
            .ok_or_else(|| "Could not determine project root. Set EMBEDCORE_ROOT environment variable or run from project root".to_string())?;

        match uri {
            "embedcore://readme" => {
                let path = project_root.join("README.md");
                fs::read_to_string(&path)
                    .map_err(|e| format!("Failed to read README: {}", e))
            }
            "embedcore://code-context" => {
                let path = project_root.join("embedcore").join("CODE_CONTEXT.md");
                fs::read_to_string(&path)
                    .map_err(|e| format!("Failed to read CODE_CONTEXT: {}", e))
            }
            uri if uri.starts_with("embedcore://example/") => {
                let example_name = uri.strip_prefix("embedcore://example/")
                    .ok_or_else(|| "Invalid example URI".to_string())?;
                let example_file = match example_name {
                    "blinky" => "blinky.rs",
                    "servo-sweep" => "servo_sweep.rs",
                    "robot-arm" => "robot_arm.rs",
                    "pid-control" => "pid_control_demo.rs",
                    "security-demo" => "security_demo.rs",
                    _ => return Err(format!("Unknown example: {}", example_name)),
                };
                let path = project_root.join("embedcore").join("examples").join(example_file);
                fs::read_to_string(&path)
                    .map_err(|e| format!("Failed to read example: {}", e))
            }
            _ => Err(format!("Unknown resource URI: {}", uri)),
        }
    }
}

