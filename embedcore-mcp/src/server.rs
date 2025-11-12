//! MCP Server implementation for EmbedCore

use crate::protocol::*;
use crate::tools::*;
use crate::resources::ResourceHandler;
use serde_json::json;
use std::io::{self, BufRead, BufReader, Write};

/// EmbedCore MCP Server
pub struct EmbedCoreMcpServer {
    initialized: bool,
}

impl EmbedCoreMcpServer {
    pub fn new() -> Self {
        Self {
            initialized: false,
        }
    }

    /// Run the MCP server (reads from stdin, writes to stdout)
    pub fn run(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let stdin = io::stdin();
        let mut reader = BufReader::new(stdin.lock());
        let mut stdout = io::stdout();

        loop {
            let mut line = String::new();
            let bytes_read = reader.read_line(&mut line)?;
            
            if bytes_read == 0 {
                break; // EOF
            }

            let trimmed = line.trim();
            if trimmed.is_empty() {
                continue;
            }

            match self.handle_request(trimmed) {
                Ok(response) => {
                    let response_json = serde_json::to_string(&response)?;
                    writeln!(stdout, "{}", response_json)?;
                    stdout.flush()?;
                }
                Err(e) => {
                    eprintln!("Error handling request: {}", e);
                }
            }
        }

        Ok(())
    }

    fn handle_request(&mut self, request_str: &str) -> Result<JsonRpcResponse, Box<dyn std::error::Error>> {
        let request: JsonRpcRequest = serde_json::from_str(request_str)?;
        let id = request.id.clone();

        let result = match request.method.as_str() {
            "initialize" => {
                self.initialized = true;
                Ok(json!({
                    "protocolVersion": "2024-11-05",
                    "capabilities": {
                        "tools": {},
                        "resources": {}
                    },
                    "serverInfo": {
                        "name": "embedcore-mcp",
                        "version": "0.1.0"
                    }
                }))
            }
            "tools/list" => {
                Ok(json!({
                    "tools": self.list_tools()
                }))
            }
            "tools/call" => {
                self.handle_tool_call(request.params)
            }
            "resources/list" => {
                Ok(json!({
                    "resources": ResourceHandler::list_resources()
                }))
            }
            "resources/read" => {
                self.handle_resource_read(request.params)
            }
            _ => {
                Err(format!("Unknown method: {}", request.method))
            }
        };

        match result {
            Ok(value) => Ok(JsonRpcResponse::success(id, value)),
            Err(e) => Ok(JsonRpcResponse::error(
                id,
                -32603,
                format!("Internal error: {}", e),
            )),
        }
    }

    fn list_tools(&self) -> Vec<serde_json::Value> {
        vec![
            json!({
                "name": "gpio_control",
                "description": "Control GPIO pins - read, write, toggle, or configure",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "pin": {"type": "number", "description": "GPIO pin number (0-255)"},
                        "action": {"type": "string", "enum": ["read", "write", "toggle", "configure"], "description": "Action to perform"},
                        "mode": {"type": "string", "enum": ["input", "output", "pullup", "pulldown"], "description": "Pin mode (for configure)"},
                        "state": {"type": "string", "enum": ["high", "low"], "description": "Pin state (for write)"}
                    },
                    "required": ["pin", "action"]
                }
            }),
            json!({
                "name": "pwm_control",
                "description": "Control PWM channels - configure, set duty cycle, set angle (for servos), enable/disable",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "channel": {"type": "number", "description": "PWM channel number"},
                        "action": {"type": "string", "enum": ["configure", "set_duty", "set_angle", "enable", "disable"], "description": "Action to perform"},
                        "frequency_hz": {"type": "number", "description": "PWM frequency in Hz"},
                        "duty_cycle_percent": {"type": "number", "description": "Duty cycle percentage (0-100)"},
                        "angle_degrees": {"type": "number", "description": "Angle in degrees (0-180) for servo control"}
                    },
                    "required": ["channel", "action"]
                }
            }),
            json!({
                "name": "motor_control",
                "description": "Control motors - configure, set speed, set position (servos), set direction, stop, emergency stop",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "motor_id": {"type": "number", "description": "Motor ID (0-255)"},
                        "action": {"type": "string", "enum": ["configure", "set_speed", "set_position", "set_direction", "stop", "emergency_stop"], "description": "Action to perform"},
                        "motor_type": {"type": "string", "enum": ["dc", "servo", "stepper"], "description": "Motor type (for configure)"},
                        "pwm_channel": {"type": "number", "description": "PWM channel (for configure)"},
                        "speed": {"type": "number", "description": "Speed percentage (0-100)"},
                        "position": {"type": "number", "description": "Position in degrees (0-180) for servo"},
                        "direction": {"type": "string", "enum": ["forward", "reverse", "stop"], "description": "Motor direction"}
                    },
                    "required": ["motor_id", "action"]
                }
            }),
            json!({
                "name": "uart_control",
                "description": "Control UART ports - configure, send data, check for available data",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "port": {"type": "number", "description": "UART port number"},
                        "action": {"type": "string", "enum": ["configure", "send", "check_available"], "description": "Action to perform"},
                        "baud_rate": {"type": "number", "description": "Baud rate (for configure)"},
                        "data": {"type": "string", "description": "Data to send (for send)"}
                    },
                    "required": ["port", "action"]
                }
            }),
            json!({
                "name": "system_status",
                "description": "Get system status - initialize, get time, get all status",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "enum": ["init", "init_with_security", "get_time", "get_all_status"], "description": "Action to perform"}
                    },
                    "required": ["action"]
                }
            }),
        ]
    }

    fn handle_tool_call(&self, params: Option<serde_json::Value>) -> Result<serde_json::Value, String> {
        let params = params.ok_or("Missing parameters")?;
        let tool_name = params.get("name")
            .and_then(|v| v.as_str())
            .ok_or("Missing tool name")?;

        let arguments = params.get("arguments")
            .ok_or("Missing arguments")?;

        match tool_name {
            "gpio_control" => {
                let input: GpioControlInput = serde_json::from_value(arguments.clone())
                    .map_err(|e| format!("Invalid GPIO control input: {}", e))?;
                let result = ToolHandler::handle_gpio_control(input)?;
                Ok(json!({
                    "content": [{
                        "type": "text",
                        "text": serde_json::to_string(&result).unwrap_or_default()
                    }]
                }))
            }
            "pwm_control" => {
                let input: PwmControlInput = serde_json::from_value(arguments.clone())
                    .map_err(|e| format!("Invalid PWM control input: {}", e))?;
                let result = ToolHandler::handle_pwm_control(input)?;
                Ok(json!({
                    "content": [{
                        "type": "text",
                        "text": serde_json::to_string(&result).unwrap_or_default()
                    }]
                }))
            }
            "motor_control" => {
                let input: MotorControlInput = serde_json::from_value(arguments.clone())
                    .map_err(|e| format!("Invalid motor control input: {}", e))?;
                let result = ToolHandler::handle_motor_control(input)?;
                Ok(json!({
                    "content": [{
                        "type": "text",
                        "text": serde_json::to_string(&result).unwrap_or_default()
                    }]
                }))
            }
            "uart_control" => {
                let input: UartControlInput = serde_json::from_value(arguments.clone())
                    .map_err(|e| format!("Invalid UART control input: {}", e))?;
                let result = ToolHandler::handle_uart_control(input)?;
                Ok(json!({
                    "content": [{
                        "type": "text",
                        "text": serde_json::to_string(&result).unwrap_or_default()
                    }]
                }))
            }
            "system_status" => {
                let input: SystemStatusInput = serde_json::from_value(arguments.clone())
                    .map_err(|e| format!("Invalid system status input: {}", e))?;
                let result = ToolHandler::handle_system_status(input)?;
                Ok(json!({
                    "content": [{
                        "type": "text",
                        "text": serde_json::to_string(&result).unwrap_or_default()
                    }]
                }))
            }
            _ => Err(format!("Unknown tool: {}", tool_name)),
        }
    }

    fn handle_resource_read(&self, params: Option<serde_json::Value>) -> Result<serde_json::Value, String> {
        let params = params.ok_or("Missing parameters")?;
        let uri = params.get("uri")
            .and_then(|v| v.as_str())
            .ok_or("Missing URI")?;

        let content = ResourceHandler::read_resource(uri)?;
        Ok(json!({
            "contents": [{
                "uri": uri,
                "mimeType": "text/plain",
                "text": content
            }]
        }))
    }
}

impl Default for EmbedCoreMcpServer {
    fn default() -> Self {
        Self::new()
    }
}

