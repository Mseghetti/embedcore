# EmbedCore MCP Server

A Model Context Protocol (MCP) server for the EmbedCore embedded systems framework. This server enables AI assistants to interact with embedded devices, control motors and servos, manage GPIO pins, and access project documentation and examples.

## Features

### Tools

The MCP server provides the following tools for controlling embedded devices:

1. **GPIO Control** (`gpio_control`)
   - Read, write, toggle, and configure GPIO pins
   - Support for input, output, pullup, and pulldown modes

2. **PWM Control** (`pwm_control`)
   - Configure PWM channels
   - Set duty cycle and frequency
   - Control servo angles (0-180 degrees)

3. **Motor Control** (`motor_control`)
   - Configure DC, servo, and stepper motors
   - Set speed, position, and direction
   - Emergency stop functionality

4. **UART Control** (`uart_control`)
   - Configure UART ports
   - Send and receive data
   - Check for available data

5. **System Status** (`system_status`)
   - Initialize the EmbedCore system
   - Get system time
   - Initialize with security monitoring

### Resources

The server provides access to:

- **Documentation**: README and code context
- **Examples**: Pre-built code examples including:
  - Blinky (GPIO LED control)
  - Servo Sweep (servo motor control)
  - Robot Arm (multi-motor coordination)
  - PID Control (control system demo)
  - Security Demo (security monitoring)

## Installation

Build the MCP server from the project root:

```bash
cargo build --release -p embedcore-mcp
```

The binary will be located at `target/release/embedcore-mcp`.

## Usage

### Running the Server

The MCP server communicates via JSON-RPC over stdio:

```bash
./target/release/embedcore-mcp
```

### Configuration

For MCP clients (like Claude Desktop), add the server to your configuration:

**Claude Desktop** (`~/Library/Application Support/Claude/claude_desktop_config.json` on macOS):

```json
{
  "mcpServers": {
    "embedcore": {
      "command": "/path/to/embedcore/target/release/embedcore-mcp",
      "env": {
        "EMBEDCORE_ROOT": "/path/to/embedcore"
      }
    }
  }
}
```

**Other MCP Clients**: Configure according to your client's documentation, pointing to the `embedcore-mcp` binary.

## Environment Variables

- `EMBEDCORE_ROOT`: Optional. Path to the EmbedCore project root. If not set, the server will attempt to find it automatically by searching for `Cargo.toml` and the `embedcore` directory.

## Example Tool Usage

### GPIO Control

```json
{
  "name": "gpio_control",
  "arguments": {
    "pin": 13,
    "action": "write",
    "state": "high"
  }
}
```

### Motor Control

```json
{
  "name": "motor_control",
  "arguments": {
    "motor_id": 0,
    "action": "configure",
    "motor_type": "servo",
    "pwm_channel": 0
  }
}
```

### Servo Position Control

```json
{
  "name": "motor_control",
  "arguments": {
    "motor_id": 0,
    "action": "set_position",
    "position": 90.0
  }
}
```

## Protocol

The server implements the Model Context Protocol specification, communicating via JSON-RPC 2.0 over stdio. It supports:

- `initialize` - Initialize the MCP connection
- `tools/list` - List available tools
- `tools/call` - Execute a tool
- `resources/list` - List available resources
- `resources/read` - Read a resource by URI

## Development

### Project Structure

```
embedcore-mcp/
├── Cargo.toml
├── README.md
└── src/
    ├── main.rs          # Entry point
    ├── lib.rs           # Library exports
    ├── server.rs        # MCP server implementation
    ├── protocol.rs      # MCP protocol types
    ├── tools.rs         # Tool handlers
    └── resources.rs     # Resource handlers
```

### Adding New Tools

1. Define the input struct in `src/tools.rs`
2. Implement the handler function
3. Add the tool to `list_tools()` in `src/server.rs`
4. Add the handler case in `handle_tool_call()`

### Adding New Resources

1. Add the resource to `list_resources()` in `src/resources.rs`
2. Add the read handler in `read_resource()`

## License

MIT License - see the main project LICENSE file.

