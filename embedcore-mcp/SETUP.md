# EmbedCore MCP Server Setup for Claude Desktop

This guide will help you set up the EmbedCore MCP server with Claude Desktop on macOS.

## Prerequisites

1. **Rust installed** - If you don't have Rust, install it:
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   source ~/.cargo/env
   ```

2. **Claude Desktop installed** - Make sure you have Claude Desktop installed

## Quick Setup (Automated)

Run the setup script:

```bash
cd /Users/micah/embedcore/embedcore-mcp
./setup-mcp.sh
```

Then restart Claude Desktop.

## Manual Setup

### Step 1: Build the MCP Server

```bash
cd /Users/micah/embedcore
cargo build --release -p embedcore-mcp
```

This will create the binary at: `/Users/micah/embedcore/target/release/embedcore-mcp`

### Step 2: Create Claude Desktop Config

Create or edit the Claude Desktop configuration file:

**Location:** `~/Library/Application Support/Claude/claude_desktop_config.json`

**Content:**

```json
{
  "mcpServers": {
    "embedcore": {
      "command": "/Users/micah/embedcore/target/release/embedcore-mcp",
      "env": {
        "EMBEDCORE_ROOT": "/Users/micah/embedcore"
      }
    }
  }
}
```

**If you already have other MCP servers configured**, add `"embedcore"` to the existing `mcpServers` object:

```json
{
  "mcpServers": {
    "existing-server": { ... },
    "embedcore": {
      "command": "/Users/micah/embedcore/target/release/embedcore-mcp",
      "env": {
        "EMBEDCORE_ROOT": "/Users/micah/embedcore"
      }
    }
  }
}
```

### Step 3: Restart Claude Desktop

1. Quit Claude Desktop completely
2. Reopen Claude Desktop
3. The EmbedCore MCP server should now be available

## Verifying the Setup

1. Open Claude Desktop
2. Start a new conversation
3. You should see EmbedCore tools available, such as:
   - `gpio_control` - Control GPIO pins
   - `pwm_control` - Control PWM channels
   - `motor_control` - Control motors
   - `uart_control` - Control UART ports
   - `system_status` - Get system status

## Testing the Server Manually

You can test the MCP server directly:

```bash
/Users/micah/embedcore/target/release/embedcore-mcp
```

It communicates via JSON-RPC over stdio, so it's designed to be used by MCP clients, not directly.

## Troubleshooting

### "Command not found: cargo"
- Install Rust: `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
- Then run: `source ~/.cargo/env`

### "Binary not found"
- Make sure you built the project: `cargo build --release -p embedcore-mcp`
- Check the binary exists: `ls -la /Users/micah/embedcore/target/release/embedcore-mcp`

### MCP server not showing in Claude Desktop
- Make sure you restarted Claude Desktop completely
- Check the config file syntax is valid JSON
- Check Claude Desktop logs for errors

### Permission denied
- Make the binary executable: `chmod +x /Users/micah/embedcore/target/release/embedcore-mcp`

## Available Tools

Once set up, you can use these tools in Claude Desktop:

- **GPIO Control**: Read/write/toggle GPIO pins
- **PWM Control**: Configure PWM channels, set duty cycles, control servos
- **Motor Control**: Configure and control DC motors, servos, and steppers
- **UART Control**: Configure and use UART serial communication
- **System Status**: Initialize system, get time, monitor status

## Resources

The MCP server also provides access to:
- Project documentation (README.md)
- Code examples (blinky, servo_sweep, robot_arm, etc.)
- Code context and architecture information

