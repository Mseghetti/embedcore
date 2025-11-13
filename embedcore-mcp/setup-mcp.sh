#!/bin/bash
# Setup script for EmbedCore MCP Server with Claude Desktop

set -e

PROJECT_ROOT="/Users/micah/embedcore"
CLAUDE_CONFIG_DIR="$HOME/Library/Application Support/Claude"
CLAUDE_CONFIG_FILE="$CLAUDE_CONFIG_DIR/claude_desktop_config.json"
MCP_BINARY="$PROJECT_ROOT/target/release/embedcore-mcp"

echo "🔧 Setting up EmbedCore MCP Server for Claude Desktop"
echo ""

# Check if Rust is installed
if ! command -v cargo &> /dev/null; then
    echo "❌ Rust/Cargo not found in PATH"
    echo ""
    echo "Please install Rust first:"
    echo "  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
    echo ""
    echo "Then run this script again."
    exit 1
fi

echo "✅ Rust/Cargo found"
echo ""

# Build the MCP server
echo "📦 Building MCP server..."
cd "$PROJECT_ROOT"
cargo build --release -p embedcore-mcp

if [ ! -f "$MCP_BINARY" ]; then
    echo "❌ Failed to build MCP server binary"
    exit 1
fi

echo "✅ MCP server built successfully"
echo ""

# Make binary executable
chmod +x "$MCP_BINARY"
echo "✅ Binary is executable"
echo ""

# Create Claude config directory if it doesn't exist
mkdir -p "$CLAUDE_CONFIG_DIR"

# Check if config file exists
if [ -f "$CLAUDE_CONFIG_FILE" ]; then
    echo "⚠️  Config file already exists: $CLAUDE_CONFIG_FILE"
    echo ""
    echo "Current contents:"
    cat "$CLAUDE_CONFIG_FILE"
    echo ""
    echo ""
    read -p "Do you want to merge EmbedCore config? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        # Merge with existing config
        python3 << EOF
import json
import sys

# Read existing config
with open("$CLAUDE_CONFIG_FILE", "r") as f:
    config = json.load(f)

# Add embedcore server
if "mcpServers" not in config:
    config["mcpServers"] = {}

config["mcpServers"]["embedcore"] = {
    "command": "$MCP_BINARY",
    "env": {
        "EMBEDCORE_ROOT": "$PROJECT_ROOT"
    }
}

# Write back
with open("$CLAUDE_CONFIG_FILE", "w") as f:
    json.dump(config, f, indent=2)

print("✅ Config merged successfully")
EOF
    else
        echo "Skipping config update. You can manually add:"
        echo ""
        cat "$PROJECT_ROOT/embedcore-mcp/claude_desktop_config.json"
        echo ""
    fi
else
    # Create new config file
    cat > "$CLAUDE_CONFIG_FILE" << EOF
{
  "mcpServers": {
    "embedcore": {
      "command": "$MCP_BINARY",
      "env": {
        "EMBEDCORE_ROOT": "$PROJECT_ROOT"
      }
    }
  }
}
EOF
    echo "✅ Created config file: $CLAUDE_CONFIG_FILE"
fi

echo ""
echo "🎉 Setup complete!"
echo ""
echo "Next steps:"
echo "1. Restart Claude Desktop"
echo "2. The EmbedCore MCP server should now be available"
echo ""
echo "To test the server manually:"
echo "  $MCP_BINARY"
echo ""

