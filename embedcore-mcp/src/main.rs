//! EmbedCore MCP Server entry point

use embedcore_mcp::EmbedCoreMcpServer;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut server = EmbedCoreMcpServer::new();
    server.run()?;
    Ok(())
}

