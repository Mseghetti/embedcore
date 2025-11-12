//! EmbedCore MCP Server
//! 
//! This module provides a Model Context Protocol (MCP) server for the EmbedCore
//! embedded systems framework, enabling AI assistants to interact with embedded
//! devices, control systems, and access documentation.

pub mod server;
pub mod tools;
pub mod resources;
pub mod protocol;

pub use server::EmbedCoreMcpServer;

