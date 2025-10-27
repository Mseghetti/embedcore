# EmbedCore Code Context

## Project Overview
EmbedCore is a simulated embedded systems middleware framework combining C++ hardware abstraction with Rust application development for robotics applications.

## Architecture
- **C++ HAL**: Hardware abstraction layer in `cpp_hal/`
- **Rust FFI**: Foreign function interface in `embedcore-sys/`
- **Safe Abstractions**: High-level Rust API in `embedcore/`
- **Web Dashboard**: Real-time visualization in `embedcore/src/web_dashboard.rs`

## Key Components

### Hardware Abstraction (C++)
- `cpp_hal/include/` - Header files for GPIO, PWM, UART, Timer, Motor
- `cpp_hal/src/` - Implementation files with simulation logic
- Provides low-level hardware control with safety checks

### Rust API (`embedcore/src/`)
- `devices/` - Safe wrappers for hardware components
- `security/` - Attack simulation and monitoring system
- `scheduler/` - Task scheduling for real-time operations
- `web_dashboard/` - WebSocket server for real-time visualization

### Examples
- `blinky.rs` - Basic GPIO LED control
- `servo_sweep.rs` - Servo motor control
- `robot_arm.rs` - Multi-motor coordination
- `visual_robot_arm.rs` - Web dashboard integration
- `secure_servo_sweep.rs` - Security-enhanced control
- `security_demo.rs` - Comprehensive security demonstration

## Security Features
- Attack simulation (DoS, injection, privilege escalation, etc.)
- Real-time monitoring and threat detection
- Emergency stop functionality
- Rate limiting and intrusion detection

## Web Dashboard
- Real-time robot visualization
- Interactive joint controls
- System status monitoring
- WebSocket communication for live updates

## Build System
- Cargo workspace with multiple crates
- C++ compilation via build.rs
- Cross-platform support (Windows, Linux, macOS)

