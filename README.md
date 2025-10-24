# EmbedCore - Embedded Systems Middleware Framework

EmbedCore is a simulated embedded systems middleware framework that combines C++ hardware abstraction with Rust application development. It's designed for robotics applications with motor and servo control, featuring a cooperative task scheduler and safe device abstractions.

## 🏗️ Architecture

- **C++ Hardware Abstraction Layer (HAL)** - Simulated GPIO, PWM, UART, timers, and motor drivers
- **Rust Application Framework** - Safe device wrappers, task scheduler, and motor control logic
- **FFI Bridge** - Safe interoperability between C++ and Rust
- **Desktop Simulator** - No actual hardware required

## 📁 Project Structure

```
embedcore/
├── Cargo.toml                    # Rust workspace root
├── README.md
├── .gitignore
├── cpp_hal/                      # C++ Hardware Abstraction Layer
│   ├── include/
│   │   ├── gpio.h
│   │   ├── pwm.h
│   │   ├── uart.h
│   │   ├── timer.h
│   │   └── motor.h
│   └── src/
│       ├── gpio.cpp
│       ├── pwm.cpp
│       ├── uart.cpp
│       ├── timer.cpp
│       └── motor.cpp
├── embedcore-sys/                # Rust FFI bindings (unsafe)
│   ├── Cargo.toml
│   ├── build.rs                  # Compiles C++ code
│   └── src/
│       └── lib.rs
├── embedcore/                    # Rust safe abstractions
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs
│       ├── scheduler.rs
│       ├── task.rs
│       └── devices/
│           ├── mod.rs
│           ├── gpio.rs
│           ├── pwm.rs
│           └── motor.rs
└── examples/
    ├── blinky.rs                 # Simple GPIO example
    ├── servo_sweep.rs            # Servo motor control
    └── robot_arm.rs              # Multi-motor coordination
```

## 🚀 Quick Start

### Prerequisites

- Rust 1.70+ with Cargo
- C++ compiler (GCC, Clang, or MSVC)
- CMake (optional, for advanced builds)

### Building the Project

```bash
# Clone the repository
git clone <repository-url>
cd embedcore

# Build the entire workspace
cargo build

# Run examples
cargo run --example blinky
cargo run --example servo_sweep
cargo run --example robot_arm
```

### Running Examples

#### 1. Blinky Example
Simple GPIO LED blinking demonstration:
```bash
cargo run --example blinky
```

#### 2. Servo Sweep Example
Servo motor control with angle sweeping:
```bash
cargo run --example servo_sweep
```

#### 3. Robot Arm Example
Multi-motor coordination with task scheduling:
```bash
cargo run --example robot_arm
```

## 🔧 Features

### Hardware Abstraction Layer (C++)

- **GPIO Control** - Digital input/output with pull-up/pull-down support
- **PWM Generation** - Servo control with 0-180° angle mapping
- **Motor Control** - DC motors with direction and speed control
- **UART Communication** - Serial communication for telemetry/debugging
- **Timer System** - Millisecond-precision timing with callbacks
- **Safety Features** - Emergency stop and bounds checking

### Rust Application Framework

- **Safe Device Wrappers** - Memory-safe abstractions with error handling
- **Cooperative Task Scheduler** - Priority-based task execution
- **Builder Patterns** - Fluent API for device configuration
- **Type Safety** - Compile-time guarantees for motor types and states
- **Error Handling** - Comprehensive error types with detailed messages

### Task Scheduler

- **Priority-based Scheduling** - Critical, High, Normal, Low priorities
- **Periodic Tasks** - Configurable execution intervals
- **Deadline Monitoring** - Missed deadline detection and reporting
- **Task Management** - Create, suspend, resume, and remove tasks
- **Statistics** - Runtime performance monitoring

## 📖 Usage Examples

### Basic GPIO Control

```rust
use embedcore::{init, Gpio, GpioMode, GpioState, delay_ms};

fn main() {
    init();
    
    let led = Gpio::new(13, GpioMode::Output).unwrap();
    
    loop {
        led.write(GpioState::High);
        delay_ms(500);
        led.write(GpioState::Low);
        delay_ms(500);
    }
}
```

### Servo Motor Control

```rust
use embedcore::{init, Motor, MotorConfig, MotorType, delay_ms};

fn main() {
    init();
    
    let mut config = MotorConfig::default();
    config.motor_type = MotorType::Servo;
    config.pwm_channel = 0;
    
    let servo = Motor::new(0, config).unwrap();
    
    // Sweep from 0 to 180 degrees
    for angle in 0..=180 {
        servo.set_position(angle as f32).unwrap();
        delay_ms(20);
    }
}
```

### Task Scheduler Usage

```rust
use embedcore::{Scheduler, TaskPriority, TaskResult};

fn main() {
    let scheduler = Scheduler::new();
    
    // Add a periodic task
    let _task_id = scheduler.add_task(
        "blink_led".to_string(),
        TaskPriority::Normal,
        || {
            // Task logic here
            TaskResult::Reschedule
        },
        Some(1000), // Run every 1000ms
    ).unwrap();
    
    scheduler.start().unwrap();
    
    // Keep running
    loop {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
}
```

## 🎯 Use Cases

### Robotics Applications

- **Robot Arms** - Multi-DOF coordination with servo control
- **Mobile Robots** - DC motor control for wheels and actuators
- **Sensor Integration** - GPIO for sensors, UART for communication
- **Real-time Control** - Task scheduler for time-critical operations

### Educational Projects

- **Embedded Systems Learning** - Safe introduction to embedded concepts
- **Rust for Embedded** - Learn Rust in an embedded context
- **Control Systems** - PID control and feedback loops
- **System Integration** - Hardware-software co-design

## 🔒 Safety Features

- **Bounds Checking** - All motor positions and speeds are validated
- **Emergency Stop** - Global emergency stop for all motors
- **Memory Safety** - Rust's ownership system prevents common bugs
- **Error Handling** - Comprehensive error types with recovery options
- **Resource Management** - Automatic cleanup on device drop

## 🧪 Testing

```bash
# Run all tests
cargo test

# Run tests for specific crate
cargo test -p embedcore
cargo test -p embedcore-sys

# Run with output
cargo test -- --nocapture
```

## 📊 Performance

The simulator provides realistic timing behavior:

- **GPIO Operations** - ~1ms simulation delay
- **PWM Generation** - 50Hz for servos, configurable frequency
- **Motor Control** - Immediate response with safety checks
- **Task Scheduling** - 1ms tick resolution
- **UART Communication** - Simulated transmission delays

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Inspired by real embedded systems middleware frameworks
- Built with Rust's safety guarantees and C++'s performance
- Designed for educational and prototyping purposes

## 📞 Support

For questions, issues, or contributions:

- Open an issue on GitHub
- Check the examples for usage patterns
- Review the test cases for expected behavior

---

**Note**: This is a simulation framework designed for learning and prototyping. For production embedded systems, consider using actual hardware-specific libraries and real-time operating systems.
