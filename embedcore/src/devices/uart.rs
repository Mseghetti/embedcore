//! UART (Universal Asynchronous Receiver-Transmitter) device abstraction

use embedcore_sys::{self, uart_config_t};
use std::ffi::CString;

/// UART configuration error
#[derive(Debug, thiserror::Error)]
pub enum UartError {
    #[error("Invalid configuration")]
    InvalidConfiguration,
    #[error("Port not configured")]
    PortNotConfigured,
    #[error("Transmission failed")]
    TransmissionFailed,
    #[error("Invalid string")]
    InvalidString,
}

/// UART configuration
#[derive(Debug, Clone, Copy)]
pub struct UartConfig {
    pub baud_rate: u32,
    pub data_bits: u8,
    pub stop_bits: u8,
    pub parity_enabled: bool,
    pub parity_odd: bool,
}

impl Default for UartConfig {
    fn default() -> Self {
        UartConfig {
            baud_rate: 9600,
            data_bits: 8,
            stop_bits: 1,
            parity_enabled: false,
            parity_odd: false,
        }
    }
}

impl From<UartConfig> for uart_config_t {
    fn from(config: UartConfig) -> Self {
        uart_config_t {
            baud_rate: config.baud_rate,
            data_bits: config.data_bits,
            stop_bits: config.stop_bits,
            parity_enabled: config.parity_enabled,
            parity_odd: config.parity_odd,
        }
    }
}

/// UART device abstraction
/// 
/// Provides safe access to UART ports with proper error handling and
/// bounds checking.
#[derive(Debug)]
pub struct Uart {
    port: u8,
    config: UartConfig,
}

impl Uart {
    /// Create a new UART instance for the specified port
    /// 
    /// # Arguments
    /// * `port` - The UART port number (0-255)
    /// * `config` - The UART configuration
    /// 
    /// # Returns
    /// * `Result<Uart, UartError>` - Ok(Uart) if successful, Err otherwise
    pub fn new(port: u8, config: UartConfig) -> Result<Self, UartError> {
        let uart = Uart { port, config };
        uart.configure()?;
        Ok(uart)
    }
    
    /// Configure the UART port
    fn configure(&self) -> Result<(), UartError> {
        let config = self.config.into();
        let success = unsafe {
            embedcore_sys::uart_configure(self.port, &config)
        };
        
        if success {
            Ok(())
        } else {
            Err(UartError::PortNotConfigured)
        }
    }
    
    /// Send data via UART
    /// 
    /// # Arguments
    /// * `data` - The data to send
    /// 
    /// # Returns
    /// * `Result<(), UartError>` - Ok if successful, Err otherwise
    pub fn send(&self, data: &[u8]) -> Result<(), UartError> {
        if data.is_empty() {
            return Ok(());
        }
        
        let success = unsafe {
            embedcore_sys::uart_send(self.port, data.as_ptr(), data.len() as u16)
        };
        
        if success {
            Ok(())
        } else {
            Err(UartError::TransmissionFailed)
        }
    }
    
    /// Receive data via UART
    /// 
    /// # Arguments
    /// * `buffer` - Buffer to store received data
    /// 
    /// # Returns
    /// * Number of bytes received
    pub fn receive(&self, buffer: &mut [u8]) -> usize {
        if buffer.is_empty() {
            return 0;
        }
        
        unsafe {
            embedcore_sys::uart_receive(self.port, buffer.as_mut_ptr(), buffer.len() as u16) as usize
        }
    }
    
    /// Check if data is available for reading
    pub fn data_available(&self) -> bool {
        unsafe {
            embedcore_sys::uart_data_available(self.port)
        }
    }
    
    /// Send a string via UART
    /// 
    /// # Arguments
    /// * `text` - The string to send
    /// 
    /// # Returns
    /// * `Result<(), UartError>` - Ok if successful, Err otherwise
    pub fn send_string(&self, text: &str) -> Result<(), UartError> {
        let c_string = CString::new(text).map_err(|_| UartError::InvalidString)?;
        let success = unsafe {
            embedcore_sys::uart_send_string(self.port, c_string.as_ptr())
        };
        
        if success {
            Ok(())
        } else {
            Err(UartError::TransmissionFailed)
        }
    }
    
    /// Flush UART buffers
    pub fn flush(&self) {
        unsafe {
            embedcore_sys::uart_flush(self.port);
        }
    }
    
    /// Get the port number
    pub fn port(&self) -> u8 {
        self.port
    }
    
    /// Get the current configuration
    pub fn config(&self) -> UartConfig {
        self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uart_creation() {
        let config = UartConfig::default();
        let uart = Uart::new(0, config);
        assert!(uart.is_ok());
    }

    #[test]
    fn test_uart_send() {
        let config = UartConfig::default();
        let uart = Uart::new(0, config).unwrap();
        let data = b"Hello, World!";
        assert!(uart.send(data).is_ok());
    }

    #[test]
    fn test_uart_send_string() {
        let config = UartConfig::default();
        let uart = Uart::new(0, config).unwrap();
        assert!(uart.send_string("Hello, World!").is_ok());
    }

    #[test]
    fn test_uart_receive() {
        let config = UartConfig::default();
        let uart = Uart::new(0, config).unwrap();
        let mut buffer = [0u8; 10];
        let received = uart.receive(&mut buffer);
        assert_eq!(received, 0); // No data available in simulation
    }
}
