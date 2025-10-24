//! Blinky example - Simple GPIO LED blinking
//! 
//! This example demonstrates basic GPIO usage by blinking an LED
//! connected to pin 13. The LED will blink on and off every 500ms.

use embedcore::{init, delay_ms};
use embedcore::devices::gpio::{Gpio, GpioMode, GpioState};

fn main() {
    println!("EmbedCore Blinky Example");
    println!("========================");
    
    // Initialize the EmbedCore system
    init();
    
    // Create GPIO pin 13 as output (simulated LED)
    let led = Gpio::new(13, GpioMode::Output).expect("Failed to create GPIO pin 13");
    
    println!("LED connected to pin 13");
    println!("Blinking LED every 500ms...");
    println!("Press Ctrl+C to stop");
    
    // Blink the LED
    loop {
        // Turn LED on
        led.write(GpioState::High);
        println!("LED ON");
        delay_ms(500);
        
        // Turn LED off
        led.write(GpioState::Low);
        println!("LED OFF");
        delay_ms(500);
    }
}
