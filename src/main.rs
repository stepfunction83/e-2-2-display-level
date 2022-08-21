//! # Pico GPIO In/Out Example
//!
//! Toggles the LED based on GPIO input.
//!
//! This will control an LED on GP25 based on a button hooked up to GP15. The
//! button should cause the line to be grounded, as the input pin is pulled high
//! internally by this example. When the button is pressed, the LED will turn
//! off.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use hal::gpio;
// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::v2::{InputPin, OutputPin};

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// Need a vec to hold the pins
extern crate heapless;
use heapless::Vec;

// Need dynamic pins to hold in Vec
use rp_pico::hal::gpio::DynPin;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then just reads the button
/// and sets the LED appropriately.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Our LED output
    // let mut led_pin = pins.gpio15.into_push_pull_output();

    let gpio6: DynPin = pins.gpio6.into_push_pull_output().into();
    let gpio7: DynPin = pins.gpio7.into_push_pull_output().into();
    let gpio8: DynPin = pins.gpio8.into_push_pull_output().into();
    let gpio9: DynPin = pins.gpio9.into_push_pull_output().into();
    let gpio10: DynPin = pins.gpio10.into_push_pull_output().into();
    let gpio11: DynPin = pins.gpio11.into_push_pull_output().into();
    let gpio12: DynPin = pins.gpio12.into_push_pull_output().into();
    let gpio13: DynPin = pins.gpio13.into_push_pull_output().into();
    let gpio14: DynPin = pins.gpio14.into_push_pull_output().into();
    let gpio15: DynPin = pins.gpio15.into_push_pull_output().into();

    // Vec of pins in output
    let mut pins_out = [
        gpio6, gpio7, gpio8, gpio9, gpio10, gpio11, gpio12, gpio13, gpio14, gpio15,
    ];

    // Our button input
    // let button_pin = pins.gpio15.into_pull_up_input();

    // reset the pins to clear the output before running
    for pin in pins_out.iter_mut() {
        pin.set_low().unwrap();
    }

    // Run forever, setting the LED according to the button
    let pin_count = pins_out.len();
    loop {
        for pin in pins_out[0..pin_count - 1].iter_mut() {
            pin.set_high().unwrap();
            delay.delay_ms(1000);
            pin.set_low().unwrap();
        }

        for pin in pins_out[1..pin_count].iter_mut().rev() {
            pin.set_high().unwrap();
            delay.delay_ms(1000);
            pin.set_low().unwrap();
        }

        // for pin in pins_out.iter_mut() {
        //     pin.set_high().unwrap();
        //     delay.delay_ms(1000);
        // }

        // for pin in pins_out.iter_mut().rev() {
        //     pin.set_low().unwrap();
        //     delay.delay_ms(1000);
        // }
        // led_pin.set_high().unwrap();
        // delay.delay_ms(500);
        // led_pin.set_low().unwrap();
        // delay.delay_ms(500);
    }
}

// End of file
