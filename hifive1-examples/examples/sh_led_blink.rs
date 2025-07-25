//! Basic blinking LEDs example using mtime/mtimecmp registers for "sleep" in a loop.
//! Blinks each led once and goes to the next one.

#![no_std]
#![no_main]

use hifive1::{
    clock,
    hal::{prelude::*, DeviceResources},
    pin, Led,
};
use semihosting::{println, process::exit};

#[riscv_rt::entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let cp = dr.core_peripherals;
    let p = dr.peripherals;
    let pins = dr.pins;

    // Configure clocks
    clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    // get all 3 led pins in a tuple (each pin is it's own type here)
    let pin = pin!(pins, led_blue);
    let mut led = pin.into_inverted_output();

    // Get the MTIMER peripheral from CLINT
    let mut mtimer = cp.clint.mtimer();

    const N_TOGGLE: usize = 4;
    const STEP: u32 = 500; // 500 ms

    println!("Toggling LED {} times", N_TOGGLE);
    for _ in 0..N_TOGGLE {
        Led::toggle(&mut led);
        println!("LED toggled. New state: {}", led.is_on());
        mtimer.delay_ms(STEP);
    }
    println!("Done toggling LED");
    exit(0);
}
