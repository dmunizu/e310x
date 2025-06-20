//! Basic blinking LEDs example using mtime/mtimecmp registers for "sleep" in a loop.
//! Blinks each led once and goes to the next one.

#![no_std]
#![no_main]

use hifive1::{
    clock,
    hal::{prelude::*, DeviceResources},
    pin, pins, sprintln, stdout, Led,
};
extern crate panic_halt;

#[riscv_rt::entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let cp = dr.core_peripherals;
    let p = dr.peripherals;
    let pins = dr.pins;

    // Configure clocks
    let clocks = clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    // Configure UART for stdout
    stdout::configure(
        p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );

    // get all 3 led pins in a tuple (each pin is it's own type here)
    let rgb_pins = pins!(pins, (led_red, led_green, led_blue));
    let mut tleds = hifive1::rgb(rgb_pins.0, rgb_pins.1, rgb_pins.2);
    // get leds as the Led trait in an array so we can index them
    let mut ileds: [&mut dyn Led; 3] = [&mut tleds.0, &mut tleds.1, &mut tleds.2];

    // Get the MTIMER peripheral from CLINT
    let mut mtimer = cp.clint.mtimer();

    const STEP: u32 = 1000; // 1s
    loop {
        for (i, led) in ileds.iter_mut().enumerate() {
            led.toggle().unwrap();
            sprintln!("LED {} toggled. New state: {}", i, led.is_on());
            mtimer.delay_ms(STEP);
        }
    }
}
