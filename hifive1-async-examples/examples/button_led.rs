//! Basic example where the LED changes its state according to a button connected to pin 9.
//! The LED must be connected to pin 10 of the board
//! This example uses synchronous UART and only tests asynchronous GPIO.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use hifive1::{
    clock,
    hal::{DeviceResources, asynch::prelude::*, e310x::Gpio0, gpio::EventType, prelude::*},
    sprintln,
};
extern crate panic_halt;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let dr = DeviceResources::take().unwrap();
    let cp = dr.core_peripherals;
    let p = dr.peripherals;
    let pins: hifive1::hal::device::DeviceGpioPins = dr.pins;

    // Configure clocks
    let clocks = clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    //Blinking LED
    let mut led = pins.pin10.into_output();

    // Configure UART for stdout
    hifive1::stdout::configure(p.UART0, pins.pin17, pins.pin16, 115_200.bps(), clocks);

    // Button pin (GPIO9) as pull-up input
    let mut button = pins.pin9.into_pull_up_input();

    // Clear pending interrupts from previous states
    Gpio0::disable_interrupts(EventType::All);
    Gpio0::clear_pending_interrupts(EventType::All);

    // Set button interrupt source priority
    let plic = cp.plic;
    let priorities = plic.priorities();
    priorities.reset::<ExternalInterrupt>();
    unsafe { button.set_exti_priority(Priority::P1) };

    // Enable GPIO9 interrupt in PLIC
    let ctx = plic.ctx0();
    unsafe {
        ctx.enables().disable_all::<ExternalInterrupt>();
        ctx.threshold().set_threshold(Priority::P0);
        button.enable_exti();
        riscv::interrupt::enable();
        plic.enable();
    };

    // Execute loop
    loop {
        led.toggle().unwrap();
        let led_state = led.is_set_high().unwrap();
        sprintln!("LED toggled. New state: {}", led_state);
        button.wait_for_any_edge().await.unwrap();
    }
}
