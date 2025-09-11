//! Simple echo example for the asynchronous UART

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use hifive1::{
    clock,
    hal::{
        DeviceResources,
        asynch::prelude::*,
        e310x::Uart0,
        gpio::{
            IOF0, NoInvert,
            gpio0::{Pin16, Pin17},
        },
        prelude::*,
        serial::{Rx, Serial, Tx},
    },
};
extern crate panic_halt;
static SHARED_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 20], 1> = Channel::new();

/// Continuously read and send the buffer over the channel.
#[embassy_executor::task]
async fn read_task(mut rx: Rx<Uart0, Pin16<IOF0<NoInvert>>>) {
    loop {
        // Temporary buffer
        let mut buf = [0u8; 20];

        // Read
        _eioa_Read::read(&mut rx, &mut buf).await.unwrap();

        // Send the filled buffer to the echo task
        SHARED_CHANNEL.send(buf).await;
    }
}

/// Wait for a message on the channel and echo it back over UART Tx.
#[embassy_executor::task]
async fn echo_task(mut tx: Tx<Uart0, Pin17<IOF0<NoInvert>>>) {
    loop {
        // Receive a buffer from the reader
        let buf = SHARED_CHANNEL.receive().await;

        // Search for a newline character in buf
        let count = buf
            .iter()
            .position(|&b| b == b'\n')
            .map(|pos| pos + 1)
            .unwrap_or(buf.len());

        // Echo back
        _eioa_Write::write(&mut tx, &buf[..count]).await.unwrap();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let dr = DeviceResources::take().unwrap();
    let cp = dr.core_peripherals;
    let p = dr.peripherals;
    let pins: hifive1::hal::device::DeviceGpioPins = dr.pins;

    // Configure clocks
    let clocks = clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    // Configure UART
    let tx = pins.pin17.into_iof0();
    let rx = pins.pin16.into_iof0();
    let serial = Serial::new(p.UART0, (tx, rx), 115_200.bps(), clocks);

    // Configure interrupts
    let plic = cp.plic;
    let ctx = plic.ctx0();
    unsafe {
        ctx.enables().disable_all::<ExternalInterrupt>();
        serial.enable_exti();
        ctx.threshold().set_threshold(Priority::P0);
        riscv::interrupt::enable();
        plic.enable();
    };

    // Split the serial into Tx and Rx parts
    let (tx, rx) = serial.split();

    // Spawn the read and write tasks
    spawner.spawn(read_task(rx)).unwrap();
    spawner.spawn(echo_task(tx)).unwrap();
}
