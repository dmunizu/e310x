//! Sample example for the BME280 temperature, pressure and humidity sensor.
//! using the SPI interface.
//! This example uses synchronous UART and only tests asynchronous SPI.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embedded_devices::devices::bosch::bme280::registers::{IIRFilter, Oversampling};
use embedded_devices::devices::bosch::bme280::{BME280Async, Configuration};
use hifive1::{
    clock,
    hal::{
        DeviceResources,
        asynch::delay::Delay,
        asynch::prelude::*,
        e310x::interrupt::Hart,
        prelude::*,
        spi::{MODE_0, SpiBus, SpiConfig},
    },
    pin, sprintln,
};
use uom::num_traits::ToPrimitive;
use uom::si::thermodynamic_temperature::degree_celsius;
extern crate panic_halt;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let cp = dr.core_peripherals;
    let pins = dr.pins;

    // Configure clocks
    let clocks = clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    // Configure UART for stdout
    hifive1::stdout::configure(
        p.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );

    // Get the MTIMER peripheral from CLINT
    let mtimer = cp.clint.mtimer();
    mtimer.disable();
    let (mtimecmp, mtime) = (mtimer.mtimecmp(Hart::H0), mtimer.mtime());
    mtime.write(0);
    mtimecmp.write(u64::MAX);
    let spi_delay = Delay::new(mtimer);
    let mut delay = Delay::new(mtimer);
    const STEP: u32 = 1000; // 1s

    // SPI configuration
    let sck = pin!(pins, spi1_sck).into_iof0();
    let miso = pin!(pins, spi1_miso).into_iof0();
    let mosi = pin!(pins, spi1_mosi).into_iof0();
    let cs = pin!(pins, spi1_ss0).into_iof0();
    let spi_bus = SpiBus::new(p.QSPI1, (mosi, miso, sck, cs));
    let spi_cfg = SpiConfig::new(MODE_0, 1_000_000.hz(), &clocks);

    // Enable interrupts
    let plic = cp.plic;
    let ctx = plic.ctx0();
    unsafe {
        ctx.enables().disable_all::<ExternalInterrupt>();
        spi_bus.enable_exti();
        ctx.threshold().set_threshold(Priority::P0);
        riscv::interrupt::enable();
        plic.enable();
    };

    //SPI device configuration
    let spi_device = spi_bus.new_device_async(&spi_cfg, spi_delay);

    //BME280 sensor configuration
    let mut bme280 = BME280Async::new_spi(spi_device);
    bme280.init(&mut delay).await.unwrap();
    bme280
        .configure(Configuration {
            temperature_oversampling: Oversampling::X_16,
            pressure_oversampling: Oversampling::Disabled,
            humidity_oversampling: Oversampling::Disabled,
            iir_filter: IIRFilter::Disabled,
        })
        .await
        .unwrap();
    loop {
        let measurements = bme280.measure(&mut delay).await.unwrap();
        let temp = measurements.temperature.get::<degree_celsius>().to_f32();
        sprintln!("Current temperature: {:.2} Celsius", temp.unwrap());
        delay.delay_ms(STEP).await;
    }
}
