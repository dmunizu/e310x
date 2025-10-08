//! Example using all the asynchronous peripherals at the same time.
//! The UART Rx controls tasks that periodically show temperature through an I2C controlled
//! BME280 and humidity through an SPI controlled BME280.
//! Another task checks the button to change the state of an LED at each press.
//! NOTE: An LED has to be connected to pin 10 and a button to pin 9. The Board LED cannot be used as it collides
//! with the SPI SCK in the RED-V RedBoard.

#![no_std]
#![no_main]

use core::fmt::Write;
use core::str;
use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::Timer;
use embedded_devices::devices::bosch::bme280::{
    registers::{IIRFilter, Oversampling},
    {BME280Async, Configuration, address::Address},
};
use embedded_devices::sensor::OneshotSensorAsync;
use heapless::String;
use hifive1::{
    clock,
    hal::{
        DeviceResources,
        asynch::{delay::Delay, prelude::*, spi::SpiExclusiveDevice},
        e310x::{Gpio0, Uart0, generic, i2c0, interrupt::Hart, qspi0},
        gpio::{EventType, IOF0, Input, NoInvert, Output, PullUp, Regular, gpio0},
        i2c::{I2c, Speed},
        prelude::*,
        serial::{Rx, Serial, Tx},
        spi::{MODE_0, SpiBus, SpiConfig},
    },
};
use uom::si::{ratio::percent, thermodynamic_temperature::degree_celsius};
extern crate panic_halt;

#[derive(PartialEq)]
enum TaskCommand {
    Start,
    Stop,
}

type LedType = gpio0::Pin10<Output<Regular<NoInvert>>>;
type ButtonType = gpio0::Pin9<Input<PullUp>>;
type RxType = Rx<Uart0, gpio0::Pin16<IOF0<NoInvert>>>;
type TxType = Tx<Uart0, gpio0::Pin17<IOF0<NoInvert>>>;
type SpiDeviceType = SpiExclusiveDevice<
    generic::Periph<qspi0::RegisterBlock, 268582912>,
    (
        gpio0::Pin3<IOF0<NoInvert>>,
        gpio0::Pin4<IOF0<NoInvert>>,
        gpio0::Pin5<IOF0<NoInvert>>,
        gpio0::Pin2<IOF0<NoInvert>>,
    ),
    Delay,
>;
type I2cType = I2c<
    generic::Periph<i2c0::RegisterBlock, 268525568>,
    (gpio0::Pin12<IOF0<NoInvert>>, gpio0::Pin13<IOF0<NoInvert>>),
>;

static I2C_SIGNAL: Signal<CriticalSectionRawMutex, TaskCommand> = Signal::new();
static SPI_SIGNAL: Signal<CriticalSectionRawMutex, TaskCommand> = Signal::new();
static LED_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static TX_MUTEX: Mutex<CriticalSectionRawMutex, Option<TxType>> = Mutex::new(None);

/// Continuously read up to newline and activate the corresponding task.
#[embassy_executor::task]
async fn read_task(mut rx: RxType) {
    loop {
        // Temporary buffer
        let mut buf = [0u8; 20];
        // Read
        let n = _eioa_Read::read(&mut rx, &mut buf).await.unwrap();

        // Activate the corresponding task
        let string = str::from_utf8(&buf[..n]).unwrap().trim();
        match string {
            "TEMP ON" => I2C_SIGNAL.signal(TaskCommand::Start),
            "TEMP OFF" => I2C_SIGNAL.signal(TaskCommand::Stop),
            "HUM ON" => SPI_SIGNAL.signal(TaskCommand::Start),
            "HUM OFF" => SPI_SIGNAL.signal(TaskCommand::Stop),
            "LED ON" => LED_SIGNAL.signal(true),
            "LED OFF" => LED_SIGNAL.signal(false),
            _ => {}
        }
    }
}

/// Wait for start signal and start I2C temperature measurement until a stop is received.
#[embassy_executor::task]
async fn temp_task(i2c: I2cType, delay: Delay) {
    //I2C BME280 sensor configuration
    let mut bme280 = BME280Async::new_i2c(delay, i2c, Address::Primary);
    bme280.init().await.unwrap();
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
        // Wait for start signal
        while let TaskCommand::Stop = I2C_SIGNAL.wait().await {}

        // Wait for stop future
        let stop = async { while let TaskCommand::Start = I2C_SIGNAL.wait().await {} };

        // Measure temperature future
        let measure_temp = async {
            loop {
                // Measure
                let measurement = bme280.measure().await.unwrap();
                let temp = measurement.temperature.get::<degree_celsius>();

                // Async Print
                let mut string: String<40> = String::new();
                if let Ok(()) = writeln!(string, "Current temperature: {:.2} Celsius", temp) {
                    async_print(string).await;
                }

                // Wait
                Timer::after_millis(1000).await;
            }
        };

        // Keep taking measurements until stop is received
        select(stop, measure_temp).await;
    }
}

/// Wait for start signal and start SPI humidity measurement until a stop is received.
#[embassy_executor::task]
async fn hum_task(spi_device: SpiDeviceType, delay: Delay) {
    //SPI BME280 sensor configuration
    let mut bme280 = BME280Async::new_spi(delay, spi_device);
    bme280.init().await.unwrap();
    bme280
        .configure(Configuration {
            temperature_oversampling: Oversampling::Disabled,
            pressure_oversampling: Oversampling::Disabled,
            humidity_oversampling: Oversampling::X_16,
            iir_filter: IIRFilter::Disabled,
        })
        .await
        .unwrap();

    loop {
        // Wait for start signal
        while let TaskCommand::Stop = SPI_SIGNAL.wait().await {}

        // Wait for stop future
        let stop = async { while let TaskCommand::Start = SPI_SIGNAL.wait().await {} };

        // Measure humidity future
        let measure_hum = async {
            loop {
                // Measure
                let measurement = bme280.measure().await.unwrap();
                let humidity = measurement.humidity.unwrap().get::<percent>();

                // Async Print
                let mut string: String<28> = String::new();
                if let Ok(()) = writeln!(string, "Current humidity: {:.2}%RH", humidity) {
                    async_print(string).await;
                }

                // Wait
                Timer::after_millis(1000).await;
            }
        };

        // Keep taking measurements until stop is received
        select(stop, measure_hum).await;
    }
}

/// Change the state of an LED through the button or an UART command.
#[embassy_executor::task]
async fn led_task(led: LedType, mut button: ButtonType) {
    let led_mutex: Mutex<CriticalSectionRawMutex, LedType> = Mutex::new(led);
    let uart_led = async {
        let mut led_state;

        loop {
            // Wait for LED signal
            match LED_SIGNAL.wait().await {
                true => {
                    let mut led = led_mutex.lock().await;
                    led.set_high().unwrap();
                    led_state = led.is_set_high().unwrap();
                }
                false => {
                    let mut led = led_mutex.lock().await;
                    led.set_low().unwrap();
                    led_state = led.is_set_high().unwrap();
                }
            }

            // Async Print
            let mut string: String<17> = String::new();
            if let Ok(()) = writeln!(string, "LED State: {:?}", led_state) {
                async_print(string).await;
            }
        }
    };

    let button_led = async {
        let rebound_step = 200;
        let mut led_state;

        loop {
            // Wait for button and toggle LED
            button.wait_for_rising_edge().await.unwrap();
            {
                let mut led = led_mutex.lock().await;
                led.toggle().unwrap();
                led_state = led.is_set_high().unwrap();
            }

            // Async Print
            let mut string: String<17> = String::new();
            if let Ok(()) = writeln!(string, "LED State: {:?}", led_state) {
                async_print(string).await;
            }

            // Rebound delay
            Timer::after_millis(rebound_step).await;
        }
    };

    join(uart_led, button_led).await;
}
// Main function
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (i2c_delay, spi_delay, led, button, rx, spi_device, i2c) = peripheral_config().await;
    // Spawn the read and write tasks
    spawner.spawn(read_task(rx)).unwrap();
    spawner.spawn(temp_task(i2c, i2c_delay)).unwrap();
    spawner.spawn(hum_task(spi_device, spi_delay)).unwrap();
    spawner.spawn(led_task(led, button)).unwrap();
}

async fn peripheral_config() -> (
    Delay,
    Delay,
    LedType,
    ButtonType,
    RxType,
    SpiDeviceType,
    I2cType,
) {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let cp = dr.core_peripherals;
    let pins = dr.pins;

    // Configure clocks
    let clocks = clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    //Blinking LED
    let led: hifive1::hal::gpio::gpio0::Pin10<
        hifive1::hal::gpio::Output<hifive1::hal::gpio::Regular<NoInvert>>,
    > = pins.pin10.into_output();

    // Button pin (GPIO9) as pull-up input
    let button = pins.pin9.into_pull_up_input();

    // Clear pending interrupts from previous states
    Gpio0::disable_interrupts(EventType::All);
    Gpio0::clear_pending_interrupts(EventType::All);

    // Configure MTIMER interrupt
    let mtimer = cp.clint.mtimer();
    mtimer.disable();
    let (mtimecmp, mtime) = (mtimer.mtimecmp(Hart::H0), mtimer.mtime());
    mtime.write(0);
    mtimecmp.write(u64::MAX);
    let i2c_delay = Delay::new(mtimer);
    let spi_delay = Delay::new(mtimer);

    // Configure UART
    let tx = pins.pin17.into_iof0();
    let rx = pins.pin16.into_iof0();
    let serial = Serial::new(p.UART0, (tx, rx), 115_200.bps(), clocks);

    // I2C configuration
    let sda = pins.pin12.into_iof0();
    let scl = pins.pin13.into_iof0();
    let i2c = I2c::new(p.I2C0, sda, scl, Speed::Normal, clocks);

    // SPI configuration
    let sck = pins.pin5.into_iof0();
    let miso = pins.pin4.into_iof0();
    let mosi = pins.pin3.into_iof0();
    let cs = pins.pin2.into_iof0();
    let spi_bus = SpiBus::new(p.QSPI1, (mosi, miso, sck, cs));
    let spi_cfg = SpiConfig::new(MODE_0, 1_000_000.hz(), &clocks);

    // Configure interrupts
    let plic = cp.plic;
    let priorities = plic.priorities();
    priorities.reset::<ExternalInterrupt>();
    unsafe {
        button.set_exti_priority(&plic, Priority::P1);
        serial.set_exti_priority(&plic, Priority::P1);
        i2c.set_exti_priority(&plic, Priority::P1);
        spi_bus.set_exti_priority(&plic, Priority::P1)
    };

    let ctx = plic.ctx0();
    unsafe {
        ctx.enables().disable_all::<ExternalInterrupt>();
        button.enable_exti(&plic);
        serial.enable_exti(&plic);
        spi_bus.enable_exti(&plic);
        i2c.enable_exti(&plic);
        ctx.threshold().set_threshold(Priority::P0);
        riscv::interrupt::enable();
        plic.enable();
    };

    // Split the serial into Tx and Rx parts
    let (tx, rx) = serial.split();

    //SPI BME280 sensor configuration
    let spi_device_delay = Delay::new(mtimer);
    let spi_device = spi_bus.new_device_async(&spi_cfg, spi_device_delay);

    // Store TX in a mutex
    let mut guard = TX_MUTEX.lock().await;
    *guard = Some(tx);

    // Return the rest of the peripherals
    (i2c_delay, spi_delay, led, button, rx, spi_device, i2c)
}

async fn async_print<const N: usize>(string: String<N>) {
    // Convert string to bytes
    let buf = string.as_bytes();
    let count = buf
        .iter()
        .position(|&b| b == b'\n')
        .map(|pos| pos + 1)
        .unwrap_or(buf.len());
    let mut buf = &buf[..count];

    // Get Mutex and write to UART
    let mut guard = TX_MUTEX.lock().await;
    let tx = guard.as_mut().unwrap();
    while !buf.is_empty() {
        let written = _eioa_Write::write(tx, buf).await.unwrap();
        buf = &buf[written..];
    }
}
