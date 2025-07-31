use super::{SharedBus, SpiExclusiveDevice};
use crate::spi::{CommType, Pins, PinsFull, PinsNoCS, SpiBus, SpiConfig, SpiX, WatermarkValue};
use core::cell::RefCell;
use core::future::poll_fn;
use core::task::{Poll, Waker};
use critical_section::Mutex;
use e310x::{Qspi0, Qspi1, Qspi2};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_hal_async::{
    delay::DelayNs,
    spi::{self, ErrorKind},
};

const EMPTY_WRITE_PAD: u8 = 0x00;
const N_SPI: usize = 3;
static SPI_WAKERS: Mutex<RefCell<[Option<Waker>; N_SPI]>> =
    Mutex::new(RefCell::new([const { None }; N_SPI]));

/// Interrupt handler function for SPI
#[inline]
fn on_irq<SPI: SpiX>(qspi: SPI) {
    // Awake the waker for the corresponding SPI index
    critical_section::with(|cs| {
        if let Some(waker) = SPI_WAKERS.borrow_ref_mut(cs)[SPI::SPI_INDEX].take() {
            waker.wake();
        }
    });

    // Disable the interrupt
    qspi.ie().write(|w| w.txwm().clear_bit().rxwm().clear_bit());
}

impl<SPI: SpiX, PINS: Pins<SPI>> SpiBus<SPI, PINS> {
    /// Create a new [`SpiExclusiveDeviceAsync`] for exclusive use on this bus
    pub fn new_device_async<D: DelayNs>(
        self,
        config: &SpiConfig,
        delay: D,
    ) -> SpiExclusiveDevice<SPI, PINS, D> {
        SpiExclusiveDevice::new(self, config, delay)
    }
}

impl<SPI: SpiX, PINS: PinsNoCS<SPI>> SpiBus<SPI, PINS> {
    /// Create a [`SharedBus`] for use with multiple devices.
    pub fn shared_async<M: RawMutex>(spi: SPI, pins: PINS) -> SharedBus<M, SPI, PINS> {
        SharedBus::new(Self::new(spi, pins))
    }
}

impl<SPI: SpiX, PINS> SpiBus<SPI, PINS> {
    /// Read a single byte from the SPI bus.
    ///
    /// This function will wait if the RX FIFO is empty.
    async fn read_input_async(&self) -> Result<u8, ErrorKind> {
        // Poll the input until data is available
        poll_fn(|cx| match self.read_input() {
            Ok(data) => Poll::Ready(Ok(data)),
            Err(nb::Error::WouldBlock) => {
                // Register the waker to be notified when an interrupt occurs
                critical_section::with(|cs| {
                    let spiwaker = &mut SPI_WAKERS.borrow_ref_mut(cs)[SPI::SPI_INDEX];
                    *spiwaker = Some(cx.waker().clone());
                });
                self.set_watermark(CommType::Rx, WatermarkValue::W0);
                self.enable_interrupt(CommType::Rx);
                Poll::Pending
            }
            Err(nb::Error::Other(e)) => Poll::Ready(Err(e)),
        })
        .await
    }

    /// Write a single byte to the SPI bus.
    ///
    /// This function will wait if the TX FIFO is full.
    async fn write_output_async(&self, word: u8) -> Result<(), ErrorKind> {
        // Poll the output until it is ready to accept data
        poll_fn(|cx| match self.write_output(word) {
            Ok(()) => Poll::Ready(Ok(())),
            Err(nb::Error::WouldBlock) => {
                // Register the waker to be notified when an interrupt occurs
                critical_section::with(|cs| {
                    let spiwaker = &mut SPI_WAKERS.borrow_ref_mut(cs)[SPI::SPI_INDEX];
                    *spiwaker = Some(cx.waker().clone())
                });
                self.set_watermark(CommType::Tx, WatermarkValue::W7);
                self.enable_interrupt(CommType::Tx);
                Poll::Pending
            }
            Err(nb::Error::Other(e)) => Poll::Ready(Err(e)),
        })
        .await
    }
}

impl<SPI: SpiX, PINS: PinsFull<SPI>> spi::SpiBus for SpiBus<SPI, PINS> {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut iwrite = 0;
        let mut iread = 0;

        // Ensure that RX FIFO is empty
        self.wait_for_rxfifo();

        while iwrite < words.len() || iread < words.len() {
            if iwrite < words.len() {
                match self.write_output_async(EMPTY_WRITE_PAD).await {
                    Ok(()) => iwrite += 1,
                    Err(e) => return Err(e),
                }
            }
            if iread < iwrite {
                match self.read_input_async().await {
                    Ok(data) => {
                        unsafe { *words.get_unchecked_mut(iread) = data };
                        iread += 1;
                    }
                    Err(e) => return Err(e),
                }
            }
        }
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut iwrite = 0;
        let mut iread = 0;

        // Ensure that RX FIFO is empty
        self.wait_for_rxfifo();

        while iwrite < words.len() || iread < words.len() {
            if iwrite < words.len() {
                let byte = unsafe { words.get_unchecked(iwrite) };
                match self.write_output_async(*byte).await {
                    Ok(()) => iwrite += 1,
                    Err(e) => return Err(e),
                }
            }
            if iread < iwrite {
                match self.read_input_async().await {
                    Ok(_) => iread += 1,
                    Err(e) => return Err(e),
                }
            }
        }
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut iwrite = 0;
        let mut iread = 0;
        let max_len = read.len().max(write.len());

        // Ensure that RX FIFO is empty
        self.wait_for_rxfifo();

        while iwrite < max_len || iread < max_len {
            if iwrite < max_len {
                let byte = write.get(iwrite).unwrap_or(&EMPTY_WRITE_PAD);
                match self.write_output_async(*byte).await {
                    Ok(()) => iwrite += 1,
                    Err(e) => return Err(e),
                }
            }
            if iread < iwrite {
                match self.read_input_async().await {
                    Ok(data) => {
                        if let Some(byte) = read.get_mut(iread) {
                            *byte = data;
                        }
                        iread += 1;
                    }
                    Err(e) => return Err(e),
                }
            }
        }
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut iwrite = 0;
        let mut iread = 0;

        // Ensure that RX FIFO is empty
        self.wait_for_rxfifo();

        while iwrite < words.len() || iread < words.len() {
            if iwrite < words.len() {
                let byte = unsafe { words.get_unchecked(iwrite) };
                match self.write_output_async(*byte).await {
                    Ok(()) => iwrite += 1,
                    Err(e) => return Err(e),
                }
            }
            if iread < iwrite {
                match self.read_input_async().await {
                    Ok(data) => {
                        unsafe { *words.get_unchecked_mut(iread) = data };
                        iread += 1;
                    }
                    Err(e) => return Err(e),
                }
            }
        }

        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // Empty RX FIFO
        self.wait_for_rxfifo();

        let mut flushed = false;
        // Wait for the TX FIFO to be empty
        poll_fn(|cx| {
            if flushed {
                Poll::Ready(())
            } else {
                flushed = true;
                // Register the waker to be notified when an interrupt occurs
                critical_section::with(|cs| {
                    let spiwaker = &mut SPI_WAKERS.borrow_ref_mut(cs)[SPI::SPI_INDEX];
                    *spiwaker = Some(cx.waker().clone())
                });
                self.set_watermark(CommType::Tx, WatermarkValue::W1);
                self.enable_interrupt(CommType::Tx);
                Poll::Pending
            }
        })
        .await;

        Ok(())
    }
}

/// Interrupt Handlers
#[riscv_rt::external_interrupt(e310x::interrupt::ExternalInterrupt::QSPI0)]
fn qspi0_handler() {
    let qspi0 = unsafe { Qspi0::steal() };
    on_irq(qspi0);
}
#[riscv_rt::external_interrupt(e310x::interrupt::ExternalInterrupt::QSPI1)]
fn qspi1_handler() {
    let qspi1 = unsafe { Qspi1::steal() };
    on_irq(qspi1);
}
#[riscv_rt::external_interrupt(e310x::interrupt::ExternalInterrupt::QSPI2)]
fn qspi2_handler() {
    let qspi2 = unsafe { Qspi2::steal() };
    on_irq(qspi2);
}
