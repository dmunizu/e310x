//! # I2C Async API
//! # Note
//!
//! Implementation of the Async Embedded HAL I2C functionality.
//!
use crate::asynch::poll_fn;
use crate::i2c::{I2c, I2cX};
use crate::DeviceResources;
use core::cell::RefCell;
use core::task::{Poll, Waker};
use critical_section::Mutex;
use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource, Operation};
use embedded_hal_async::i2c;

const FLAG_READ: u8 = 1;
const FLAG_WRITE: u8 = 0;
static I2C_WAKER: Mutex<RefCell<Option<I2cWaker>>> = Mutex::new(RefCell::new(None));

/// Trait for asynchronous I2C wait operations.
trait AsyncI2C {
    async fn wait_idle(&mut self);
    async fn ack_interrupt(&mut self) -> Result<(), ErrorKind>;
    async fn wait_for_write(&mut self, source: NoAcknowledgeSource) -> Result<(), ErrorKind>;
    async fn wait_for_read(&mut self) -> Result<(), ErrorKind>;
}

/// Interrupt handler function for I2C
fn on_irq() {
    // Wake the waker if it exists
    critical_section::with(|cs| {
        let mut i2cwaker = I2C_WAKER.borrow_ref_mut(cs);
        if let Some(waker) = i2cwaker.take() {
            waker.wake();
        }
    });
    // Clear the interrupt
    DeviceResources::take()
        .unwrap()
        .peripherals
        .I2C0
        .cr()
        .write(|w| w.iack().set_bit());
}

/// I2C Waker
struct I2cWaker {
    waker: Waker,
}

impl I2cWaker {
    fn new(waker: Waker) -> Self {
        Self { waker }
    }

    fn wake(self) {
        self.waker.wake();
    }
}

impl<I2C: I2cX, PINS> AsyncI2C for I2c<I2C, PINS> {
    async fn wait_idle(&mut self) {
        poll_fn(|cx| {
            if self.is_idle() {
                Poll::Ready(())
            } else {
                // Register the waker to be notified when the I2C is idle
                critical_section::with(|cs| {
                    let mut i2cwaker = I2C_WAKER.borrow_ref_mut(cs);
                    *i2cwaker = Some(I2cWaker::new(cx.waker().clone()))
                });
                Poll::Pending
            }
        })
        .await;
    }

    async fn ack_interrupt(&mut self) -> Result<(), ErrorKind> {
        poll_fn(|cx| {
            if !self.read_sr().tip().bit_is_set() {
                if self.read_sr().al().bit_is_set() {
                    self.set_stop();
                    Poll::Ready(Err(ErrorKind::ArbitrationLoss))
                } else {
                    Poll::Ready(Ok(()))
                }
            } else {
                // Register the waker to be notified when an interrupt occurs
                critical_section::with(|cs| {
                    let mut i2cwaker = I2C_WAKER.borrow_ref_mut(cs);
                    *i2cwaker = Some(I2cWaker::new(cx.waker().clone()))
                });
                Poll::Pending
            }
        })
        .await
    }

    async fn wait_for_write(&mut self, source: NoAcknowledgeSource) -> Result<(), ErrorKind> {
        self.ack_interrupt().await?;
        if self.read_sr().rx_ack().bit_is_set() {
            self.set_stop();
            Err(ErrorKind::NoAcknowledge(source))
        } else {
            Ok(())
        }
    }

    async fn wait_for_read(&mut self) -> Result<(), ErrorKind> {
        self.ack_interrupt().await
    }
}

impl<I2C: I2cX, PINS> i2c::I2c for I2c<I2C, PINS> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let n_ops = operations.len();
        if n_ops == 0 {
            return Ok(());
        }

        // Turn on i2c interrupt
        self.i2c.ctr().write(|w| w.ien().set_bit());

        self.wait_idle().await;
        self.reset();

        // we use this flag to detect when we need to send a (repeated) start
        let mut last_op_was_read = match &operations[0] {
            Operation::Read(_) => false,
            Operation::Write(_) => true,
        };

        for (i, operation) in operations.iter_mut().enumerate() {
            match operation {
                Operation::Write(bytes) => {
                    // Send write command
                    self.write_txr((address << 1) + FLAG_WRITE);
                    self.trigger_write(last_op_was_read, false);
                    self.wait_for_write(NoAcknowledgeSource::Address).await?;
                    last_op_was_read = false;

                    // Write bytes
                    let n_bytes = bytes.len();
                    for (j, byte) in bytes.iter().enumerate() {
                        self.write_txr(*byte);
                        self.trigger_write(false, (i == n_ops - 1) && (j == n_bytes - 1));
                        self.wait_for_write(NoAcknowledgeSource::Data).await?;
                    }
                }
                Operation::Read(buffer) => {
                    // Send read command
                    self.write_txr((address << 1) + FLAG_READ);
                    self.trigger_write(!last_op_was_read, false);
                    self.wait_for_write(NoAcknowledgeSource::Address).await?;
                    last_op_was_read = true;

                    // Read bytes
                    let n_bytes = buffer.len();
                    for (j, byte) in buffer.iter_mut().enumerate() {
                        self.trigger_read(j == n_bytes - 1, (i == n_ops - 1) && (j == n_bytes - 1));
                        self.wait_for_read().await?;
                        *byte = self.read_rxr();
                    }
                }
            }
        }
        self.wait_idle().await;

        // Clear and turn off i2c interrupt
        self.i2c.ctr().write(|w| w.ien().clear_bit());
        self.clear_interrupt();

        Ok(())
    }
}

/// Interrupt Handler
#[riscv_rt::external_interrupt(e310x::interrupt::ExternalInterrupt::I2C0)]
fn i2c_handler() {
    on_irq();
}
