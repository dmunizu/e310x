//! # I2C Async API
//! # Note
//!
//! Implementation of the Async Embedded HAL I2C functionality.
//!
use crate::i2c::{I2c, I2cX};
use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource, Operation};
use embedded_hal_async::i2c;

const FLAG_READ: u8 = 1;
const FLAG_WRITE: u8 = 0;

/// Trait for asynchronous I2C wait operations.
trait AsyncI2C {
    async fn wait_idle(&mut self);
    async fn write_txr(&mut self, byte: u8);
    async fn wait_for_write(&mut self, source: NoAcknowledgeSource) -> Result<(), ErrorKind>;
    async fn wait_for_read(&mut self) -> Result<(), ErrorKind>;
}

impl<I2C: I2cX, PINS> AsyncI2C for I2c<I2C, PINS> {}

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

        self.wait_idle();
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
                    self.wait_for_write(NoAcknowledgeSource::Address)?;
                    last_op_was_read = false;

                    // Write bytes
                    let n_bytes = bytes.len();
                    for (j, byte) in bytes.iter().enumerate() {
                        self.write_txr(*byte);
                        self.trigger_write(false, (i == n_ops - 1) && (j == n_bytes - 1));
                        self.wait_for_write(NoAcknowledgeSource::Data)?;
                    }
                }
                Operation::Read(buffer) => {
                    // Send read command
                    self.write_txr((address << 1) + FLAG_READ);
                    self.trigger_write(!last_op_was_read, false);
                    self.wait_for_write(NoAcknowledgeSource::Address)?;
                    last_op_was_read = true;

                    // Read bytes
                    let n_bytes = buffer.len();
                    for (j, byte) in buffer.iter_mut().enumerate() {
                        self.trigger_read(j == n_bytes - 1, (i == n_ops - 1) && (j == n_bytes - 1));
                        self.wait_for_read()?;
                        *byte = self.read_rxr();
                    }
                }
            }
        }
        self.wait_idle();

        Ok(())
    }
}
