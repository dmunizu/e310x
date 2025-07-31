//! Asynchronous HAL for the E310x family of microcontrollers
//!
//! This is an implementation of the [`embedded-hal-async`] traits for the E310x
//! family of microcontrollers.

#![deny(missing_docs)]

pub mod delay;
pub mod digital;
pub mod i2c;
pub mod prelude;
pub mod serial;
pub mod spi;

#[cfg(feature = "embassy")]
#[path = "asynch/embassy/time-driver.rs"]
pub mod time_driver;
