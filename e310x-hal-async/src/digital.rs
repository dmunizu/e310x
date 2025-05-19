//! # Digital I/O
//! # Note
//!
//! Implementation of the Async Embedded HAL I/O functionality.
use embedded_hal_async::digital::{InputPin, OutputPin, StatefulOutputPin};
use gpio::{Input, Output};

pub struct Input<I> {
    inner: I,
}

impl<I> Input<I> {
    pub fn new(inner: I) -> Self {
        Input { inner }
    }
}

pub struct Output<O> {
    inner: O,
}

impl<O> Output<O> {
    pub fn new(inner: O) -> Self {
        Output { inner }
    }
}

impl<P: gpio::Pin> InputPin for Input<P> {
    async fn is_high(&self) -> Result<bool, Self::Error> {
        self.pin.is_high()
    }

    async fn is_low(&self) -> Result<bool, Self::Error> {
        self.pin.is_low()
    }
}

impl<P: gpio::Pin> OutputPin for Output<P> {
    async fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high()
    }

    async fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low()
    }
}
