//! On-board user LEDs
//!
//! Hifive1 (+ revB)
//! - Red = Pin 22
//! - Green = Pin 19
//! - Blue = Pin 21
//!
//! RedV
//! - Blue = Pin 5

use core::convert::Infallible;
#[cfg(feature = "board-redv")]
use e310x_hal::gpio::gpio0::Pin5;
#[cfg(any(feature = "board-hifive1", feature = "board-hifive1-revb"))]
use e310x_hal::gpio::gpio0::{Pin19, Pin21, Pin22};
use e310x_hal::{
    gpio::{Invert, Output, Regular},
    prelude::*,
};

#[cfg(any(feature = "board-hifive1", feature = "board-hifive1-revb"))]
/// Red LED
pub type RED = Pin22<Output<Regular<Invert>>>;

#[cfg(any(feature = "board-hifive1", feature = "board-hifive1-revb"))]
/// Green LED
pub type GREEN = Pin19<Output<Regular<Invert>>>;

#[cfg(any(feature = "board-hifive1", feature = "board-hifive1-revb"))]
/// Blue LED
pub type BLUE = Pin21<Output<Regular<Invert>>>;

#[cfg(feature = "board-redv")]
/// Blue LED
pub type BLUE = Pin5<Output<Regular<Invert>>>;

#[cfg(any(feature = "board-hifive1", feature = "board-hifive1-revb"))]
/// Returns RED, GREEN and BLUE LEDs.
pub fn rgb<X, Y, Z>(red: Pin22<X>, green: Pin19<Y>, blue: Pin21<Z>) -> (RED, GREEN, BLUE) {
    let red: RED = red.into_inverted_output();
    let green: GREEN = green.into_inverted_output();
    let blue: BLUE = blue.into_inverted_output();
    (red, green, blue)
}

/// Generic LED
pub trait Led: StatefulOutputPin<Error = Infallible> {
    /// Returns true if the LED is on
    fn is_on(&mut self) -> bool;

    /// Turns the LED off
    fn off(&mut self);

    /// Turns the LED on
    fn on(&mut self);

    /// Toggles the LED state
    fn toggle(&mut self) {
        StatefulOutputPin::toggle(self).unwrap();
    }
}

/// Macro to implement the Led trait for each of the board LEDs
macro_rules! led_impl {
    ($($LEDTYPE:ident),+) => {
        $(
            impl Led for $LEDTYPE {
                fn is_on(&mut self) -> bool {
                    self.is_set_low().unwrap()
                }

                fn off(&mut self) {
                    self.set_high().unwrap();
                }

                fn on(&mut self) {
                    self.set_low().unwrap();
                }
            }
        )+
    }
}

// Call the macro for each LED

#[cfg(any(feature = "board-hifive1", feature = "board-hifive1-revb"))]
led_impl!(RED, GREEN);

led_impl!(BLUE);
