#![no_std]
#![no_main]


use embedded_hal::digital::v2::OutputPin;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;
use rp2040_hal::gpio::{Output, Pin, PushPull};
use rp2040_hal::gpio::bank0::Gpio25;
use rp2040_hal::pac;
use rtic::app;
use seeeduino_xiao_rp2040::hal;
use systick_monotonic::{fugit::Duration, Systick};

#[rtic::app(device = seeeduino_xiao_rp2040::pac, dispatchers = [TIMER_IRQ_0])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: Pin<Gpio25, Output<PushPull>>,
        state: bool,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Grab our singleton objects
        let mut pac = pac::Peripherals::take().unwrap();

        // The single-cycle I/O block controls our GPIO pins
        let sio = hal::Sio::new(pac.SIO);

        // Set the pins up according to their function on this particular board
        let pins = seeeduino_xiao_rp2040::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let mono = Systick::new(cx.core.SYST, 36_000_000);

        let mut led: Pin<Gpio25, Output<PushPull>> = pins.led_blue.into_push_pull_output();
        led.set_low().unwrap();

        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();

        (
            Shared {},
            Local {
                led: led,
                state: false,
            },
            init::Monotonics(mono)
        )
    }

    #[task(local = [led, state])]
    fn blink(cx: blink::Context) {
        if *cx.local.state {
            cx.local.led.set_high().unwrap();
            *cx.local.state = false
        } else {
            cx.local.led.set_low().unwrap();
            *cx.local.state = true;
        }
        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
    }
}