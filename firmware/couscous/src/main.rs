#![no_std]
#![no_main]

use core::iter::once;

use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use rp2040_hal::{Clock, gpio::DynPin, pac, pio::PIOExt, Timer};
use seeeduino_xiao_rp2040::entry;
use seeeduino_xiao_rp2040::hal;
use smart_leds::{brightness, RGB8, SmartLedsWrite};
use ws2812_pio::Ws2812;

// use heapless::Vec;
// use rp2040_hal::gpio::{AnyPin, Output};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        seeeduino_xiao_rp2040::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    // Set the pins up according to their function on this particular board
    let pins = seeeduino_xiao_rp2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.neopixel_data.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );
    let mut neopower = pins.neopixel_power.into_push_pull_output();
    neopower.set_high().unwrap();


    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let green: DynPin = pins.led_green.into_push_pull_output().into();
    let blue: DynPin = pins.led_blue.into_push_pull_output().into();
    let red: DynPin = pins.led_red.into_push_pull_output().into();

    let x: DynPin = pins.a0.into_push_pull_output().into();

    let mut leds = [green, blue, red];

    for led in leds.iter_mut() {
        led.set_low().unwrap();
    }
    let mut pins = [x];
    for pin in pins.iter_mut() {
        pin.set_low().unwrap();
    }

    let mut n: u8 = 128;
    loop {
        ws.write(brightness(once(wheel(n)), 32)).unwrap();
        n = n.wrapping_add(1);

        for led in leds.iter_mut() {
            led.set_low().unwrap();
            delay.delay_ms(200);
        }
        for pin in pins.iter_mut() {
            pin.into_pull_down_input();
        }

        delay.delay_ms(500);
        for led in leds.iter_mut() {
            led.set_high().unwrap();
            delay.delay_ms(200);
        }
        for pin in pins.iter_mut() {
            pin.into_push_pull_output();
        }

        delay.delay_ms(500);
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}