#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use rp2040_hal::{Clock, pac};
use seeeduino_xiao_rp2040::entry;
use seeeduino_xiao_rp2040::hal;

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

    // Set the pins up according to their function on this particular board
    let pins = seeeduino_xiao_rp2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut green = pins.led_green.into_push_pull_output();
    let mut blue = pins.led_blue.into_push_pull_output();
    let mut red = pins.led_red.into_push_pull_output();
    green.set_low().unwrap();
    blue.set_low().unwrap();
    red.set_low().unwrap();
    // let mut leds : Vec<AnyPin<Id=_, Mode=_, Type=_>, 8>= Vec::new();
    // leds.push(green).unwrap();
    // leds.push(blue).unwrap();
    // leds.push(red).unwrap();

    // for led in leds.iter_mut() {
    //     led.set_low().unwrap();
    // }

    loop {
        // for led in leds.iter_mut() {
        //     led.set_low().unwrap();
        //     delay.delay_ms(200);
        // }
        green.set_high().unwrap();
        delay.delay_ms(500);
        blue.set_high().unwrap();
        delay.delay_ms(500);
        red.set_high().unwrap();

        delay.delay_ms(500);
        // for led in leds.iter_mut() {
        //     led.set_high().unwrap();
        //     delay.delay_ms(200);
        // }
        green.set_low().unwrap();
        delay.delay_ms(500);
        blue.set_low().unwrap();
        delay.delay_ms(500);
        red.set_low().unwrap();

        delay.delay_ms(500);
    }
}