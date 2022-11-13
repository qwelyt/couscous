#![no_std]
#![no_main]

#[allow(unused, unused_variables, unused_mut)]
use panic_halt as _;

#[rtic::app(device = seeeduino_xiao_rp2040::pac, dispatchers = [SPI0_IRQ])]
mod app {
    use core::iter::once;

    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use embedded_hal::prelude::_embedded_hal_blocking_spi_Write;
    use rp2040_hal::{Clock, gpio::DynPin, pac, pio::PIOExt, Timer};
    use rp2040_hal::gpio::{DynPinId, DynPinMode, PinId, PinMode};
    use rp2040_hal::pio::{SM0, StateMachine, StateMachineIndex, ValidStateMachine};
    use rp2040_hal::timer::CountDown;
    use rp2040_monotonic::{ExtU64, Rp2040Monotonic};
    use seeeduino_xiao_rp2040::entry;
    use seeeduino_xiao_rp2040::hal;
    use seeeduino_xiao_rp2040::pac::PIO0;
    use smart_leds::{brightness, RGB8, SmartLedsWrite};
    use ws2812_pio::{Ws2812, Ws2812Direct};

    #[shared]
    struct Shared {}

    // Set up pins with rows and cols
    // Loop thru them and mark which combos are active
    // As we have double cols, make sure to add extra index somehow
    // store the marked state in shared
    // run a task to check if state has changed and run task to send usb datagram
    #[local]
    struct Local {
        rows: [DynPin; 5],
        cols: [DynPin; 6],
        neo: Ws2812Direct<PIO0, SM0, hal::gpio::bank0::Gpio12>,
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MoMono = Rp2040Monotonic;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut pac = cx.device;
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
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

        let sio = hal::Sio::new(pac.SIO);
        let pins = seeeduino_xiao_rp2040::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
        let mut ws = Ws2812Direct::new(
            pins.neopixel_data.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
        );
        let mut neopower = pins.neopixel_power.into_push_pull_output();
        neopower.set_high().unwrap();
        let mono = rp2040_monotonic::Rp2040Monotonic::new(pac.TIMER);
        ws.write(brightness(once(wheel(20)), 32)).unwrap();

        scan_n_stuff::spawn_after(60.millis()).ok();

        (
            Shared {},
            Local {
                rows: [pins.a0.into(), pins.a1.into(), pins.a2.into(), pins.a3.into(), pins.sda.into()],
                cols: [pins.scl.into(), pins.tx.into(), pins.mosi.into(), pins.miso.into(), pins.sck.into(), pins.rx.into()],
                neo: ws,
            },
            init::Monotonics(mono)
        )
    }

    #[task(local = [rows, cols, neo])]
    fn scan_n_stuff(cx: scan_n_stuff::Context) {
        for row in cx.local.rows {
            row.into_push_pull_output();
        }
        for col in cx.local.cols {
            col.into_pull_down_input();
        }
        scan_n_stuff::spawn_after(60.millis()).ok();
    }

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
}