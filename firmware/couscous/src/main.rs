#![no_std]
#![no_main]

#[allow(unused, unused_variables, unused_mut)]
use panic_halt as _;

#[rtic::app(device = seeeduino_xiao_rp2040::pac, dispatchers = [PIO0_IRQ_0])]
mod app {
    use core::iter::once;

    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use embedded_hal::prelude::_embedded_hal_blocking_spi_Write;
    use rp2040_hal::{Clock, gpio::DynPin, pac, pio::PIOExt, Timer};
    use rp2040_hal::gpio::{DynPinId, DynPinMode, PinId, PinMode};
    use rp2040_hal::pio::{SM0, StateMachine, StateMachineIndex, ValidStateMachine};
    use rp2040_hal::timer::CountDown;
    use seeeduino_xiao_rp2040::entry;
    use seeeduino_xiao_rp2040::hal;
    use seeeduino_xiao_rp2040::pac::PIO0;
    use smart_leds::{brightness, RGB8, SmartLedsWrite};
    use systick_monotonic::{fugit::Duration, Systick};
    use ws2812_pio::Ws2812;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        pin5: DynPin,
        pin6: DynPin,
        neo: Ws2812<PIO0, SM0, CountDown<'_>, dyn PinId<Reset=<(PinMode + 'static) as Trait>::Output>>,
    }

    #[init(
    local = []
    )]
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
        let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
        let ws = Ws2812::new(
            pins.neopixel_data.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
            timer.count_down(),
        );
        let mut neopower = pins.neopixel_power.into_push_pull_output();
        neopower.set_high().unwrap();
        scan_n_stuff::spawn().ok();

        (
            Shared {},
            Local {
                pin5: pins.sda.into(),
                pin6: pins.scl.into(),
                neo: ws.into(),
            },
            init::Monotonics()
        )
    }

    #[task(local = [pin5, pin6, neo])]
    fn scan_n_stuff(mut cx: scan_n_stuff::Context) {
        *cx.local.pin6.into_pull_down_input();
        *cx.local.pin5.into_push_pull_output();
        *cx.local.pin5.set_high().unwrap();
        let g = cx.local.pin5_is_high().unwrap();

        *cx.local.pin5.into_pull_down_input();
        *cx.local.pin6.into_push_pull_output();
        *cx.local.pin6.set_high().unwrap();
        let b = cx.local.pin6_is_high().unwrap();

        if b && g {
            cx.local.neo.write(brightness(once(wheel(192)), 32)).unwrap();
        } else if b {
            cx.local.neo.write(brightness(once(wheel(128)), 32)).unwrap();
        } else if g {
            cx.local.neo.write(brightness(once(wheel(64)), 32)).unwrap();
        } else {
            cx.local.neo.write(brightness(once(wheel(255)), 32)).unwrap();
        }
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