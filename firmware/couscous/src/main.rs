#![no_std]
#![no_main]

#[allow(unused, unused_variables, unused_mut)]
use panic_halt as _;

#[rtic::app(device = seeeduino_xiao_rp2040::pac, dispatchers = [SPI0_IRQ])]
mod app {
    use core::iter::once;

    use cortex_m::asm::delay;
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use embedded_hal::prelude::_embedded_hal_blocking_spi_Write;
    use hash32::{BuildHasherDefault, FnvHasher};
    use heapless::{FnvIndexSet, IndexSet, Vec};
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
    struct Shared {
        current_state: IndexSet<(u8, u8), BuildHasherDefault<FnvHasher>, 64>,
        last_state: IndexSet<(u8, u8), BuildHasherDefault<FnvHasher>, 64>,
    }

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

        scan::spawn_after(60.millis()).ok();

        (
            Shared {
                current_state: FnvIndexSet::new(),
                last_state: FnvIndexSet::new(),
            },
            Local {
                rows: [pins.a0.into(), pins.a1.into(), pins.a2.into(), pins.a3.into(), pins.sda.into()],
                cols: [pins.scl.into(), pins.tx.into(), pins.mosi.into(), pins.miso.into(), pins.sck.into(), pins.rx.into()],
                neo: ws,
            },
            init::Monotonics(mono)
        )
    }

    #[task(local = [rows, cols], shared = [current_state])]
    fn scan(mut cx: scan::Context) {
        let rows = cx.local.rows;
        let cols = cx.local.cols;

        let scan1 = scan_matrix(rows, cols);
        // Debounce
        delay(20);
        let scan2 = scan_matrix(rows, cols);

        cx.shared.current_state.lock(|current_state| {
            current_state.clear();
            let x: IndexSet<&(u8, u8), BuildHasherDefault<FnvHasher>, 64> = scan1.intersection(&scan2).collect();
            for pos in x.iter() {
                current_state.insert(**pos).unwrap();
            }
        });
        check_state::spawn().ok();
    }

    #[task(local = [neo, wheel_pos: u8 = 1], shared = [current_state, last_state])]
    fn check_state(mut cx: check_state::Context) {
        let current_state = cx.shared.current_state;
        let last_state = cx.shared.last_state;

        (current_state, last_state).lock(|current_state, last_state| {
            if *(&last_state.eq(&current_state)) {
                // Nothing has changed
            } else {
                // send_new_report();
                cx.local.neo.write(brightness(once(wheel(*cx.local.wheel_pos)), 32)).unwrap();
                *cx.local.wheel_pos = *cx.local.wheel_pos + 10 % 255;
                last_state.clear();
                for pos in current_state.iter() {
                    last_state.insert(*pos).unwrap();
                }
            }
        });

        scan::spawn().ok();
    }

    fn scan_matrix(rows: &mut [DynPin; 5], cols: &mut [DynPin; 6]) -> IndexSet<(u8, u8), BuildHasherDefault<FnvHasher>, 64> {
        let mut pressed = FnvIndexSet::<(u8, u8), 64>::new();
        for (r, row) in rows.iter_mut().enumerate() {
            row.into_push_pull_output();
            row.set_high().unwrap();
            for (c, col) in cols.iter_mut().enumerate() {
                col.into_pull_down_input();
                if col.is_high().unwrap() {
                    pressed.insert((u8::try_from(r).unwrap(), u8::try_from(c).unwrap())).unwrap();
                }
            }
        }
        pressed
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