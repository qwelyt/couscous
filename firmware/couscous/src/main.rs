#![no_std]
#![no_main]

#[allow(unused, unused_variables, unused_mut)]
use panic_halt as _;

mod key_mapping;
mod position;

#[rtic::app(device = seeeduino_xiao_rp2040::pac, peripherals = true, dispatchers = [SPI0_IRQ, TIMER_IRQ_1])]
mod app {
    use core::fmt::{self, Write};
    use core::iter::once;

    use bsp::{
        hal::{
            self,
            clocks::Clock,
            gpio::DynPin,
            pio::{PIOExt, SM0},
        },
        pac::PIO0,
    };
    use cortex_m::asm::delay;
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use hash32::{BuildHasherDefault, FnvHasher};
    use heapless::{FnvIndexSet, IndexSet};
    use rp2040_monotonic::{ExtU64, Rp2040Monotonic};
    use seeeduino_xiao_rp2040 as bsp;
    use smart_leds::{brightness, RGB8, SmartLedsWrite};
    // USB Device support
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_hid::descriptor::KeyboardReport;
    use usbd_hid::descriptor::SerializedDescriptor;
    use usbd_hid::hid_class::{HIDClass, HidClassSettings, HidProtocol, HidSubClass, ProtocolModeConfig};
    use usbd_serial::SerialPort;
    use ws2812_pio::Ws2812Direct;

    use crate::key_mapping::{map_pos_to_key, meta_value};
    use crate::position::position::{Direction::{Col2Row, Row2Col}, Position};

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type AppMonotonic = Rp2040Monotonic;
    type Instant = <Rp2040Monotonic as rtic::Monotonic>::Instant;

    /// Wrapper around a usb-cdc SerialPort
    /// to be able to use the `write!()` macro with it
    pub struct DebugPort<'a>(SerialPort<'a, hal::usb::UsbBus>);

    impl<'a> Write for DebugPort<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            let _ = self.0.write(s.as_bytes());
            Ok(())
        }
    }

    #[shared]
    struct Shared {
        usb_device: UsbDevice<'static, hal::usb::UsbBus>,
        debug_port: DebugPort<'static>,
        hid_device: HIDClass<'static, hal::usb::UsbBus>,
        neo: Ws2812Direct<PIO0, SM0, bsp::hal::gpio::bank0::Gpio12>,
        neo_power: DynPin,
    }

    #[local]
    struct Local {
        rows: [DynPin; 5],
        cols: [DynPin; 6],
        current_state: IndexSet<Position, BuildHasherDefault<FnvHasher>, 64>,
        last_state: IndexSet<Position, BuildHasherDefault<FnvHasher>, 64>,
    }

    #[init(local = [
    // any local for `init()` will have a static lifetime
    usb_bus: Option < usb_device::bus::UsbBusAllocator < hal::usb::UsbBus >> = None,
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {

        // rtic exposes the pac as `cx.device`
        let mut pac = cx.device;

        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        // Configure the clocks
        // The default is to generate a 125 MHz system clock
        let clocks = hal::clocks::init_clocks_and_plls(
            bsp::XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
            .ok()
            .unwrap();

        // set up the pins in the right state for the pio
        // state machine
        let sio = hal::Sio::new(pac.SIO);
        let pins = bsp::Pins::new(
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
        ws.write(brightness(once(wheel(20)), 32)).unwrap();

        // we want to run the statemachine at 30MHz
        // since that's the max clock speed of the SRT400
        // shift registers
        // let divisor = f64::from(clocks.reference_clock.freq().raw()) / 30.0;


        // Set up the USB driver
        // We do this last in the init function so that we can enable interrupts as soon as
        // possible to make sure that `poll` is called as soon as possible. This is the recommended
        // workaround described here: https://docs.rs/rp2040-hal/latest/rp2040_hal/usb/index.html.
        // It might be better to just set `.max_packet_size_ep0(64)` as also described in the
        // documentation, instead of having racy code.
        let usb_bus = cx
            .local
            .usb_bus
            .insert(UsbBusAllocator::new(hal::usb::UsbBus::new(
                pac.USBCTRL_REGS,
                pac.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut pac.RESETS,
            )));

        // Set up the USB Communications Class Device driver for debugging
        let debug_port = DebugPort(SerialPort::new(usb_bus));

        let mut hid_settings = HidClassSettings::default();
        hid_settings.protocol = HidProtocol::Keyboard;
        hid_settings.subclass = HidSubClass::Boot;
        hid_settings.config = ProtocolModeConfig::ForceBoot;
        let hid_device = HIDClass::new_with_settings(
            usb_bus,
            KeyboardReport::desc(),
            20,
            hid_settings,
        );
        let mut blue: DynPin = pins.led_blue.into_push_pull_output().into();
        let mut red: DynPin = pins.led_red.into_push_pull_output().into();
        let mut green: DynPin = pins.led_green.into_push_pull_output().into();
        blue.set_low().unwrap();
        red.set_low().unwrap();
        green.set_low().unwrap();

        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("qwelyt")
            .product("couscous")
            .serial_number("TEST")
            .device_class(0)
            .build();

        let mono = Rp2040Monotonic::new(pac.TIMER);
        let now = monotonics::now();
        let next = now + 500.millis();
        runner::spawn_at(next, next).unwrap();

        (
            Shared {
                usb_device,
                debug_port,
                hid_device,
                neo: ws,
                neo_power: neopower.into(),
            },
            Local {
                rows: [pins.sda.into(), pins.a3.into(), pins.a2.into(), pins.a1.into(), pins.a0.into()],
                cols: [pins.scl.into(), pins.tx.into(), pins.rx.into(), pins.sck.into(), pins.miso.into(), pins.mosi.into()],
                current_state: FnvIndexSet::new(),
                last_state: FnvIndexSet::new(),
            },
            init::Monotonics(mono),
        )
    }

    #[task(
    priority = 2,
    local = [rows, cols, last_state, current_state, wheel_pos: u8 = 1, cnt: u8 = 0],
    shared = [hid_device, debug_port, neo]
    )]
    fn runner(mut cx: runner::Context, scheduled: Instant) {
        let _ = cx
            .shared
            .neo
            .lock(|ws| ws.write(brightness(once(wheel(*cx.local.wheel_pos)), 10)).unwrap());
        *cx.local.cnt = *cx.local.cnt + 1 % 100;
        if *cx.local.cnt == 0 {
            *cx.local.wheel_pos = *cx.local.wheel_pos + 1 % 255;
        }
        let rows = cx.local.rows;
        let cols = cx.local.cols;
        let last_state = cx.local.last_state;
        let state: IndexSet<Position, BuildHasherDefault<FnvHasher>, 64> = scan(rows, cols);
        if !(state.eq(last_state)) {
            (cx.shared.hid_device, cx.shared.debug_port)
                .lock(|hid, debug| {
                    write!(debug, "State was changed: {state:?}\r\n").unwrap();
                    write_keyboard(hid, debug, &state);
                });
            last_state.clear();
            for pos in state.iter() {
                last_state.insert(*pos).unwrap();
            }
        } else {
        }
        let next = scheduled + 1.millis();
        runner::spawn_at(next, next).unwrap();
    }

    fn scan(rows: &mut [DynPin; 5], cols: &mut [DynPin; 6]) -> IndexSet<Position, BuildHasherDefault<FnvHasher>, 64> {
        let scan1 = scan_matrix(rows, cols);
        // Debounce
        delay(20);
        let scan2 = scan_matrix(rows, cols);

        let mut state: IndexSet<Position, BuildHasherDefault<FnvHasher>, 64> = FnvIndexSet::new();
        let x: IndexSet<&Position, BuildHasherDefault<FnvHasher>, 64> = scan1.intersection(&scan2).collect();
        for pos in x.iter() {
            state.insert(**pos).unwrap();
        }
        state
    }

    fn scan_matrix(rows: &mut [DynPin; 5], cols: &mut [DynPin; 6]) -> IndexSet<Position, BuildHasherDefault<FnvHasher>, 64> {
        let mut pressed = FnvIndexSet::<Position, 64>::new();
        for (r, row) in rows.iter_mut().enumerate() {
            row.into_push_pull_output();
            row.set_high().unwrap();
            for (c, col) in cols.iter_mut().enumerate() {
                col.into_pull_down_input();
                if col.is_high().unwrap() {
                    let position = Position::new(
                        u8::try_from(r).unwrap(),
                        u8::try_from(c).unwrap(),
                        Row2Col,
                    );
                    pressed.insert(position).unwrap();
                }
                col.into_pull_down_disabled();
                delay(20);
            }
            delay(20);
            row.set_low().unwrap();
            row.into_pull_down_input();
            for (c, col) in cols.iter_mut().enumerate() {
                col.into_push_pull_output();
                col.set_high().unwrap();
                if row.is_high().unwrap() {
                    let position = Position::new(
                        u8::try_from(r).unwrap(),
                        u8::try_from(c).unwrap(),
                        Col2Row,
                    );
                    pressed.insert(position).unwrap();
                }
                col.set_low().unwrap();
            }
            delay(20);
        }
        pressed
    }

    /// This task is reponsible for polling for USB events
    /// whenever the USBCTRL_IRQ is raised
    #[task(binds = USBCTRL_IRQ, shared = [usb_device, debug_port, hid_device])]
    fn usbctrl_irq(cx: usbctrl_irq::Context) {
        (
            cx.shared.usb_device,
            cx.shared.debug_port,
            cx.shared.hid_device,
        )
            .lock(|usb_device, debug_port, hid_device| {
                if usb_device.poll(&mut [&mut debug_port.0, hid_device]) {
                    // DEBUG functionality, if "b" is received
                    // over the debug serial port we reser to bootloader
                    let mut buf = [0u8; 64];
                    if let Ok(x) = debug_port.0.read(&mut buf) {
                        let bytes = &buf[..x];
                        if bytes == b"b" {
                            hal::rom_data::reset_to_usb_boot(0, 0);
                        }
                        if bytes == b"a" {
                            write!(debug_port, "Yes hello\r\n").unwrap();
                        }
                        if bytes == b"c" {
                            write!(debug_port, "Do spawn\r\n").unwrap();
                            runner::spawn(monotonics::now()).unwrap();
                            write!(debug_port, "Done spawn\r\n").unwrap();
                        }
                    }
                }
            });
    }

    fn write_keyboard(hid: &mut HIDClass<'static, hal::usb::UsbBus>, debug: &mut DebugPort<'static>, state: &IndexSet<Position, BuildHasherDefault<FnvHasher>, 64>) {
        let mut report = KeyboardReport {
            modifier: 0,
            reserved: 0,
            leds: 0,
            keycodes: [0, 0, 0, 0, 0, 0],
        };

        let mut chars: u8 = 0;
        for pos in state {
            let key = map_pos_to_key(pos);
            let meta_value = meta_value(key);
            report.modifier = report.modifier | meta_value;
            if meta_value == 0 {
                report.keycodes[chars as usize] = map_pos_to_key(pos);
                chars = chars + 1;
            }
            if chars >= 6 {
                if let Err(e) = hid.push_input(&report) {
                    let _ = write!(debug, "got error when pushing hid: {e:?}\r\n");
                }
                report.keycodes = [0, 0, 0, 0, 0, 0];
                chars = 0;
            }
        }

        if let Err(e) = hid.push_input(&report) {
            let _ = write!(debug, "got error when pushing hid: {e:?}\r\n");
        }
    }


    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
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