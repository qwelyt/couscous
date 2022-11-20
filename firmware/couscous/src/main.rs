#![no_std]
#![no_main]

#[allow(unused, unused_variables, unused_mut)]
use panic_halt as _;

#[rtic::app(device = seeeduino_xiao_rp2040::pac, dispatchers = [SPI0_IRQ, TIMER_IRQ_1])]
mod app {
    use core::fmt::{self, Write};
    use core::hash::Hasher;
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
    use usbd_hid::hid_class::{HIDClass, HidClassSettings, HidCountryCode, HidProtocol, HidSubClass, ProtocolModeConfig};
    use usbd_serial::SerialPort;
    use ws2812_pio::Ws2812Direct;

    const REPORT_ID: u8 = 0x02;
    const DESCRIPTOR: &[u8] = &[
        0x05, 0x01,         // Usage Page (Generic Desktop),
        0x09, 0x06,         // Usage (Keyboard),
        0xA1, 0x01,         // Collection (Application),
        0x85, REPORT_ID,    //     REPORT_ID
        0x75, 0x01,         //     Report Size (1),
        0x95, 0x08,         //     Report Count (8),
        0x05, 0x07,         //     Usage Page (Key Codes),
        0x19, 0xE0,         //     Usage Minimum (224),
        0x29, 0xE7,         //     Usage Maximum (231),
        0x15, 0x00,         //     Logical Minimum (0),
        0x25, 0x01,         //     Logical Maximum (1),
        0x81, 0x02,         //     Input (Data, Variable, Absolute), ;Modifier byte
        0x95, 0x01,         //     Report Count (1),
        0x75, 0x08,         //     Report Size (8),
        0x81, 0x01,         //     Input (Constant), ;Reserved byte
        0x95, 0x05,         //     Report Count (5),
        0x75, 0x01,         //     Report Size (1),
        0x05, 0x08,         //     Usage Page (LEDs),
        0x19, 0x01,         //     Usage Minimum (1),
        0x29, 0x05,         //     Usage Maximum (5),
        0x91, 0x02,         //     Output (Data, Variable, Absolute), ;LED report
        0x95, 0x01,         //     Report Count (1),
        0x75, 0x03,         //     Report Size (3),
        0x91, 0x01,         //     Output (Constant), ;LED report padding
        0x95, 0x06,         //     Report Count (6),
        0x75, 0x08,         //     Report Size (8),
        0x15, 0x00,         //     Logical Minimum (0),
        0x26, 0xFF, 0x00,   //     Logical Maximum(255),
        0x05, 0x07,         //     Usage Page (Key Codes),
        0x19, 0x00,         //     Usage Minimum (0),
        0x2A, 0xFF, 0x00,   //     Usage Maximum (255),
        0x81, 0x00,         //     Input (Data, Array),
        0xC0,               // End Collection
    ];

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
        current_state: IndexSet<(u8, u8, bool), BuildHasherDefault<FnvHasher>, 64>,
        last_state: IndexSet<(u8, u8, bool), BuildHasherDefault<FnvHasher>, 64>,
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
        blue: DynPin,
        red: DynPin,
        green: DynPin,
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
        // hid_settings.locale = HidCountryCode::Swedish;
        let hid_device = HIDClass::new_with_settings(
            usb_bus,
            KeyboardReport::desc(),
            200,
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
        // write_keyboard::spawn_at(next, next).unwrap();

        (
            Shared {
                current_state: FnvIndexSet::new(),
                last_state: FnvIndexSet::new(),
                usb_device,
                debug_port,
                hid_device,
                neo: ws,
                neo_power: neopower.into(),
            },
            Local {
                rows: [pins.a0.into(), pins.a1.into(), pins.a2.into(), pins.a3.into(), pins.sda.into()],
                cols: [pins.scl.into(), pins.tx.into(), pins.mosi.into(), pins.miso.into(), pins.sck.into(), pins.rx.into()],
                blue,
                red,
                green,
            },
            init::Monotonics(mono),
        )
    }

    #[task(
    priority = 1,
    local = [rows, cols],
    shared = [current_state]
    )]
    fn scan(mut cx: scan::Context) {
        let rows = cx.local.rows;
        let cols = cx.local.cols;

        let scan1 = scan_matrix(rows, cols);
        // Debounce
        delay(20);
        let scan2 = scan_matrix(rows, cols);

        cx.shared.current_state.lock(|current_state| {
            current_state.clear();
            let x: IndexSet<&(u8, u8, bool), BuildHasherDefault<FnvHasher>, 64> = scan1.intersection(&scan2).collect();
            for pos in x.iter() {
                current_state.insert(**pos).unwrap();
            }
        });
    }

    #[task(
    priority = 1,
    local = [wheel_pos: u8 = 1, red, blue],
    shared = [current_state, last_state])
    ]
    fn check_state(cx: check_state::Context) {
        let current_state = cx.shared.current_state;
        let last_state = cx.shared.last_state;

        (current_state, last_state).lock(|current_state, last_state| {
            if *(&last_state.eq(&current_state)) {
                // Nothing has changed
                // cx.local.red.set_low().unwrap();
                // cx.local.green.set_low().unwrap();
                // cx.local.blue.set_low().unwrap();
            } else {
                // send_new_report();
                // cx.local.neo.write(brightness(once(wheel(*cx.local.wheel_pos)), 32)).unwrap();
                *cx.local.wheel_pos = *cx.local.wheel_pos + 10 % 255;
                // cx.local.red.set_high().unwrap();
                // cx.local.green.set_high().unwrap();
                // cx.local.blue.set_high().unwrap();
                last_state.clear();
                for pos in current_state.iter() {
                    last_state.insert(*pos).unwrap();
                }
            }
        });
    }

    fn scan_matrix(rows: &mut [DynPin; 5], cols: &mut [DynPin; 6]) -> IndexSet<(u8, u8, bool), BuildHasherDefault<FnvHasher>, 64> {
        let mut pressed = FnvIndexSet::<(u8, u8, bool), 64>::new();
        for (r, row) in rows.iter_mut().enumerate() {
            row.into_push_pull_output();
            row.set_high().unwrap();
            for (c, col) in cols.iter_mut().enumerate() {
                col.into_pull_down_input();
                if col.is_high().unwrap() {
                    pressed.insert((u8::try_from(r).unwrap(), u8::try_from(c).unwrap(), false)).unwrap();
                }
            }
            row.set_low().unwrap();
            row.into_pull_down_input();
            for (c, col) in cols.iter_mut().enumerate() {
                col.into_push_pull_output();
                col.set_high().unwrap();
                if row.is_high().unwrap() {
                    pressed.insert((u8::try_from(r).unwrap(), u8::try_from(c).unwrap(), true)).unwrap();
                }
                col.set_low().unwrap();
            }
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
                            write_keyboard::spawn().unwrap();
                            write!(debug_port, "Done spawn\r\n").unwrap();
                        }
                    }
                }
            });
    }

    #[task(
    priority = 2,
    shared = [debug_port, hid_device, last_state],
    )]
    fn write_keyboard(mut cx: write_keyboard::Context) {
        let _ = cx
            .shared
            .debug_port
            .lock(|dp| write!(dp, "in write_keyboard\r\n"));
        (cx.shared.hid_device, cx.shared.last_state)
            .lock(|hid, last_state| {
                // .lock(|hid: HIDClass<_>, neo: Ws2812Direct<_, _, _>, last_state: FnvIndexSet<_, 64>| {
                let mut report = KeyboardReport {
                    modifier: 0,
                    reserved: 0,
                    leds: 0,
                    keycodes: [0x04, 0, 0, 0, 0, 0],
                };
                report.keycodes = [0x04, 0, 0, 0, 0, 0];
                if let Err(e) = hid.push_input(&report) {
                    // *cx.local.wheel_pos = 255;
                    let _ = cx
                        .shared
                        .debug_port
                        .lock(|dp| write!(dp, "got error when pushing hid: {e:?}\r\n"));
                }
                // neo.write(brightness(once(wheel(*cx.local.wheel_pos)), 32)).unwrap();
            });
    }

    // #[task(
    // shared = [debug_port, hid_device, neo, neo_power, last_state],
    // local = [
    // report: [u8; 8] = [0, 0, 0, 0, 0, 0, 0, 0],
    // wheel_pos: u8 = 1,
    // green
    // ]
    // )]
    // fn write_keyboard(mut cx: write_keyboard::Context, scheduled: Instant) {
    //     let _ = cx
    //         .shared
    //         .debug_port
    //         .lock(|dp| write!(dp, "in write_keyboard"));
    //     // cx.local.green.set_low().unwrap();
    //     // let mut next = scheduled + 50.millis();
    //     // (cx.shared.hid_device, cx.shared.neo, cx.shared.neo_power, cx.shared.last_state)
    //     //     .lock(|hid, neo, neo_power, last_state| {
    //     //         // .lock(|hid: HIDClass<_>, neo: Ws2812Direct<_, _, _>, last_state: FnvIndexSet<_, 64>| {
    //     //         let mut report = KeyboardReport {
    //     //             modifier: 0,
    //     //             reserved: 0,
    //     //             leds: 0,
    //     //             keycodes: [0x04, 0, 0, 0, 0, 0],
    //     //         };
    //     //         // report.keycodes = [0x04, 0, 0, 0, 0, 0];
    //     //         let slice = 100 / 8;
    //     //         *cx.local.wheel_pos = *cx.local.wheel_pos + 10 % 255;
    //     //         // match hid.push_input(&report) {
    //     //         //     Ok(_) => *cx.local.wheel_pos = 200,
    //     //         //     Err(UsbError::InvalidState) => {
    //     //         //         *cx.local.wheel_pos = slice * 1;
    //     //         //     }
    //     //         //     Err(UsbError::WouldBlock) => {
    //     //         //         *cx.local.wheel_pos = slice * 2;
    //     //         //     }
    //     //         //     Err(UsbError::ParseError) => {
    //     //         //         *cx.local.wheel_pos = slice * 3;
    //     //         //     }
    //     //         //     Err(UsbError::BufferOverflow) => {
    //     //         //         *cx.local.wheel_pos = slice * 4;
    //     //         //     }
    //     //         //     Err(UsbError::EndpointOverflow) => *cx.local.wheel_pos = slice * 5,
    //     //         //     Err(UsbError::EndpointMemoryOverflow) => *cx.local.wheel_pos = slice * 6,
    //     //         //     Err(UsbError::InvalidEndpoint) => *cx.local.wheel_pos = slice * 7,
    //     //         //     Err(UsbError::Unsupported) => *cx.local.wheel_pos = slice * 8,
    //     //         // }
    //     //         // next = scheduled + 1000.millis();
    //     //         // if let Err(e) = hid.push_input(&report) {
    //     //         //     *cx.local.wheel_pos = 255;
    //     //         //     let _ = cx
    //     //         //         .shared
    //     //         //         .debug_port
    //     //         //         .lock(|dp| write!(dp, "got error when pushing hid: {e:?}\r\n"));
    //     //         // }
    //     //         neo.write(brightness(once(wheel(*cx.local.wheel_pos)), 32)).unwrap();
    //     //     });
    //     //
    //     // let _ = cx
    //     //     .shared
    //     //     .debug_port
    //     //     .lock(|dp| write!(dp, "in write_keyboard -- done"));
    //     // write_keyboard::spawn_at(next, next).unwrap();
    // }

    // #[task(
    // binds = PIO0_IRQ_0,
    // shared = [debug_port, hid_device, neo],
    // local = [
    // report: [u8; 8] = [0, 0, 0, 0, 0, 0, 0, 0],
    // wheel_pos: u8 = 1
    // ])]
    // fn pio0_irq_0(mut cx: pio0_irq_0::Context) {
    //     // use byteorder::{BigEndian, ByteOrder};
    //     // while let Some(pio_report) = cx.local.rx.read() {
    //     //     BigEndian::write_u32(&mut cx.local.report[1..], !pio_report);
    //     //     cx.local.report[3] &= 0xfe;
    //     //     cx.local.report[4] = 0;
    //     //     // FIXME: error handling when pushing reports
    //     //     cx.shared.hid_device.lock(|hid| {
    //     //         if let Err(e) = hid.push_raw_input(cx.local.report) {
    //     //             let _ = cx
    //     //                 .shared
    //     //                 .debug_port
    //     //                 .lock(|dp| write!(dp, "got error when pushing hid: {e:?}\r\n"));
    //     //         }
    //     //     });
    //     // }
    // }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // cortex_m::asm::wfi();
            // scan::spawn().unwrap();
            // check_state::spawn().unwrap();
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