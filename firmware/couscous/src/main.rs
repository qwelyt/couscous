#![no_std]
#![no_main]


#[allow(unused, unused_variables, unused_mut)]
use panic_halt as _;

mod rp2040_monotonic;

#[rtic::app(device = seeeduino_xiao_rp2040::pac, dispatchers = [SPI0_IRQ])]
mod app {
    use core::fmt;
    use core::fmt::Write;

    use cortex_m::asm::delay;
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use hash32::{BuildHasherDefault, FnvHasher};
    use heapless::{FnvIndexSet, IndexSet};
    // use rp2040_hal::pac::PIO0;
    // use rp2040_hal::{Clock, gpio::DynPin, pac, pio::PIOExt, Sio, Timer};
    // use rp2040_hal::gpio::{DynPinId, DynPinMode, PinId, PinMode};
    // use rp2040_hal::pio::{SM0, StateMachine, StateMachineIndex, ValidStateMachine};
    // use rp2040_hal::timer::CountDown;
    // use rp2040_monotonic::{ExtU64, Rp2040Monotonic};
    // use systick_monotonic::{ExtU64, Systick};
    use seeeduino_xiao_rp2040::{
        hal::{
            self,
            gpio::DynPin,
            Sio,
        },
    };
    // use smart_leds::{brightness, RGB8,};
    // USB Device support
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_hid::descriptor::KeyboardReport;
    use usbd_hid::hid_class::HIDClass;
    use usbd_serial::SerialPort;

    // use ws2812_pio::{Ws2812, Ws2812Direct};
    use crate::rp2040_monotonic::{ExtU64, Rp2040Monotonic};

// use core::iter::once;

    const REPORT_ID: u8 = 0x01;

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
        // neo: Ws2812Direct<PIO0, SM0, gpio::bank0::Gpio12>,
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MoMono = Rp2040Monotonic;

    #[init(local = [
    // any local for `init()` will have a static lifetime
    usb_bus: Option < usb_device::bus::UsbBusAllocator < hal::usb::UsbBus >> = None,
    ])]
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

        let sio = Sio::new(pac.SIO);
        let pins = seeeduino_xiao_rp2040::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
        // let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
        // let mut ws = Ws2812Direct::new(
        //     pins.neopixel_data.into_mode(),
        //     &mut pio,
        //     sm0,
        //     clocks.peripheral_clock.freq(),
        // );
        // let mut neopower = pins.neopixel_power.into_push_pull_output();
        // neopower.set_high().unwrap();
        let mono = Rp2040Monotonic::new(pac.TIMER);
        // ws.write(brightness(once(wheel(20)), 32)).unwrap();

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

        const DESCRIPTOR: &[u8] = &[
            //  Keyboard
            0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
            0x09, 0x06,                    // USAGE (Keyboard)
            0xa1, 0x01,                    // COLLECTION (Application)
            0x85, REPORT_ID,               //   REPORT_ID
            0x05, 0x07,                    //   USAGE_PAGE (Keyboard)

            0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
            0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard RightGUI)
            0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
            0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
            0x75, 0x01,                    //   REPORT_SIZE (1)

            0x95, 0x08,                      //   REPORT_COUNT (8)
            0x81, 0x02,                    //   INPUT (Data,Var,Abs)
            0x95, 0x01,                    //   REPORT_COUNT (1)
            0x75, 0x08,                    //   REPORT_SIZE (8)
            0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)

            0x95, 0x06,                      //   REPORT_COUNT (6)
            0x75, 0x08,                    //   REPORT_SIZE (8)
            0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
            0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
            0x05, 0x07,                    //   USAGE_PAGE (Keyboard)

            0x19, 0x00,                      //   USAGE_MINIMUM (Reserved (no event indicated))
            0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
            0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
            0xc0,                          // END_COLLECTION
        ];
        let hid_device = HIDClass::new(usb_bus, DESCRIPTOR, 20);

        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("qwelyt")
            .product("couscous")
            .serial_number("0.01")
            .device_class(0)
            .build();

        scan::spawn_after(60.millis()).ok();

        (
            Shared {
                current_state: FnvIndexSet::new(),
                last_state: FnvIndexSet::new(),
                usb_device,
                debug_port,
                hid_device,
            },
            Local {
                rows: [pins.a0.into(), pins.a1.into(), pins.a2.into(), pins.a3.into(), pins.sda.into()],
                cols: [pins.scl.into(), pins.tx.into(), pins.mosi.into(), pins.miso.into(), pins.sck.into(), pins.rx.into()],
                // neo: ws,
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
            let x: IndexSet<&(u8, u8, bool), BuildHasherDefault<FnvHasher>, 64> = scan1.intersection(&scan2).collect();
            for pos in x.iter() {
                current_state.insert(**pos).unwrap();
            }
        });
        check_state::spawn().ok();
    }

    #[task(local = [wheel_pos: u8 = 1], shared = [current_state, last_state])]
    fn check_state(mut cx: check_state::Context) {
        let current_state = cx.shared.current_state;
        let last_state = cx.shared.last_state;

        (current_state, last_state).lock(|current_state, last_state| {
            if *(&last_state.eq(&current_state)) {
                // Nothing has changed
            } else {
                // send_new_report();
                // cx.local.neo.write(brightness(once(wheel(*cx.local.wheel_pos)), 32)).unwrap();
                *cx.local.wheel_pos = *cx.local.wheel_pos + 10 % 255;
                last_state.clear();
                for pos in current_state.iter() {
                    last_state.insert(*pos).unwrap();
                }
            }
        });

        scan::spawn().ok();
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

    // fn wheel(mut wheel_pos: u8) -> RGB8 {
    //     wheel_pos = 255 - wheel_pos;
    //     if wheel_pos < 85 {
    //         // No green in this sector - red and blue only
    //         (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    //     } else if wheel_pos < 170 {
    //         // No red in this sector - green and blue only
    //         wheel_pos -= 85;
    //         (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    //     } else {
    //         // No blue in this sector - red and green only
    //         wheel_pos -= 170;
    //         (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    //     }
    // }

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
                    }
                }
            });
    }

    /// This task is responsible for sending HID reports to the connected computer
    #[task(
    binds = PIO0_IRQ_0,
    shared = [debug_port, hid_device, current_state],
    local = [
    last_seen_state: IndexSet < (u8, u8, bool), BuildHasherDefault < FnvHasher >, 64 > = FnvIndexSet::new(),
    last_report: [u8; 8] = [0, 0, 0, 0, 0, 0, 0, 0],
    ],
    )]
    fn pio0_irq_0(mut cx: pio0_irq_0::Context) {
        // Check if teh last report we sent matches what state we have now
        // It it's the same, do nothing (or send the same? probably not)
        // Or perhaps store the last state we saw and match that. Since one state
        // could fill up more than one report. We need to be sure we only act on new
        // states. But then, why do we do check_state if we want to change do a check here
        // Or even more: Why do any work at all unless we trigger this task? When this task
        // gets triggerd, we could do the entire scan and check state and send that. That ought
        // to work just as well. As long as the scanning is not to slow. Then we want to do it
        // in between.
        cx.shared.current_state.lock(|current_state| {
            if *(&cx.local.last_seen_state.eq(&current_state)) {
                //
            } else {
                let report: [u8; 8] = [REPORT_ID, 0, 0x04, 0, 0, 0, 0, 0];
                cx.shared.hid_device.lock(|hid| {
                    let kr = KeyboardReport {
                        modifier: 0,
                        reserved: 0,
                        leds: 0,
                        keycodes: [0x04, 0, 0, 0, 0, 0],
                    };
                    if let Err(e) = hid.push_input(&kr) {
                        let _ = cx
                            .shared
                            .debug_port
                            .lock(|dp| write!(dp, "got error when pushing hid: {e:?}\r\n"));
                    }
                });
                cx.local.last_seen_state.clear();
                for pos in current_state.iter() {
                    cx.local.last_seen_state.insert(*pos).unwrap();
                }
            }
        })
    }
}