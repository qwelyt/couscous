#![no_std]
#![no_main]

#[allow(unused, unused_variables, unused_mut)]
use panic_halt as _;

#[rtic::app(device = seeeduino_xiao_rp2040::pac, dispatchers = [SPI0_IRQ])]
mod app {
    use core::fmt::{self, Write};
    use core::iter::once;

    use bsp::hal::{
        self,
        clocks::Clock,
        gpio::{FunctionPio0, Pin},
        pio::{PinState, PIOExt},
    };
    use embedded_hal::digital::v2::OutputPin;
    use hash32::{BuildHasherDefault, FnvHasher};
    use heapless::{FnvIndexSet, IndexSet, Vec};
    use rp2040_hal::gpio::DynPin;
    use seeeduino_xiao_rp2040 as bsp;
    use smart_leds::{brightness, RGB8, SmartLedsWrite};
    // USB Device support
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_hid::hid_class::HIDClass;
    use usbd_serial::SerialPort;
    use ws2812_pio::{Ws2812, Ws2812Direct};

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
    }

    #[local]
    struct Local {
        rows: [DynPin; 5],
        cols: [DynPin; 6],
        // neo: Ws2812Direct<PIO0, SM0, gpio::bank0::Gpio12>,
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
        // we want to run the statemachine at 30MHz
        // since that's the max clock speed of the SRT400
        // shift registers
        let divisor = f64::from(clocks.reference_clock.freq().raw()) / 30.0;


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

        let hid_device = HIDClass::new(
            usb_bus,
            DESCRIPTOR,
            200);

        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("qwelyt")
            .product("couscous")
            .serial_number("TEST")
            .device_class(0)
            .build();

        (
            Shared {
                usb_device,
                debug_port,
                hid_device,
            },
            Local {
                rows: [pins.a0.into(), pins.a1.into(), pins.a2.into(), pins.a3.into(), pins.sda.into()],
                cols: [pins.scl.into(), pins.tx.into(), pins.mosi.into(), pins.miso.into(), pins.sck.into(), pins.rx.into()],
            },
            init::Monotonics(),
        )
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
                    }
                }
            });
    }

    /// This task is reponsible for reading data from the rx fifo
    /// of pio0, and sending HID reports to the connected computer
    #[task(binds = PIO0_IRQ_0, shared = [debug_port, hid_device], local = [
    rows, cols,
    report: [u8; 8] = [0x02, 0, 0, 0, 0, 0, 0, 0],
    ])]
    fn pio0_irq_0(mut cx: pio0_irq_0::Context) {
        cx.shared.hid_device.lock(|hid| {
            if let Err(e) = hid.push_raw_input(cx.local.report) {
                let _ = cx
                    .shared
                    .debug_port
                    .lock(|dp| write!(dp, "got error when pushing hid: {e:?}\r\n"));
            }
        });
        // use byteorder::{BigEndian, ByteOrder};
        // while let Some(pio_report) = cx.local.rx.read() {
        //     BigEndian::write_u32(&mut cx.local.report[1..], !pio_report);
        //     cx.local.report[3] &= 0xfe;
        //     cx.local.report[4] = 0;
        //     // FIXME: error handling when pushing reports
        //     cx.shared.hid_device.lock(|hid| {
        //         if let Err(e) = hid.push_raw_input(cx.local.report) {
        //             let _ = cx
        //                 .shared
        //                 .debug_port
        //                 .lock(|dp| write!(dp, "got error when pushing hid: {e:?}\r\n"));
        //         }
        //     });
        // }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}