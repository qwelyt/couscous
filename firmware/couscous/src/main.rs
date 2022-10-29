#![no_std]
#![no_main]

#[allow(unused, dead_code, unused_variables, unused_variables)]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

#[rtic::app(device = seeeduino_xiao_rp2040::pac)]
mod app {
    use core::fmt::{self, Write};

    use embedded_hal::digital::v2::OutputPin;
    use seeeduino_xiao_rp2040::hal::{
        self,
        clocks::Clock,
        //gpio::{FunctionPio0, Pin},
        //pio::{PIOExt, PinState},
    };
    use seeeduino_xiao_rp2040::pac;
    // USB Device support
    // use usb_device::{class_prelude::*, prelude::*};
    // use usbd_hid::hid_class::HIDClass;
    use usbd_serial::SerialPort;

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
        // usb_device: UsbDevice<'static, hal::usb::UsbBus>,
        _debug_port: DebugPort<'static>,
        // hid_device: HIDClass<'static, hal::usb::UsbBus>,
    }

    #[local]
    struct Local {
        /// The RX fifo of the PIO state machine
        _rx: seeeduino_xiao_rp2040::hal::pio::Rx<(
            seeeduino_xiao_rp2040::pac::PIO0,
            seeeduino_xiao_rp2040::hal::pio::SM0,
        )>,
    }

    #[init(local = [
    // any local for `init()` will have a static lifetime
    usb_bus: Option < usb_device::bus::UsbBusAllocator < hal::usb::UsbBus >> = None,
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // rtic exposes the pac as `cx.device`
        let mut pac = cx.device;
        let core = pac::CorePeripherals::take().unwrap();

        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        // Configure the clocks
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

        // set up the pins in the right state for the pio
        // state machine
        let sio = hal::Sio::new(pac.SIO);
        let pins = seeeduino_xiao_rp2040::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        // The delay object lets us wait for specified amounts of time (in
        // milliseconds)
        let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

        // Init PWMs
        let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

        // Configure PWM4
        let pwm = &mut pwm_slices.pwm0;
        pwm.set_ph_correct();
        pwm.enable();

        // Set the LEDs to be output, initially off
        let mut led_red_pin = pins.led_red.into_push_pull_output();
        let mut led_green_pin = pins.led_green.into_push_pull_output();
        let mut led_blue_pin = pins.led_blue.into_push_pull_output();
        led_red_pin.set_high().unwrap();
        led_green_pin.set_high().unwrap();
        led_blue_pin.set_high().unwrap();

        // Set up the USB driver
        // We do this last in the init function so that we can enable interrupts as soon as
        // possible to make sure that `poll` is called as soon as possible. This is the recommended
        // workaround described here: https://docs.rs/rp2040-hal/latest/rp2040_hal/usb/index.html.
        // It might be better to just set `.max_packet_size_ep0(64)` as also described in the
        // documentation, instead of having racy code.
        // let usb_bus = cx
        //     .local
        //     .usb_bus
        //     .insert(UsbBusAllocator::new(hal::usb::UsbBus::new(
        //         pac.USBCTRL_REGS,
        //         pac.USBCTRL_DPRAM,
        //         clocks.usb_clock,
        //         true,
        //         &mut pac.RESETS,
        //     )));
        //
        // // Set up the USB Communications Class Device driver for debugging
        // let _debug_port = DebugPort(SerialPort::new(usb_bus));
        //
        // // This is the hid-descriptor for the plover-hid protocol
        // // see https://github.com/dnaq/plover-machine-hid
        // const DESCRIPTOR: &[u8] = &[
        //     //  Keyboard
        //     0x05, 0x01, // USAGE_PAGE (Generic Desktop)
        //     0x09, 0x06, // USAGE (Keyboard)
        //     0xa1, 0x01, // COLLECTION (Application)
        //     0x85, 0x01, //   REPORT_ID (2)
        //     0x05, 0x07, //   USAGE_PAGE (Keyboard)
        //     0x19, 0xe0, //   USAGE_MINIMUM (Keyboard LeftControl)
        //     0x29, 0xe7, //   USAGE_MAXIMUM (Keyboard RightGUI)
        //     0x15, 0x00, //   LOGICAL_MINIMUM (0)
        //     0x25, 0x01, //   LOGICAL_MAXIMUM (1)
        //     0x75, 0x01, //   REPORT_SIZE (1)
        //     0x95, 0x08, //   REPORT_COUNT (8)
        //     0x81, 0x02, //   INPUT (Data,Var,Abs)
        //     0x95, 0x01, //   REPORT_COUNT (1)
        //     0x75, 0x08, //   REPORT_SIZE (8)
        //     0x81, 0x03, //   INPUT (Cnst,Var,Abs)
        //     0x95, 0x06, //   REPORT_COUNT (6)
        //     0x75, 0x08, //   REPORT_SIZE (8)
        //     0x15, 0x00, //   LOGICAL_MINIMUM (0)
        //     0x25, 0x65, //   LOGICAL_MAXIMUM (101)
        //     0x05, 0x07, //   USAGE_PAGE (Keyboard)
        //     0x19, 0x00, //   USAGE_MINIMUM (Reserved (no event indicated))
        //     0x29, 0x65, //   USAGE_MAXIMUM (Keyboard Application)
        //     0x81, 0x00, //   INPUT (Data,Ary,Abs)
        //     0xc0, // END_COLLECTION
        // ];
        // let _hid_device = HIDClass::new(usb_bus, DESCRIPTOR, 20);
        //
        // let _usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        //     .manufacturer("qwelyt")
        //     .product("couscous")
        //     .serial_number("TEST")
        //     .device_class(0)
        //     .build();

        loop {
            // Blink LEDs at 1 Hz
            for _ in 0..5 {
                // On
                led_red_pin.set_low().unwrap();
                delay.delay_ms(500);
                led_green_pin.set_low().unwrap();
                delay.delay_ms(500);
                led_blue_pin.set_low().unwrap();
                delay.delay_ms(500);

                // Off
                led_red_pin.set_high().unwrap();
                delay.delay_ms(500);
                led_green_pin.set_high().unwrap();
                delay.delay_ms(500);
                led_blue_pin.set_high().unwrap();
                delay.delay_ms(500);
            }
        }

        //(
        //    Shared {
        //        usb_device,
        //        debug_port,
        //        hid_device,
        //    },
        //    Local { rx },
        //    init::Monotonics(),
        //)
    }

    // /// This task is reponsible for polling for USB events
    // /// whenever the USBCTRL_IRQ is raised
    // #[task(binds = USBCTRL_IRQ, shared = [usb_device, debug_port, hid_device])]
    // fn usbctrl_irq(cx: usbctrl_irq::Context) {
    //     (
    //         cx.shared.usb_device,
    //         cx.shared.debug_port,
    //         cx.shared.hid_device,
    //     )
    //         .lock(|usb_device, debug_port, hid_device| {
    //             if usb_device.poll(&mut [&mut debug_port.0, hid_device]) {
    //                 // DEBUG functionality, if "b" is received
    //                 // over the debug serial port we reser to bootloader
    //                 let mut buf = [0u8; 64];
    //                 if let Ok(x) = debug_port.0.read(&mut buf) {
    //                     let bytes = &buf[..x];
    //                     if bytes == b"b" {
    //                         hal::rom_data::reset_to_usb_boot(0, 0);
    //                     }
    //                 }
    //             }
    //         });
    // }

    /// This task is reponsible for reading data from the rx fifo
    /// of pio0, and sending HID reports to the connected computer
    // #[task(binds = PIO0_IRQ_0, shared = [debug_port, hid_device], local = [
    // rx,
    // report: [u8; 9] = [80, 0, 0, 0, 0, 0, 0, 0, 0],
    // ])]
    // fn pio0_irq_0(mut cx: pio0_irq_0::Context) {
    //     use byteorder::{BigEndian, ByteOrder};
    //     while let Some(pio_report) = cx.local.rx.read() {
    //         let _ = cx
    //             .shared
    //             .debug_port
    //             .lock(|dp| write!(dp, "got report: 0b{pio_report:032b}\r\n"));
    //         BigEndian::write_u32(&mut cx.local.report[1..], !pio_report);
    //         cx.local.report[3] &= 0xfe;
    //         cx.local.report[4] = 0;
    //         // FIXME: error handling when pushing reports
    //         cx.shared.hid_device.lock(|hid| {
    //             if let Err(e) = hid.push_raw_input(cx.local.report) {
    //                 let _ = cx
    //                     .shared
    //                     .debug_port
    //                     .lock(|dp| write!(dp, "got error when pushing hid: {e:?}\r\n"));
    //             }
    //         });
    //     }
    // }
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}

// End of file
