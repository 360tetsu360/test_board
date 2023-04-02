//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bno055::{
    mint::EulerAngles, BNO055Calibration, BNO055CalibrationStatus, BNO055OperationMode,
    BNO055_CALIB_SIZE,
};
use bsp::entry;
use cortex_m::{asm::wfi, prelude::*};
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, i2c,
    pac::{self, interrupt},
    sio::Sio,
    spi,
    watchdog::Watchdog,
    Timer
};

use fugit::{RateExtU32, ExtU32};

use embedded_sdmmc::{Controller, SdMmcSpi, TimeSource, Timestamp, VolumeIdx};

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

use crate::flash::{FlashMemory, NAUTILUS_ID};

mod flash;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<bsp::hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<bsp::hal::usb::UsbBus>> = None;

#[derive(Default)]
pub struct DummyTimesource();

impl TimeSource for DummyTimesource {
    // In theory you could use the RTC of the rp2040 here, if you had
    // any external time synchronizing device.
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(bsp::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB Communications Class Device driver
    let serial = SerialPort::new(bus_ref);

    unsafe {
        USB_SERIAL = Some(serial);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("tmmgq2023a3")
        .product("Serial port")
        .serial_number("n4ut1lu5_robot")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(bsp::hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut flash_mem = FlashMemory::new();

    delay.delay_ms(1000);
    serial_out(b"Hello pc!\n");

    let sda_pin = pins.gpio0.into_mode::<gpio::FunctionI2C>();
    let scl_pin = pins.gpio1.into_mode::<gpio::FunctionI2C>();

    let i2c = i2c::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut imu = bno055::Bno055::new(i2c).with_alternative_address();

    imu.init(&mut delay).unwrap();
    serial_out(b"imu init\n");

    // Enable 9-degrees-of-freedom sensor fusion mode with fast magnetometer calibration
    imu.set_mode(BNO055OperationMode::NDOF, &mut delay).unwrap();
    serial_out(b"bno set mode\n");

    serial_out(b"reading flash\n");

    if flash_mem.initialized() {
        serial_out(b"found imu calibration data\n");

        let mut calibration_data = [0u8; BNO055_CALIB_SIZE];
        calibration_data.copy_from_slice(flash_mem.read_range(8..8 + BNO055_CALIB_SIZE));

        let calib = BNO055Calibration::from_buf(&calibration_data);
        imu.set_calibration_profile(calib, &mut delay).unwrap();
    } else {
        serial_out(b"calibration start\n");

        let mut status = imu.get_calibration_status().unwrap();
        serial_out_imu_stat(status);

        serial_out(b"- About to begin BNO055 IMU calibration...\n");
        while !imu.is_fully_calibrated().unwrap() {
            status = imu.get_calibration_status().unwrap();
            delay.delay_ms(1000);
            serial_out_imu_stat(status);
        }

        let calib = imu.calibration_profile(&mut delay).unwrap();
        imu.set_calibration_profile(calib, &mut delay).unwrap();
        serial_out(b"calibration complete!\n");

        serial_out(b"saving calibration data\n");
        let calib = imu.calibration_profile(&mut delay).unwrap();
        serial_out(b"got calibration profile\n");
        flash_mem.write_from(NAUTILUS_ID.len().., calib.as_bytes());
        serial_out(b"copied!\n");
    }
    //flash_mem.save();

    let _spi_sclk = pins.gpio10.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio12.into_mode::<gpio::FunctionSpi>();
    let spi_cs = pins.gpio9.into_push_pull_output();
    let spi = spi::Spi::<_, _, 8>::new(pac.SPI1);
    serial_out(b"init spi pins\n");

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    serial_out(b"init spi\n");

    let mut sdspi = SdMmcSpi::new(spi, spi_cs);
    serial_out(b"sdmmspi init\n");

    let block = match sdspi.acquire() {
        Ok(block) => block,
        Err(_) => {
            serial_out(b"breakpoint 1\n");
            loop {wfi()}
        }
    };

    let mut cont = Controller::new(block, DummyTimesource::default());

    match cont.device().card_size_bytes() {
        Ok(size) => info!("card size is {} bytes", size),
        Err(_) => {
            serial_out(b"breakpoint 1.5\n");
            loop {wfi()}
        }
    }

    let volume = match cont.get_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(_) => {
            serial_out(b"breakpoint 2\n");
            loop {wfi()}
        }
    };
    let dir = match cont.open_root_dir(&volume) {
        Ok(dir) => dir,
        Err(_) => {
            serial_out(b"breakpoint 3\n");
            loop {wfi()}
        }
    };

    cont.iterate_dir(&volume, &dir, |ent| {
        serial_out(ent.name.base_name());
        serial_out(ent.name.extension());
    })
    .unwrap();

    let mut euler_angles: EulerAngles<f32, ()>; // = EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]);
                                                //let mut quaternion: Quaternion<f32>; // = Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]);

    loop {
        count_down.start(100.millis());
        // Euler angles, directly read
        match imu.euler_angles() {
            Ok(val) => {
                euler_angles = val;
                //serial_out(b"euler angles:\n");
                serial_out(&[0xFE]);
                serial_out_f32(euler_angles.a);
                serial_out_f32(euler_angles.b);
                serial_out_f32(euler_angles.c);
            }
            Err(_) => {
                serial_out(b"something got error! e\n");
            }
        }

        let _ = nb::block!(count_down.wait());
    }
}

fn serial_out(buf: &[u8]) {
    unsafe {
        let serial = USB_SERIAL.as_mut().unwrap();
        _ = serial.write(buf);
    }
}

fn serial_out_imu_stat(status: BNO055CalibrationStatus) {
    serial_out(b"calibration status\n");
    let acc_calib_stat_msg = [
        b'a',
        b'c',
        b'c',
        b' ',
        b':',
        b' ',
        status.acc + 0x30,
        b'\n',
    ];
    let gyr_calib_stat_msg = [
        b'g',
        b'y',
        b'r',
        b' ',
        b':',
        b' ',
        status.gyr + 0x30,
        b'\n',
    ];
    let mag_calib_stat_msg = [
        b'm',
        b'a',
        b'g',
        b' ',
        b':',
        b' ',
        status.mag + 0x30,
        b'\n',
    ];
    let sys_calib_stat_msg = [
        b's',
        b'y',
        b's',
        b' ',
        b':',
        b' ',
        status.sys + 0x30,
        b'\n',
    ];
    serial_out(&acc_calib_stat_msg[..]);
    serial_out(&gyr_calib_stat_msg[..]);
    serial_out(&mag_calib_stat_msg[..]);
    serial_out(&sys_calib_stat_msg[..]);
}

fn serial_out_f32(v: f32) {
    let bytes = v.to_be_bytes();
    serial_out(&[0xF3]);
    serial_out(&bytes);
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 12];
        match serial.read(&mut buf) {
            Err(_e) => {
                // Do nothing
            }
            Ok(0) => {
                // Do nothing
            }
            Ok(_) => {}
        }
    }
}
