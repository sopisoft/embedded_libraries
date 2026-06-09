#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example connects an RP2350 board to the Adafruit
// LSM6DS3TR-C + LIS3MDL 9-DoF breakout over I2C and performs:
//
// 1. attitude estimation with the Madgwick filter,
// 2. relative altitude estimation by integrating gravity-compensated
//    vertical acceleration.
//
// Important limitation:
// - There is no barometer on this board.
// - The "altitude" printed by this example is therefore only a relative,
//   IMU-integrated estimate and will drift over time.
//
// Suggested wiring for a Pico 2 style board:
// - GPIO18 -> STEMMA QT SDA
// - GPIO19 -> STEMMA QT SCL
// - 3V3    -> VIN
// - GND    -> GND
// - GPIO0  -> UART0 TX for debug prints
// - GPIO1  -> UART0 RX for debug console (optional)
//
// The Adafruit board defaults are:
// - LSM6DS3TR-C: 0x6A
// - LIS3MDL:     0x1C
//
// If you close the address jumpers, change the constants below to:
// - LSM6DS3TR-C: 0x6B
// - LIS3MDL:     0x1E

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_example {
    use core::{cell::RefCell, fmt::Write};

    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::I2c;
    use imu::{AccelGyroSample, MargEstimator, MargSample, SharedI2c};
    use linked_list_allocator::LockedHeap;
    use lis3mdl::{Address as Lis3mdlAddress, Config as Lis3mdlConfig, Lis3mdl};
    use lsm6ds3tr::{
        AccelSampleRate, AccelScale, AccelSettings, GyroSettings, LSM6DS3TR, LsmSettings,
        interface::Interface,
    };
    use math::{Vec3, deg_to_rad};
    use panic_halt as _;
    use rp235x_hal as hal;

    use hal::clocks::Clock;
    use hal::fugit::RateExtU32;
    use hal::uart::{DataBits, StopBits, UartConfig};

    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    #[global_allocator]
    static HEAP: LockedHeap = LockedHeap::empty();

    const XTAL_FREQ_HZ: u32 = 12_000_000;
    const SAMPLE_PERIOD_MS: u32 = 10;
    const LSM6DS3TR_ADDR: u8 = 0x6A;
    const LIS3MDL_ADDR: Lis3mdlAddress = Lis3mdlAddress::Addr1c;
    const GRAVITY_M_S2: f32 = 9.80665;
    const HEAP_SIZE: usize = 4096;

    #[unsafe(link_section = ".uninit")]
    static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    struct Lsm6ds3trI2c<BUS> {
        bus: BUS,
        address: u8,
    }

    impl<BUS> Lsm6ds3trI2c<BUS> {
        const fn new(bus: BUS, address: u8) -> Self {
            Self { bus, address }
        }
    }

    impl<BUS> Interface for Lsm6ds3trI2c<BUS>
    where
        BUS: I2c,
    {
        type Error = BUS::Error;

        fn write(&mut self, addr: u8, value: u8) -> Result<(), Self::Error> {
            self.bus.write(self.address, &[addr, value])
        }

        fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            self.bus.write_read(self.address, &[addr], buffer)
        }
    }

    #[hal::entry]
    fn main() -> ! {
        unsafe {
            HEAP.lock()
                .init(core::ptr::addr_of_mut!(HEAP_MEM) as *mut u8, HEAP_SIZE);
        }

        let mut pac = hal::pac::Peripherals::take().unwrap();
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .unwrap();

        let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let uart_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());
        let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(115200u32.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        // I2C1 on GPIO18/19 is only one common choice. If your board routes
        // STEMMA QT to a different I2C peripheral or pin pair, change it here.
        let i2c = hal::i2c::I2C::i2c1(
            pac.I2C1,
            pins.gpio18.reconfigure(),
            pins.gpio19.reconfigure(),
            400u32.kHz(),
            &mut pac.RESETS,
            clocks.system_clock.freq(),
        );

        let shared_bus = RefCell::new(i2c);

        let lsm_settings = LsmSettings::basic()
            .with_accel(
                AccelSettings::new()
                    .with_sample_rate(AccelSampleRate::_104Hz)
                    .with_scale(AccelScale::_4G),
            )
            .with_gyro(GyroSettings::new());

        let mut accel_gyro = LSM6DS3TR::new(Lsm6ds3trI2c::new(
            SharedI2c::new(&shared_bus),
            LSM6DS3TR_ADDR,
        ))
        .with_settings(lsm_settings);

        let mut magnetometer = Lis3mdl::new(SharedI2c::new(&shared_bus), LIS3MDL_ADDR);

        accel_gyro.init().unwrap();
        magnetometer.init(Lis3mdlConfig::default()).unwrap();

        writeln!(uart, "\r\n9-DoF estimation example\r").ok();
        writeln!(
            uart,
            "LSM6DS3TR reachable: {}\r",
            accel_gyro.is_reachable().unwrap()
        )
        .ok();
        writeln!(
            uart,
            "LIS3MDL WHO_AM_I: 0x{:02X}\r",
            magnetometer.who_am_i().unwrap()
        )
        .ok();

        let mut estimator = MargEstimator::new(0.08);
        let dt = fugit::MicrosDurationU32::from_millis(SAMPLE_PERIOD_MS);
        let mut report_divider = 0u8;

        loop {
            let accel_g = accel_gyro.read_accel().unwrap();
            let gyro_dps = accel_gyro.read_gyro().unwrap();
            let mag_mgauss = magnetometer.read_magnetic_mgauss().unwrap();

            let sample = MargSample::new(
                AccelGyroSample::without_temperature(
                    Vec3::new(accel_g.x, accel_g.y, accel_g.z) * GRAVITY_M_S2,
                    Vec3::new(
                        deg_to_rad(gyro_dps.x),
                        deg_to_rad(gyro_dps.y),
                        deg_to_rad(gyro_dps.z),
                    ),
                ),
                Vec3::new(
                    mag_mgauss.x_mgauss,
                    mag_mgauss.y_mgauss,
                    mag_mgauss.z_mgauss,
                ),
            );

            let estimate = estimator.update_marg(sample, dt);
            report_divider = report_divider.wrapping_add(1);

            if report_divider >= 10 {
                report_divider = 0;
                let euler_deg = estimate.euler.to_degrees();
                writeln!(
                    uart,
                    "roll={:>6.2} deg pitch={:>6.2} deg yaw={:>6.2} deg alt={:>7.3} m vz={:>6.3} m/s\r",
                    euler_deg.roll,
                    euler_deg.pitch,
                    euler_deg.yaw,
                    estimate.relative_altitude_m,
                    estimate.vertical_speed_m_s,
                )
                .ok();
            }

            timer.delay_ms(SAMPLE_PERIOD_MS);
        }
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {
    println!(
        "This example is for RP2350 hardware. Build it with \
         `cargo build -p imu --example rp235x_stemma_qt_9dof --target thumbv8m.main-none-eabihf`."
    );
}
