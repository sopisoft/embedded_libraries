use airframe::{AttitudeHoldLimits, FixedWingController, ServoMap};
use embedded_hal::i2c::I2c;
use imu::find_i2c_address_by_id;
use linked_list_allocator::LockedHeap;
use lis3mdl::Address as Lis3mdlAddress;
use lsm6ds3tr::{
    AccelSampleRate, AccelScale, AccelSettings, GyroSettings, LsmSettings, interface::Interface,
};
use pwm::{ServoRange, ServoSet};
use stabilization::{AxisErrorMode, CascadeAttitudeController, CascadeAxis};

pub const XTAL_FREQ_HZ: u32 = 12_000_000;
pub const SAMPLE_PERIOD_MS: u32 = 10;
pub const GRAVITY_M_S2: f32 = 9.80665;

const HEAP_SIZE: usize = 4096;
const WHO_AM_I_REGISTER: u8 = 0x0F;
const LSM6DS3TR_DEVICE_ID: u8 = 0x6A;
const LSM6DS3TR_ADDR_CANDIDATES: [u8; 2] = [0x6A, 0x6B];
const LIS3MDL_ADDR_CANDIDATES: [u8; 2] = [0x1C, 0x1E];

#[global_allocator]
static HEAP: LockedHeap = LockedHeap::empty();

#[unsafe(link_section = ".uninit")]
static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

pub fn init_heap() {
    unsafe {
        HEAP.lock()
            .init(core::ptr::addr_of_mut!(HEAP_MEM) as *mut u8, HEAP_SIZE);
    }
}

pub struct Lsm6ds3trI2c<BUS> {
    bus: BUS,
    address: u8,
}

impl<BUS> Lsm6ds3trI2c<BUS> {
    pub const fn new(bus: BUS, address: u8) -> Self {
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

pub fn lsm_settings() -> LsmSettings {
    LsmSettings::basic()
        .with_accel(
            AccelSettings::new()
                .with_sample_rate(AccelSampleRate::_104Hz)
                .with_scale(AccelScale::_4G),
        )
        .with_gyro(GyroSettings::new())
}

pub fn detect_lsm6ds3tr_address<BUS>(shared_bus: &core::cell::RefCell<BUS>) -> u8
where
    BUS: I2c,
{
    find_i2c_address_by_id(
        shared_bus,
        &LSM6DS3TR_ADDR_CANDIDATES,
        WHO_AM_I_REGISTER,
        LSM6DS3TR_DEVICE_ID,
    )
    .unwrap_or_else(|| panic!("LSM6DS3TR-C not found at 0x6A or 0x6B"))
}

pub fn detect_lis3mdl_address<BUS>(shared_bus: &core::cell::RefCell<BUS>) -> Lis3mdlAddress
where
    BUS: I2c,
{
    let address = find_i2c_address_by_id(
        shared_bus,
        &LIS3MDL_ADDR_CANDIDATES,
        WHO_AM_I_REGISTER,
        lis3mdl::DEVICE_ID,
    )
    .unwrap_or_else(|| panic!("LIS3MDL not found at 0x1C or 0x1E"));

    Lis3mdlAddress::from_u8(address).unwrap()
}

pub fn servo_ranges() -> ServoSet<5> {
    let base = ServoRange::new(
        fugit::MicrosDurationU32::from_micros(20_000),
        fugit::MicrosDurationU32::from_micros(1_000),
        fugit::MicrosDurationU32::from_micros(2_000),
        -60.0,
        60.0,
    );
    ServoSet::new([
        base,
        base,
        base,
        base,
        ServoRange::new(
            fugit::MicrosDurationU32::from_micros(20_000),
            fugit::MicrosDurationU32::from_micros(1_000),
            fugit::MicrosDurationU32::from_micros(2_000),
            0.0,
            90.0,
        ),
    ])
}

pub fn build_controller(servos: ServoSet<5>) -> FixedWingController<5> {
    let mut roll = CascadeAxis::new(
        control::PidController::new(5.0, 0.2, 0.0),
        control::PidController::new(0.8, 0.05, 0.01),
        2.5,
    );
    roll.attitude_pid.set_output_limits(-2.5, 2.5);
    roll.rate_pid.set_output_limits(-1.0, 1.0);

    let mut pitch = CascadeAxis::new(
        control::PidController::new(6.0, 0.2, 0.0),
        control::PidController::new(0.9, 0.08, 0.02),
        2.0,
    );
    pitch.attitude_pid.set_output_limits(-2.0, 2.0);
    pitch.rate_pid.set_output_limits(-1.0, 1.0);

    let mut yaw = CascadeAxis::new(
        control::PidController::new(3.0, 0.0, 0.0),
        control::PidController::new(0.4, 0.02, 0.0),
        1.5,
    )
    .with_error_mode(AxisErrorMode::WrappedAngle);
    yaw.rate_pid.set_output_limits(-1.0, 1.0);

    FixedWingController::new(
        CascadeAttitudeController::new(roll, pitch, yaw),
        control::ConventionalTailMixer::new(),
        servos,
        ServoMap::conventional_5ch(),
        AttitudeHoldLimits::default(),
    )
}
