#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use linked_list_allocator::LockedHeap;
use lsm6ds3tr::interface::I2cInterface;
use lsm6ds3tr::LSM6DS3TR;
use nrf52840_hal::{
    gpio::p0::Parts,
    pac::Peripherals,
    twim::{Frequency, Pins, Twim},
};
use panic_probe as _;

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 4096;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    unsafe {
        ALLOCATOR
            .lock()
            .init(core::ptr::addr_of_mut!(HEAP) as *mut u8, HEAP_SIZE);
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init_heap();

    let _p = embassy_nrf::init(Default::default());

    info!("XIAO nRF52840 Sense - IMU Sensor Application");

    // Initialize nRF52840 peripherals
    let hal_p = unsafe { Peripherals::steal() };
    let pins = Parts::new(hal_p.P0);

    // Configure I2C (TWIM0) pins: SDA=P0.26, SCL=P0.27
    let sda = pins.p0_26.into_floating_input().degrade();
    let scl = pins.p0_27.into_floating_input().degrade();
    let pins = Pins { sda, scl };

    let i2c = Twim::new(hal_p.TWIM0, pins, Frequency::K100);

    info!("I2C initialized on TWIM0");
    info!("  SDA: P0.26, SCL: P0.27");
    info!("  Frequency: 100 kHz");

    // Wrap I2C for lsm6ds3tr
    let i2c_interface = I2cInterface::new(i2c);
    let mut imu = LSM6DS3TR::new(i2c_interface);

    info!("IMU (LSM6DS3TR) initialized at 0x6A");

    loop {
        // Read raw accelerometer data
        match imu.read_accel_raw() {
            Ok(accel) => {
                info!("Accel: X={}, Y={}, Z={}", accel.x, accel.y, accel.z);
            }
            Err(_e) => {
                info!("Failed to read accelerometer");
            }
        }

        // Read raw gyroscope data
        match imu.read_gyro_raw() {
            Ok(gyro) => {
                info!("Gyro: X={}, Y={}, Z={}", gyro.x, gyro.y, gyro.z);
            }
            Err(_e) => {
                info!("Failed to read gyroscope");
            }
        }

        Timer::after_millis(1000).await;
    }
}
