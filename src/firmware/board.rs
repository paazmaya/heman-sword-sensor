//! Embedded board adapter. Hardware-independent contracts live in the library.

use crate::Irqs;
use defmt::{info, warn};
use embassy_nrf::{
    peripherals,
    twim::{self, Twim},
};
use lsm6ds3tr::{interface::I2cInterface, LSM6DS3TR};
use static_cell::ConstStaticCell;
use xiao_nrf52840_sword::SensorData;

pub(crate) fn init_i2c(
    twim0: embassy_nrf::Peri<'static, peripherals::TWISPI0>,
    sda: embassy_nrf::Peri<'static, peripherals::P0_07>,
    scl: embassy_nrf::Peri<'static, peripherals::P0_27>,
) -> I2cInterface<Twim<'static>> {
    let i2c_config = twim::Config::default();
    static RAM_BUFFER: ConstStaticCell<[u8; 255]> = ConstStaticCell::new([0; 255]);
    let i2c = twim::Twim::new(twim0, Irqs, sda, scl, i2c_config, RAM_BUFFER.take());
    info!("I2C on TWISPI0 – SDA P0.07, SCL P0.27, 100 kHz");
    I2cInterface::new(i2c)
}

pub(crate) fn init_imu(i2c: I2cInterface<Twim<'static>>) -> LSM6DS3TR<I2cInterface<Twim<'static>>> {
    let mut imu = LSM6DS3TR::new(i2c);
    if !matches!(imu.is_reachable(), Ok(true)) {
        defmt::panic!("LSM6DS3TR WHO_AM_I check failed");
    }
    // Explicit scales match the library's raw-count conversion factors.
    imu.settings.accel = lsm6ds3tr::AccelSettings::new();
    imu.settings.gyro = lsm6ds3tr::GyroSettings::new();
    if imu.init().is_err() {
        defmt::panic!("LSM6DS3TR configuration failed");
    }
    info!("IMU LSM6DS3TR @ 0x6A");
    imu
}

// ============================================================================
// SENSOR READING
// ============================================================================

pub(crate) fn read_sensor_data(
    imu: &mut LSM6DS3TR<I2cInterface<Twim<'static>>>,
) -> Option<SensorData> {
    match imu.read_accel_raw() {
        Ok(accel) => match imu.read_gyro_raw() {
            Ok(gyro) => Some(SensorData {
                accel_x: accel.x,
                accel_y: accel.y,
                accel_z: accel.z,
                gyro_x: gyro.x,
                gyro_y: gyro.y,
                gyro_z: gyro.z,
            }),
            Err(_) => {
                warn!("gyro read failed");
                None
            }
        },
        Err(_) => {
            warn!("accel read failed");
            None
        }
    }
}
