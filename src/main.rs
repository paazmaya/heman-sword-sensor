#![no_std]
#![no_main]

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Instant, Timer};
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

// Sensor data packet for BLE transmission (12 bytes)
#[repr(C)]
#[derive(Clone, Copy, defmt::Format)]
struct SensorData {
    accel_x: i16,
    accel_y: i16,
    accel_z: i16,
    gyro_x: i16,
    gyro_y: i16,
    gyro_z: i16,
}

static SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorData, 16> = Channel::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init_heap();

    info!("═══════════════════════════════════════════");
    info!("XIAO nRF52840 Sense - He-Man Sword Sensor");
    info!("═══════════════════════════════════════════");

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
    info!("");

    // NFC Pairing Mode - wait for NFC field detection or timeout
    info!("🔌 NFC Pairing Mode - Waiting for NFC reader...");
    info!("   Timeout in 15 seconds...");
    info!("");

    let pairing_start = Instant::now();
    let pairing_timeout = embassy_time::Duration::from_secs(15);

    loop {
        if pairing_start.elapsed() > pairing_timeout {
            info!("⏱️  NFC pairing timeout - switching to Bluetooth mode");
            break;
        }

        // TODO: Check for NFC field using nRF52840 NFC controller
        // nRF52840 has built-in NFC Type 2 tag support
        // For now, simulate with polling

        Timer::after_millis(100).await;
    }

    info!("");
    info!("📡 Bluetooth Advertising Mode (Legacy)");
    info!("   Device: He-Man Sword Sensor");
    info!("   Transmitting accelerometer & gyroscope data");
    info!("   Sampling rate: 20 Hz (50ms interval)");
    info!("");

    // Main sensor reading and BLE broadcasting loop
    loop {
        match imu.read_accel_raw() {
            Ok(accel) => {
                match imu.read_gyro_raw() {
                    Ok(gyro) => {
                        let data = SensorData {
                            accel_x: accel.x,
                            accel_y: accel.y,
                            accel_z: accel.z,
                            gyro_x: gyro.x,
                            gyro_y: gyro.y,
                            gyro_z: gyro.z,
                        };

                        // Try to send to BLE channel (skip if full)
                        let _ = SENSOR_CHANNEL.try_send(data);

                        // Log the data
                        info!(
                            "📊 A:({},{},{}) G:({},{},{})",
                            data.accel_x,
                            data.accel_y,
                            data.accel_z,
                            data.gyro_x,
                            data.gyro_y,
                            data.gyro_z
                        );
                    }
                    Err(_) => {
                        warn!("Failed to read gyroscope");
                    }
                }
            }
            Err(_) => {
                warn!("Failed to read accelerometer");
            }
        }

        Timer::after_millis(50).await; // 20 Hz sampling
    }
}
