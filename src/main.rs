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

// Re-export from lib module for use in main.rs
use xiao_nrf52840_sword::{
    detect_upward_thrust, NFC_PAIRING_TIMEOUT_SECS, SENSOR_SAMPLING_INTERVAL_MS,
};

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

// Sensor data packet for BLE transmission (12 bytes) - embedded version with defmt
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

fn init_heap() {
    const HEAP_SIZE: usize = 4096;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    unsafe {
        ALLOCATOR
            .lock()
            .init(core::ptr::addr_of_mut!(HEAP) as *mut u8, HEAP_SIZE);
    }
}

static SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorData, 16> = Channel::new();

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

/// Initialize I2C (TWIM0) for IMU communication
fn init_i2c() -> I2cInterface<Twim<nrf52840_hal::pac::TWIM0>> {
    let hal_p = unsafe { Peripherals::steal() };
    let pins = Parts::new(hal_p.P0);

    let sda = pins.p0_26.into_floating_input().degrade();
    let scl = pins.p0_27.into_floating_input().degrade();
    let pins = Pins { sda, scl };

    let i2c = Twim::new(hal_p.TWIM0, pins, Frequency::K100);

    info!("I2C initialized on TWIM0");
    info!("  SDA: P0.26, SCL: P0.27");
    info!("  Frequency: 100 kHz");

    I2cInterface::new(i2c)
}

/// Initialize the LSM6DS3TR IMU sensor
fn init_imu(
    i2c_interface: I2cInterface<Twim<nrf52840_hal::pac::TWIM0>>,
) -> LSM6DS3TR<I2cInterface<Twim<nrf52840_hal::pac::TWIM0>>> {
    let imu = LSM6DS3TR::new(i2c_interface);
    info!("IMU (LSM6DS3TR) initialized at 0x6A");
    info!("");
    imu
}

// ============================================================================
// SENSOR READING FUNCTIONS
// ============================================================================

/// Read sensor data (accel + gyro) from IMU
/// Returns None if either read fails
fn read_sensor_data(
    imu: &mut LSM6DS3TR<I2cInterface<Twim<nrf52840_hal::pac::TWIM0>>>,
) -> Option<SensorData> {
    match imu.read_accel_raw() {
        Ok(accel) => match imu.read_gyro_raw() {
            Ok(gyro) => {
                let data = SensorData {
                    accel_x: accel.x,
                    accel_y: accel.y,
                    accel_z: accel.z,
                    gyro_x: gyro.x,
                    gyro_y: gyro.y,
                    gyro_z: gyro.z,
                };
                Some(data)
            }
            Err(_) => {
                warn!("Failed to read gyroscope");
                None
            }
        },
        Err(_) => {
            warn!("Failed to read accelerometer");
            None
        }
    }
}

// ============================================================================
// LED ANIMATION FUNCTIONS
// ============================================================================

/// Animate LED strip during thrust (placeholder)
/// TODO: Implement WS2812B NeoPixel animation via PWM
fn animate_led_thrust(_duration_ms: u32) {
    info!("💡 LED thrust animation (stub - not yet implemented)");
}

// ============================================================================
// BLUETOOTH FUNCTIONS
// ============================================================================

/// Advertise to bonded device only
/// TODO: Implement BLE radio initialization and advertising
fn ble_advertise_bonded_device(_mac: [u8; 6]) -> Result<(), &'static str> {
    info!("📡 BLE advertising to bonded device (stub - not yet implemented)");
    Ok(())
}

// ============================================================================
// NFC PAIRING MODE
// ============================================================================

#[cfg(feature = "nfc")]
mod nfc_pairing {
    use super::*;

    /// NFC Pairing Mode: wait for NFC field or timeout
    /// Returns true if NFC detected, false if timeout
    async fn nfc_pairing_mode() -> bool {
        info!("🔌 NFC Pairing Mode - Waiting for NFC reader...");
        info!("   Timeout in {} seconds...", NFC_PAIRING_TIMEOUT_SECS);
        info!("");

        let pairing_start = Instant::now();
        let pairing_timeout = embassy_time::Duration::from_secs(NFC_PAIRING_TIMEOUT_SECS);

        loop {
            // TODO: Implement actual NFCT peripheral detection
            // For now, use a stub that returns false
            if false {
                info!("✅ NFC field detected - pairing successful");
                return true;
            }

            if pairing_start.elapsed() > pairing_timeout {
                info!("⏱️  NFC pairing timeout - switching to Bluetooth mode");
                return false;
            }

            Timer::after_millis(100).await;
        }
    }
}

#[cfg(not(feature = "nfc"))]
fn nfc_pairing_mode_stub() -> bool {
    info!("⚠️  NFC pairing disabled - using Bluetooth fallback");
    false
}

// ============================================================================
// MAIN SENSOR LOOP
// ============================================================================

/// Main sensor reading loop
/// Continuously reads IMU data and sends to BLE channel
async fn main_sensor_loop(imu: &mut LSM6DS3TR<I2cInterface<Twim<nrf52840_hal::pac::TWIM0>>>) {
    loop {
        if let Some(data) = read_sensor_data(imu) {
            // Validate sensor data before processing (inline for embedded use)
            let accel_valid = data.accel_x.abs() < 10000
                && data.accel_y.abs() < 10000
                && data.accel_z.abs() < 10000;
            let gyro_valid =
                data.gyro_x.abs() < 20000 && data.gyro_y.abs() < 20000 && data.gyro_z.abs() < 20000;

            if !accel_valid || !gyro_valid {
                warn!("⚠️  Sensor data out of valid range");
                Timer::after_millis(SENSOR_SAMPLING_INTERVAL_MS).await;
                continue;
            }

            // Check for upward thrust and animate LED
            if detect_upward_thrust(data.accel_z) {
                animate_led_thrust(200);
            }

            // Try to send to BLE channel (skip if full)
            let _ = SENSOR_CHANNEL.try_send(data);

            // Log the data
            info!(
                "📊 A:({},{},{}) G:({},{},{})",
                data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z
            );
        }

        Timer::after_millis(SENSOR_SAMPLING_INTERVAL_MS).await;
    }
}

// ============================================================================
// MAIN ENTRY POINT
// ============================================================================

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init_heap();

    info!("═══════════════════════════════════════════");
    info!("XIAO nRF52840 Sense - He-Man Sword Sensor");
    info!("═══════════════════════════════════════════");

    // Initialize I2C (TWIM0) for IMU communication
    let i2c_interface = init_i2c();
    let mut imu = init_imu(i2c_interface);

    // NFC Pairing Mode
    info!("");
    info!("🔌 Starting NFC Pairing Mode...");
    #[cfg(feature = "nfc")]
    {
        let nfc_detected = nfc_pairing_mode().await;
        info!("");

        if nfc_detected {
            info!("✅ NFC pairing successful - bonded device authenticated");
            info!("   Secure BLE connection established");
        } else {
            info!("⚠️  NFC pairing timed out - using Bluetooth fallback");
            info!("   Legacy BLE advertising mode");
        }
    }
    #[cfg(not(feature = "nfc"))]
    {
        let nfc_detected = nfc_pairing_mode_stub();
        info!("");

        if nfc_detected {
            info!("✅ NFC pairing successful - bonded device authenticated");
            info!("   Secure BLE connection established");
        } else {
            info!("⚠️  NFC pairing disabled - using Bluetooth fallback");
            info!("   Legacy BLE advertising mode");
        }
    }

    // Bluetooth Advertising
    info!("📡 Bluetooth Advertising Mode (Legacy)");
    info!("   Device: He-Man Sword Sensor");
    info!("   Transmitting accelerometer & gyroscope data");
    info!(
        "   Sampling rate: 20 Hz ({}ms interval)",
        SENSOR_SAMPLING_INTERVAL_MS
    );
    info!("");

    // Main sensor loop
    main_sensor_loop(&mut imu).await;
}
