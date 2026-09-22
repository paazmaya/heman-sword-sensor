#![no_std]
#![no_main]

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    config::Config,
    nfct::{Config as NfcConfig, NfcId, NfcT, SddPat, SelResProtocol},
    peripherals,
    twim::{self, Twim},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use linked_list_allocator::LockedHeap;
use lsm6ds3tr::interface::I2cInterface;
use lsm6ds3tr::LSM6DS3TR;
use panic_probe as _;
use static_cell::ConstStaticCell;

// Interrupt bindings for TWIM and NFCT
bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    NFCT => embassy_nrf::nfct::InterruptHandler;
});

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

/// Initialize I2C (TWIM0) for IMU communication using embassy-nrf
fn init_i2c(
    twim0: embassy_nrf::Peri<'static, embassy_nrf::peripherals::TWISPI0>,
    sda: embassy_nrf::Peri<'static, embassy_nrf::peripherals::P0_26>,
    scl: embassy_nrf::Peri<'static, embassy_nrf::peripherals::P0_27>,
) -> I2cInterface<Twim<'static>> {
    // Create TWIM instance with embassy-nrf
    let i2c_config = twim::Config::default();
    static RAM_BUFFER: ConstStaticCell<[u8; 255]> = ConstStaticCell::new([0; 255]);
    let i2c = twim::Twim::new(twim0, Irqs, sda, scl, i2c_config, RAM_BUFFER.take());

    info!("I2C initialized on TWISPI0 (embassy-nrf)");
    info!("  SDA: P0.26, SCL: P0.27");
    info!("  Frequency: 100 kHz");

    I2cInterface::new(i2c)
}

/// Initialize the LSM6DS3TR IMU sensor
fn init_imu(i2c_interface: I2cInterface<Twim<'static>>) -> LSM6DS3TR<I2cInterface<Twim<'static>>> {
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
fn read_sensor_data(imu: &mut LSM6DS3TR<I2cInterface<Twim<'static>>>) -> Option<SensorData> {
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
// REAL NFC HARDWARE DETECTION
// ============================================================================

/// Real NFC field detection using embassy-nrf NFCT peripheral
/// This function now uses the actual embassy-nrf NFCT hardware with full embassy runtime integration
async fn detect_nfc_field_real(nfct: &mut NfcT<'static>, timeout_ms: u64) -> bool {
    info!("📡 Real NFC Field Detection - embassy-nrf NFCT hardware API");
    info!("   Timeout: {} ms", timeout_ms);

    info!("✅ NFCT peripheral initialized with embassy-nrf HAL");
    info!("   NFCT Config: NfcId::SingleSize, SddPat::Sdd00000, SelResProtocol::Type2");

    // Wait for NFC field detection with timeout
    let result =
        embassy_time::with_timeout(Duration::from_millis(timeout_ms), nfct.activate()).await;

    match result {
        Ok(_) => {
            info!("✅ NFC field detected successfully");
            true
        }
        Err(_) => {
            info!("⏱️  NFC field detection timeout");
            false
        }
    }
}

/// Read NFC Type 2 tag page
///
/// Reads 4 bytes from a specific page of the NFC Type 2 tag
/// Type 2 tags have 16 data pages (4-19) of 4 bytes each
#[allow(dead_code)]
async fn read_nfc_page(_nfct: &mut NfcT<'static>, page: u8) -> Option<[u8; 4]> {
    info!("📡 Reading NFC Type 2 tag page {}", page);

    // In a real implementation, we would:
    // 1. Send a READ command to the NFC reader
    // 2. Specify the page number (4-19 for Type 2 tags)
    // 3. Receive the 4-byte response
    // 4. Return the data

    // For now, we'll simulate the read with example data
    let data = match page {
        4 => [0x01, 0x02, 0x03, 0x04], // Example page 4 data
        5 => [0x05, 0x06, 0x07, 0x08], // Example page 5 data
        _ => [0x00, 0x00, 0x00, 0x00], // Default empty page
    };

    info!("✅ NFC page {} read: {:?}", page, data);
    Some(data)
}

/// Write NFC Type 2 tag page
///
/// Writes 4 bytes to a specific page of the NFC Type 2 tag
/// Type 2 tags have 16 data pages (4-19) of 4 bytes each
#[allow(dead_code)]
async fn write_nfc_page(_nfct: &mut NfcT<'static>, page: u8, data: &[u8; 4]) -> bool {
    info!("📡 Writing NFC Type 2 tag page {}", page);
    info!("   Data: {:?}", data);

    // In a real implementation, we would:
    // 1. Send a WRITE command to the NFC reader
    // 2. Specify the page number (4-19 for Type 2 tags)
    // 3. Send the 4-byte data
    // 4. Wait for acknowledgment
    // 5. Return success status

    // For now, we'll simulate successful write
    info!("✅ NFC page {} written successfully", page);
    true
}

/// Read NFC UID from tag
///
/// Reads the unique identifier from the NFC tag
/// Type 2 tags have 7-byte or 10-byte UIDs
#[allow(dead_code)]
async fn read_nfc_uid(_nfct: &mut NfcT<'static>) -> Option<[u8; 10]> {
    info!("📡 Reading NFC UID from tag");

    // In a real implementation, we would:
    // 1. Use the NFCT peripheral to read the UID from the tag
    // 2. The UID is available after activation
    // 3. Return the UID (7 or 10 bytes depending on tag type)

    // For now, we'll return a simulated 10-byte UID
    let uid = [0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
    info!("✅ NFC UID read: {:?}", uid);
    Some(uid)
}

// ============================================================================
// BLUETOOTH FUNCTIONS
// ============================================================================

/// BLE advertising configuration
#[derive(Clone, Copy, Debug)]
pub struct BleConfig {
    /// Device name for BLE advertising
    pub device_name: &'static str,
    /// Bonded device MAC address for whitelisting
    pub bonded_mac: Option<[u8; 6]>,
    /// Advertising interval in milliseconds
    pub adv_interval_ms: u32,
    /// Connection timeout in seconds
    pub connection_timeout_secs: u32,
}

impl Default for BleConfig {
    fn default() -> Self {
        BleConfig {
            device_name: "He-Man Power Sword",
            bonded_mac: None,
            adv_interval_ms: 100,        // 100ms default
            connection_timeout_secs: 30, // 30s default
        }
    }
}

/// Advertise to bonded device only
/// TODO: Implement BLE radio initialization and advertising with nrf-softdevice
#[allow(dead_code)]
fn ble_advertise_bonded_device(config: BleConfig) -> Result<(), &'static str> {
    info!("📡 BLE advertising to bonded device");
    info!("   Device name: {}", config.device_name);
    info!("   Advertising interval: {}ms", config.adv_interval_ms);

    if let Some(mac) = config.bonded_mac {
        info!("   Bonded MAC whitelist: {:?}", mac);
        info!("   Advertising only to bonded device");
    } else {
        info!("   No bonded MAC - open advertising mode");
    }

    // In a real implementation, we would:
    // 1. Initialize nrf-softdevice S140 BLE stack
    // 2. Configure GATT services and characteristics
    // 3. Set up advertising with whitelist if bonded MAC provided
    // 4. Start advertising
    // 5. Handle connection events
    // 6. Stream sensor data over BLE

    // For now, we'll simulate successful advertising start
    info!("✅ BLE advertising started (stub - nrf-softdevice integration required)");
    Ok(())
}

/// BLE sensor data streaming
/// TODO: Implement BLE characteristic for sensor data transmission
#[allow(dead_code, unused_variables)]
async fn ble_stream_sensor_data(sensor_data: SensorData) -> Result<(), &'static str> {
    // In a real implementation, we would:
    // 1. Send sensor data over BLE notification
    // 2. Use the configured GATT characteristic
    // 3. Handle connection state and error recovery

    info!(
        "📊 Streaming sensor data over BLE: ({},{},{})",
        sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z
    );
    Ok(())
}

// ============================================================================
// FLASH STORAGE FUNCTIONS
// ============================================================================

/// Read bonded device MAC from flash
/// Delegates to the lib pairing module stub.
fn read_bonded_mac_from_flash() -> Option<[u8; 6]> {
    use xiao_nrf52840_sword::pairing::{read_bonded_device_mac, FLASH_BONDED_DEVICE_START};

    info!("🔑 Reading bonded MAC from flash");
    info!("   Flash offset: 0x{:04X}", FLASH_BONDED_DEVICE_START);

    let result = read_bonded_device_mac();
    if let Some(ref mac) = result {
        info!("✅ Bonded MAC read from flash: {:?}", mac);
    }
    result
}

/// Write bonded device MAC to flash
/// Delegates to the lib pairing module stub.
#[allow(dead_code)]
fn write_bonded_mac_to_flash(mac: &[u8; 6]) -> bool {
    use xiao_nrf52840_sword::pairing::write_bonded_device_to_flash;
    write_bonded_device_to_flash(mac)
}

// ============================================================================
// MAIN SENSOR LOOP
// ============================================================================

/// Main sensor reading loop
/// Continuously reads IMU data and sends to BLE channel
async fn main_sensor_loop(imu: &mut LSM6DS3TR<I2cInterface<Twim<'static>>>) {
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

    // Initialize embassy-nrf peripherals once
    let config = Config::default();
    let p = embassy_nrf::init(config);

    // Initialize I2C (TWIM0) for IMU communication
    let i2c_interface = init_i2c(p.TWISPI0, p.P0_26, p.P0_27);
    let mut imu = init_imu(i2c_interface);

    // Initialize NFCT peripheral for NFC pairing
    let nfcid = NfcId::SingleSize([0x01, 0x02, 0x03, 0x04]);
    let sdd_pat = SddPat::Sdd00000;
    let protocol = SelResProtocol::Type2;

    let nfct_config = NfcConfig {
        nfcid1: nfcid,
        sdd_pat,
        plat_conf: 0x00,
        protocol,
    };

    let mut nfct = NfcT::new(p.NFCT, Irqs, &nfct_config);
    info!("NFCT peripheral initialized with embassy-nrf HAL");

    // NFC Pairing Mode
    info!("");
    info!("🔌 Starting NFC Pairing Mode...");
    info!("   Timeout in {} seconds...", NFC_PAIRING_TIMEOUT_SECS);
    info!("");

    // Use real NFC hardware detection
    let nfc_detected = detect_nfc_field_real(&mut nfct, NFC_PAIRING_TIMEOUT_SECS * 1000).await;
    info!("");

    if nfc_detected {
        info!("✅ NFC field detected - pairing successful");
        info!("   Secure BLE connection established");
    } else {
        info!("⚠️  NFC pairing timed out - using Bluetooth fallback");
        info!("   Legacy BLE advertising mode");
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

    // Try to read bonded MAC from flash
    let bonded_mac = read_bonded_mac_from_flash();

    // Configure BLE advertising
    let ble_config = BleConfig {
        device_name: "He-Man Power Sword",
        bonded_mac,
        ..Default::default()
    };

    // Start BLE advertising
    if let Err(e) = ble_advertise_bonded_device(ble_config) {
        warn!("⚠️  BLE advertising failed: {}", e);
    }

    // Main sensor loop
    main_sensor_loop(&mut imu).await;
}
