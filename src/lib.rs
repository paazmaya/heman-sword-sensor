//! Pure logic module for He-Man Sword Sensor
//!
//! This module contains testable functions that don't depend on embedded hardware.
//! All embedded-specific code remains in main.rs.
//!
//! # Feature gates
//! - `embedded`: compiles async embassy functions and defmt formatting
//! - `nfc`: compiles the `nfc` and `pairing` modules on any target (including host tests)

#![cfg_attr(not(test), no_std)]

// Conditional imports for embedded features
#[cfg(feature = "embedded")]
use defmt::{info, warn};

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

pub const THRUST_THRESHOLD: i16 = 50; // Threshold for upward thrust detection (~0.5g)
pub const NFC_PAIRING_TIMEOUT_SECS: u64 = 15;
pub const SENSOR_SAMPLING_INTERVAL_MS: u64 = 50; // 20 Hz

// ============================================================================
// SENSOR DATA STRUCTURE
// ============================================================================

/// Sensor data packet for BLE transmission (12 bytes).
///
/// `repr(C)` ensures a stable 2-byte-aligned layout so that the raw bytes
/// can be sent directly as a BLE characteristic value.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorData {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
}

impl SensorData {
    /// Create a new SensorData instance.
    pub fn new(
        accel_x: i16,
        accel_y: i16,
        accel_z: i16,
        gyro_x: i16,
        gyro_y: i16,
        gyro_z: i16,
    ) -> Self {
        SensorData {
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
        }
    }

    /// Get the byte size of this struct (always 12).
    pub fn size() -> usize {
        core::mem::size_of::<SensorData>()
    }

    /// Serialize to a 12-byte array suitable for a BLE characteristic value.
    pub fn to_bytes(&self) -> [u8; 12] {
        let mut buf = [0u8; 12];
        buf[0..2].copy_from_slice(&self.accel_x.to_le_bytes());
        buf[2..4].copy_from_slice(&self.accel_y.to_le_bytes());
        buf[4..6].copy_from_slice(&self.accel_z.to_le_bytes());
        buf[6..8].copy_from_slice(&self.gyro_x.to_le_bytes());
        buf[8..10].copy_from_slice(&self.gyro_y.to_le_bytes());
        buf[10..12].copy_from_slice(&self.gyro_z.to_le_bytes());
        buf
    }

    /// Deserialize from a 12-byte array.
    pub fn from_bytes(buf: &[u8; 12]) -> Self {
        SensorData {
            accel_x: i16::from_le_bytes([buf[0], buf[1]]),
            accel_y: i16::from_le_bytes([buf[2], buf[3]]),
            accel_z: i16::from_le_bytes([buf[4], buf[5]]),
            gyro_x: i16::from_le_bytes([buf[6], buf[7]]),
            gyro_y: i16::from_le_bytes([buf[8], buf[9]]),
            gyro_z: i16::from_le_bytes([buf[10], buf[11]]),
        }
    }
}

// ============================================================================
// MOTION DETECTION
// ============================================================================

/// Detect upward thrust based on Z-axis acceleration.
/// Returns `true` if acceleration exceeds [`THRUST_THRESHOLD`].
pub fn detect_upward_thrust(accel_z: i16) -> bool {
    accel_z > THRUST_THRESHOLD
}

/// Classify motion based on acceleration magnitude.
pub fn classify_motion(accel_x: i16, accel_y: i16, accel_z: i16) -> MotionType {
    let magnitude_sq = (accel_x as i32).pow(2) + (accel_y as i32).pow(2) + (accel_z as i32).pow(2);

    if detect_upward_thrust(accel_z) {
        MotionType::UpwardThrust
    } else if magnitude_sq >= 10000 {
        MotionType::Intense
    } else if magnitude_sq >= 900 {
        MotionType::Moderate
    } else {
        MotionType::Idle
    }
}

/// Motion classification result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionType {
    Idle,
    Moderate,
    Intense,
    UpwardThrust,
}

// ============================================================================
// VALIDATION FUNCTIONS
// ============================================================================

/// Validate sensor data for reasonable values.
/// Returns `true` if all axes are within expected hardware ranges.
pub fn validate_sensor_data(data: &SensorData) -> bool {
    // Accelerometer: ±16g maximum range; raw value limit used here is ±10000 LSB
    let accel_valid =
        data.accel_x.abs() < 10000 && data.accel_y.abs() < 10000 && data.accel_z.abs() < 10000;

    // Gyroscope: ±2000 dps maximum; raw value limit used here is ±20000 LSB
    let gyro_valid =
        data.gyro_x.abs() < 20000 && data.gyro_y.abs() < 20000 && data.gyro_z.abs() < 20000;

    accel_valid && gyro_valid
}

// ============================================================================
// NFC PAIRING STATUS
// ============================================================================

/// Lifecycle states for the NFC pairing gate.
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NfcPairingStatus {
    /// Pairing gate not yet started.
    Idle,
    /// Scanning for an NFC field.
    Scanning,
    /// Tag detected; verifying device identity.
    Authenticating,
    /// Pairing completed successfully.
    Success,
    /// Device authentication failed.
    Failed,
    /// NFC field not detected before timeout.
    Timeout,
}

// ============================================================================
// BONDED DEVICE STORAGE
// ============================================================================

/// Serialised byte size of one [`BondedDevice`] record: 6 (MAC) + 4 (timestamp) + 1 (flags).
pub const BONDED_DEVICE_STRUCT_SIZE: usize = 11;

/// A device that has been paired over NFC and is authorised to connect via BLE.
///
/// Stored in flash starting at [`pairing::FLASH_BONDED_DEVICE_START`].
/// Flag bits:
/// - bit 0: active (record is in use)
/// - bit 1: paired  (NFC pairing ceremony completed)
/// - bit 2: verified (secondary UID check passed)
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
pub struct BondedDevice {
    /// Bluetooth MAC address of the bonded device (6 bytes, little-endian).
    pub mac: [u8; 6],
    /// Unix timestamp of the last pairing (seconds, u32).
    pub timestamp: u32,
    /// Status flags (bit 0 = active, bit 1 = paired, bit 2 = verified).
    pub flags: u8,
}

impl BondedDevice {
    /// Create a new bonded device record with the active flag set.
    pub fn new(mac: [u8; 6]) -> Self {
        BondedDevice {
            mac,
            timestamp: 0,
            flags: 0b001, // active only
        }
    }

    /// Deserialise from an 11-byte flash record.
    pub fn from_bytes(bytes: &[u8; BONDED_DEVICE_STRUCT_SIZE]) -> Self {
        let mac = [bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5]];
        let timestamp = u32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]);
        let flags = bytes[10];
        BondedDevice {
            mac,
            timestamp,
            flags,
        }
    }

    /// Serialise to an 11-byte flash record.
    pub fn to_bytes(&self) -> [u8; BONDED_DEVICE_STRUCT_SIZE] {
        let mut buf = [0u8; BONDED_DEVICE_STRUCT_SIZE];
        buf[0..6].copy_from_slice(&self.mac);
        buf[6..10].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[10] = self.flags;
        buf
    }

    /// The record is in use (active flag set).
    pub fn is_active(&self) -> bool {
        self.flags & 0b001 != 0
    }

    /// The NFC pairing ceremony has been completed.
    pub fn is_paired(&self) -> bool {
        self.flags & 0b010 != 0
    }

    /// Secondary UID authentication has passed.
    pub fn is_verified(&self) -> bool {
        self.flags & 0b100 != 0
    }

    /// Set or clear the active flag (bit 0).
    pub fn set_active(&mut self, active: bool) {
        if active {
            self.flags |= 0b001;
        } else {
            self.flags &= !0b001;
        }
    }

    /// Set or clear the paired flag (bit 1).
    pub fn set_paired(&mut self, paired: bool) {
        if paired {
            self.flags |= 0b010;
        } else {
            self.flags &= !0b010;
        }
    }

    /// Set or clear the verified flag (bit 2).
    pub fn set_verified(&mut self, verified: bool) {
        if verified {
            self.flags |= 0b100;
        } else {
            self.flags &= !0b100;
        }
    }

    /// Return the stored timestamp.
    pub fn timestamp(&self) -> u32 {
        self.timestamp
    }

    /// Update the stored timestamp.
    pub fn set_timestamp(&mut self, timestamp: u32) {
        self.timestamp = timestamp;
    }
}

// ============================================================================
// NFC MODULE
// ============================================================================

/// NFC field detection and tag I/O.
///
/// Synchronous helpers are available under both the `nfc` and `embedded`
/// features so they can be tested on the host.  Async functions that depend
/// on embassy are gated to `embedded` only.
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub mod nfc {
    /// NFC Field Detection State machine.
    #[cfg_attr(feature = "embedded", derive(defmt::Format))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum NfcFieldState {
        /// No NFC field detected.
        Idle,
        /// Actively scanning for a field.
        Scanning,
        /// NFC field (and tag) detected.
        Detected,
        /// Hardware or protocol error.
        Error,
    }

    /// Return the current NFC field state.
    ///
    /// On the embedded target this is driven by the NFCT ISR; the
    /// synchronous stub always returns `Idle` so it is safe to call
    /// from host tests.
    pub fn get_field_state() -> NfcFieldState {
        NfcFieldState::Idle
    }

    // ------------------------------------------------------------------
    // UID validation helpers (no hardware dependency)
    // ------------------------------------------------------------------

    /// Validate a 10-byte NFC UID.
    ///
    /// Nordic's NFCT peripheral initialises the first byte to `0x04`
    /// (ISO/IEC 14443 cascade tag) for 10-byte UIDs.  A zero UID
    /// (all bytes `0x00`) is treated as invalid.
    pub fn is_valid_uid(uid: &[u8; 10]) -> bool {
        // All-zeros is a placeholder / uninitialized value
        if uid.iter().all(|&b| b == 0) {
            return false;
        }
        // First byte of a real 10-byte cascade UID is always 0x04
        uid[0] == 0x04
    }

    /// Validate a 7-byte NFC UID (standard short-form tag).
    pub fn is_valid_uid_7(uid: &[u8; 7]) -> bool {
        if uid.iter().all(|&b| b == 0) {
            return false;
        }
        uid[0] == 0x04
    }

    // ------------------------------------------------------------------
    // Async helpers — only compiled when the embassy runtime is present
    // ------------------------------------------------------------------

    /// Detect NFC field presence with a custom timeout.
    ///
    /// The actual NFCT hardware activation is handled in `main.rs` via
    /// `NfcT::activate()`.  This library-level function exists so the
    /// higher-level pairing logic can call it without importing
    /// `embassy-nrf` directly.
    #[cfg(feature = "embedded")]
    pub async fn detect_field_with_timeout(timeout_ms: u64) -> bool {
        use embassy_time::Timer;

        #[cfg(feature = "embedded")]
        defmt::info!("📡 NFC field detection – timeout {} ms", timeout_ms);

        // Hardware activation is performed via `NfcT::activate()` in
        // main.rs.  This stub parks for one poll cycle so the pairing
        // loop can keep iterating with its own deadline.
        Timer::after_millis(100).await;
        false
    }

    /// Detect NFC field with the default 500 ms poll timeout.
    #[cfg(feature = "embedded")]
    pub async fn detect_field() -> bool {
        detect_field_with_timeout(500).await
    }

    /// Read the 10-byte UID from the NFC tag that was activated by
    /// `NfcT::activate()` in `main.rs`.
    ///
    /// `raw_uid` is the NfcId value passed through from the NFCT
    /// peripheral after activation.  For `NfcId::SingleSize` the
    /// hardware provides 4 bytes; we extend to 10 bytes for the return
    /// type.  For a real tag the caller should supply the full UID
    /// returned by the reader.
    ///
    /// Returns `None` when the UID is all-zero (placeholder / error).
    #[cfg(feature = "embedded")]
    pub fn extract_uid_from_nfct(raw: &[u8]) -> Option<[u8; 10]> {
        let mut uid = [0u8; 10];
        let copy_len = raw.len().min(10);
        uid[..copy_len].copy_from_slice(&raw[..copy_len]);
        if uid.iter().all(|&b| b == 0) {
            None
        } else {
            Some(uid)
        }
    }

    /// Write 4 bytes to a Type 2 tag page (pages 4–19).
    /// Stub — actual writes happen through the NFCT peripheral in main.rs.
    #[cfg(feature = "embedded")]
    pub async fn write_nfc_page(page: u8, data: &[u8; 4]) -> bool {
        #[cfg(feature = "embedded")]
        {
            defmt::info!("📡 Writing NFC page {}", page);
            defmt::info!("   Data: {:?}", data);
        }
        #[cfg(not(feature = "embedded"))]
        let _ = (page, data);
        true
    }
}

// ============================================================================
// PAIRING MODULE
// ============================================================================

/// NFC pairing and bonded-device management.
///
/// Synchronous helpers are always compiled under `nfc` or `embedded`; they
/// use direct NVMC memory reads when running on the nRF52840 target.
/// Async pairing orchestration requires the `embedded` feature.
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub mod pairing {
    use super::*;

    // ------------------------------------------------------------------
    // Flash storage layout
    // ------------------------------------------------------------------

    /// Start address of the bonded-device storage region in flash.
    /// Must be 4 KB-page-aligned (0x2000 = 8192 = 2 × 4096).
    pub const FLASH_BONDED_DEVICE_START: u32 = 0x0002_0000;
    /// Size of the bonded-device storage region (4 KB = one flash page).
    pub const FLASH_BONDED_DEVICE_SIZE: usize = 4096;
    /// Maximum number of bonded devices that fit in one flash page.
    pub const MAX_BONDED_DEVICES: usize = FLASH_BONDED_DEVICE_SIZE / FLASH_RECORD_SIZE;
    /// Magic marker written at the start of a valid flash record.
    pub const FLASH_RECORD_MAGIC: u8 = 0xAB;

    // ------------------------------------------------------------------
    // Flash record layout (12 bytes per record = magic + 11-byte device)
    // ------------------------------------------------------------------

    /// Total byte size of one flash record (magic byte + BondedDevice).
    pub const FLASH_RECORD_SIZE: usize = 1 + BONDED_DEVICE_STRUCT_SIZE;

    // ------------------------------------------------------------------
    // Pure helpers (no hardware, testable on host)
    // ------------------------------------------------------------------

    /// Parse a bonded device from a 12-byte flash record.
    ///
    /// Returns `None` when the magic byte is missing or the active flag
    /// is not set (record slot is empty / erased).
    pub fn parse_flash_record(record: &[u8; FLASH_RECORD_SIZE]) -> Option<BondedDevice> {
        if record[0] != FLASH_RECORD_MAGIC {
            return None;
        }
        let mut buf = [0u8; BONDED_DEVICE_STRUCT_SIZE];
        buf.copy_from_slice(&record[1..]);
        let dev = BondedDevice::from_bytes(&buf);
        if dev.is_active() {
            Some(dev)
        } else {
            None
        }
    }

    /// Serialise a bonded device into a 12-byte flash record.
    pub fn make_flash_record(device: &BondedDevice) -> [u8; FLASH_RECORD_SIZE] {
        let mut record = [0u8; FLASH_RECORD_SIZE];
        record[0] = FLASH_RECORD_MAGIC;
        record[1..].copy_from_slice(&device.to_bytes());
        record
    }

    /// Authenticate a device by comparing its MAC against the stored record.
    ///
    /// The device passes when:
    /// - Its MAC matches the stored MAC exactly.
    /// - The stored record is active (bit 0 set).
    /// - The stored record is marked as paired (bit 1 set).
    pub fn authenticate_bonded_device(mac: &[u8; 6]) -> bool {
        #[cfg(feature = "embedded")]
        defmt::info!("🔐 Authenticating device MAC: {:?}", mac);

        match read_bonded_device() {
            Some(device) if device.mac == *mac && device.is_active() && device.is_paired() => {
                #[cfg(feature = "embedded")]
                defmt::info!("✅ Authentication successful");
                true
            }
            _ => {
                #[cfg(feature = "embedded")]
                defmt::warn!(
                    "❌ Authentication failed – MAC not found or device not active/paired"
                );
                false
            }
        }
    }

    /// Authenticate using both the MAC address and the 10-byte NFC UID.
    ///
    /// The device passes only when both the MAC and the UID match the
    /// stored record and all required flags are set.
    pub fn authenticate_with_uid(mac: &[u8; 6], uid: &[u8; 10]) -> bool {
        #[cfg(feature = "embedded")]
        {
            defmt::info!("🔐 Authenticating with MAC + NFC UID");
            defmt::info!("   MAC: {:?}", mac);
            defmt::info!("   UID: {:?}", uid);
        }

        match read_bonded_device_full() {
            Some(device)
                if device.dev.mac == *mac
                    && device.uid == *uid
                    && device.dev.is_active()
                    && device.dev.is_paired() =>
            {
                #[cfg(feature = "embedded")]
                defmt::info!("✅ MAC + UID authentication successful");
                true
            }
            _ => {
                #[cfg(feature = "embedded")]
                defmt::warn!("❌ MAC + UID authentication failed");
                false
            }
        }
    }

    /// Return the current pairing status.
    ///
    /// Stateless stub; a production implementation would track state
    /// in a shared atomic or mutex-protected variable.
    pub fn get_nfc_pairing_status() -> NfcPairingStatus {
        NfcPairingStatus::Idle
    }

    // ------------------------------------------------------------------
    // Extended bonded device record (MAC + UID)
    // ------------------------------------------------------------------

    /// A bonded device record that also stores the 10-byte NFC UID for
    /// secondary authentication.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BondedDeviceFull {
        /// Core device info (MAC, timestamp, flags).
        pub dev: BondedDevice,
        /// 10-byte NFC UID read during the pairing ceremony.
        pub uid: [u8; 10],
    }

    impl BondedDeviceFull {
        /// Total serialised size: BONDED_DEVICE_STRUCT_SIZE + 10
        pub const SERIALISED_SIZE: usize = BONDED_DEVICE_STRUCT_SIZE + 10;

        /// Serialise to bytes for flash storage.
        pub fn to_bytes(&self) -> [u8; Self::SERIALISED_SIZE] {
            let mut buf = [0u8; Self::SERIALISED_SIZE];
            buf[..BONDED_DEVICE_STRUCT_SIZE].copy_from_slice(&self.dev.to_bytes());
            buf[BONDED_DEVICE_STRUCT_SIZE..].copy_from_slice(&self.uid);
            buf
        }

        /// Deserialise from bytes.
        pub fn from_bytes(buf: &[u8; Self::SERIALISED_SIZE]) -> Self {
            let dev_bytes: [u8; BONDED_DEVICE_STRUCT_SIZE] =
                buf[..BONDED_DEVICE_STRUCT_SIZE].try_into().unwrap();
            let mut uid = [0u8; 10];
            uid.copy_from_slice(&buf[BONDED_DEVICE_STRUCT_SIZE..]);
            BondedDeviceFull {
                dev: BondedDevice::from_bytes(&dev_bytes),
                uid,
            }
        }
    }

    // ------------------------------------------------------------------
    // NVMC-backed flash operations
    //
    // On the embedded target these functions use direct memory reads
    // (the nRF52840 maps flash at address 0, so reads are plain pointer
    // dereferences) and delegate writes to main.rs via the Nvmc driver.
    //
    // On the host target (desktop tests) they fall back to in-memory stubs.
    // ------------------------------------------------------------------

    /// Read the primary bonded device MAC from flash.
    ///
    /// On the nRF52840 target, flash is memory-mapped starting at 0x00000000.
    /// We read [`BONDED_DEVICE_STRUCT_SIZE`] bytes at [`FLASH_BONDED_DEVICE_START`]
    /// plus one magic byte offset.
    pub fn read_bonded_device_mac() -> Option<[u8; 6]> {
        read_bonded_device().map(|d| d.mac)
    }

    /// Read the primary [`BondedDevice`] record from flash.
    pub fn read_bonded_device() -> Option<BondedDevice> {
        #[cfg(feature = "embedded")]
        {
            // On the nRF52840, flash is directly readable as memory.
            // Read FLASH_RECORD_SIZE bytes at FLASH_BONDED_DEVICE_START.
            let addr = FLASH_BONDED_DEVICE_START as *const u8;
            let mut record = [0u8; FLASH_RECORD_SIZE];
            unsafe {
                for (i, byte) in record.iter_mut().enumerate() {
                    *byte = addr.add(i).read_volatile();
                }
            }
            parse_flash_record(&record)
        }

        #[cfg(not(feature = "embedded"))]
        {
            // Host stub – returns a simulated bonded device for testing.
            Some(BondedDevice {
                mac: [0x00, 11, 22, 33, 44, 55],
                timestamp: 1_234_567_890,
                flags: 0b111,
            })
        }
    }

    /// Read the extended bonded device record (MAC + NFC UID) from flash.
    pub fn read_bonded_device_full() -> Option<BondedDeviceFull> {
        #[cfg(feature = "embedded")]
        {
            let addr = (FLASH_BONDED_DEVICE_START + FLASH_RECORD_SIZE as u32) as *const u8;
            let mut buf = [0u8; BondedDeviceFull::SERIALISED_SIZE];
            unsafe {
                for (i, byte) in buf.iter_mut().enumerate() {
                    *byte = addr.add(i).read_volatile();
                }
            }
            let full = BondedDeviceFull::from_bytes(&buf);
            if full.dev.is_active() && full.dev.is_paired() {
                Some(full)
            } else {
                None
            }
        }

        #[cfg(not(feature = "embedded"))]
        {
            Some(BondedDeviceFull {
                dev: BondedDevice {
                    mac: [0x00, 11, 22, 33, 44, 55],
                    timestamp: 1_234_567_890,
                    flags: 0b111,
                },
                uid: [0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09],
            })
        }
    }

    /// Write the primary bonded device MAC to flash.
    ///
    /// On the embedded target, the actual NVMC erase+write is performed
    /// in `main.rs` via `nvmc_write_bonded_device`.  This function
    /// returns `true` unconditionally here (the result reflects the
    /// caller's intent; errors are handled in main.rs).
    pub fn write_bonded_device_to_flash(mac: &[u8; 6]) -> bool {
        let dev = BondedDevice {
            mac: *mac,
            timestamp: 0,
            flags: 0b011, // active + paired
        };
        write_bonded_device_to_flash_full(&dev)
    }

    /// Write the full [`BondedDevice`] structure to flash (stub).
    ///
    /// The actual NVMC write happens in `main.rs`; this function is a
    /// pure-logic entry point that can be tested on the host.
    pub fn write_bonded_device_to_flash_full(_device: &BondedDevice) -> bool {
        #[cfg(feature = "embedded")]
        defmt::info!(
            "💾 Flash write requested for MAC {:?} (delegated to NVMC in main.rs)",
            _device.mac
        );
        true
    }

    /// Register a new bonded device (first-time pairing).
    pub fn register_bonded_device(mac: &[u8; 6]) -> bool {
        #[cfg(feature = "embedded")]
        defmt::info!("📝 Registering bonded device MAC: {:?}", mac);
        write_bonded_device_to_flash(mac)
    }

    /// Unregister a bonded device (clear its flash record).
    pub fn unregister_bonded_device(mac: &[u8; 6]) -> bool {
        #[cfg(feature = "embedded")]
        defmt::info!("🗑️  Unregistering bonded device MAC: {:?}", mac);
        #[cfg(not(feature = "embedded"))]
        let _ = mac;
        true
    }

    /// Return all registered bonded devices from flash.
    ///
    /// Reads up to [`MAX_BONDED_DEVICES`] records starting at
    /// [`FLASH_BONDED_DEVICE_START`].  Invalid records (bad magic,
    /// inactive flag) are skipped.
    ///
    /// Returns a fixed-size array; entries with `flags == 0` are empty slots.
    pub fn get_bonded_devices() -> [BondedDevice; 2] {
        #[cfg(feature = "embedded")]
        {
            let base = FLASH_BONDED_DEVICE_START as *const u8;
            let mut devices = [BondedDevice {
                mac: [0u8; 6],
                timestamp: 0,
                flags: 0,
            }; 2];
            let mut found = 0usize;

            for slot in 0..MAX_BONDED_DEVICES {
                if found >= 2 {
                    break;
                }
                let offset = slot * FLASH_RECORD_SIZE;
                let mut record = [0u8; FLASH_RECORD_SIZE];
                unsafe {
                    for (i, byte) in record.iter_mut().enumerate() {
                        *byte = base.add(offset + i).read_volatile();
                    }
                }
                if let Some(dev) = parse_flash_record(&record) {
                    devices[found] = dev;
                    found += 1;
                }
            }
            devices
        }

        #[cfg(not(feature = "embedded"))]
        {
            [
                BondedDevice {
                    mac: [0x00, 11, 22, 33, 44, 55],
                    timestamp: 1_234_567_890,
                    flags: 0b111,
                },
                BondedDevice {
                    mac: [0x00, 12, 23, 34, 45, 56],
                    timestamp: 1_234_567_891,
                    flags: 0b111,
                },
            ]
        }
    }

    /// Check whether `mac` matches any registered bonded device.
    pub fn is_bonded_device(mac: &[u8; 6]) -> bool {
        get_bonded_devices()
            .iter()
            .any(|d| d.is_active() && d.mac == *mac)
    }

    // ------------------------------------------------------------------
    // BLE configuration
    // ------------------------------------------------------------------

    /// BLE advertising configuration.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct BleConfig {
        /// Advertising interval in units of 0.625 ms (e.g. 160 = 100 ms).
        pub adv_interval_units: u16,
        /// Bonded device MAC address for directed advertising / whitelist.
        pub bonded_mac: Option<[u8; 6]>,
        /// Connection timeout in seconds.
        pub connection_timeout_secs: u32,
        /// Use whitelist (advertise only to bonded device).
        pub use_whitelist: bool,
    }

    impl Default for BleConfig {
        fn default() -> Self {
            BleConfig {
                adv_interval_units: 160, // 160 × 0.625 ms = 100 ms
                bonded_mac: None,
                connection_timeout_secs: 30,
                use_whitelist: false,
            }
        }
    }

    impl BleConfig {
        /// Create a whitelist-only config targeting the given bonded MAC.
        pub fn bonded_only(mac: [u8; 6]) -> Self {
            BleConfig {
                bonded_mac: Some(mac),
                use_whitelist: true,
                ..Default::default()
            }
        }

        /// Advertising interval in milliseconds (approximate).
        pub fn adv_interval_ms(&self) -> u32 {
            self.adv_interval_units as u32 * 625 / 1000
        }
    }

    // ------------------------------------------------------------------
    // Async pairing orchestration — requires embassy runtime
    // ------------------------------------------------------------------

    /// Run the full NFC pairing gate.
    ///
    /// 1. Polls for an NFC field every 100 ms for up to
    ///    [`NFC_PAIRING_TIMEOUT_SECS`] seconds.
    /// 2. When a field is detected, reads the bonded device MAC from flash.
    /// 3. Authenticates the MAC against the stored record.
    /// 4. Returns `true` on success, `false` on timeout or auth failure.
    ///
    /// The actual NFCT hardware activation (`NfcT::activate()`) is
    /// performed in `main.rs`; this function drives the higher-level
    /// pairing state machine.
    #[cfg(feature = "embedded")]
    pub async fn pairing_mode() -> bool {
        use embassy_time::{Duration, Instant, Timer};

        defmt::info!("═══════════════════════════════════════════");
        defmt::info!("🔌 NFC Pairing Mode – Primary Authentication Gate");
        defmt::info!("   Timeout: {} s", NFC_PAIRING_TIMEOUT_SECS);
        defmt::info!("═══════════════════════════════════════════");

        let start = Instant::now();
        let timeout = Duration::from_secs(NFC_PAIRING_TIMEOUT_SECS);

        loop {
            if nfc::detect_field().await {
                defmt::info!("✅ NFC field detected – initiating pairing…");

                match read_bonded_device_mac() {
                    Some(mac) => {
                        if authenticate_bonded_device(&mac) {
                            return true;
                        } else {
                            defmt::warn!("⚠️  Bonded device authentication failed");
                            return false;
                        }
                    }
                    None => {
                        defmt::warn!("⚠️  No bonded device in flash – open pairing mode");
                        // First-time pairing: accept any NFC tap and register
                        return false;
                    }
                }
            }

            if start.elapsed() > timeout {
                defmt::info!("⏱️  NFC pairing timeout – falling back to BLE advertising");
                return false;
            }

            Timer::after_millis(100).await;
        }
    }
}

// ============================================================================
// TOP-LEVEL API RE-EXPORTS
// ============================================================================

/// Verify a device MAC against the stored bonded device record.
///
/// Returns `true` when the MAC matches the stored record and the record
/// is both active and paired.
///
/// See [`pairing::authenticate_bonded_device`].
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub fn authenticate_bonded_device(mac: &[u8; 6]) -> bool {
    pairing::authenticate_bonded_device(mac)
}

/// Return the current NFC pairing status.
///
/// See [`pairing::get_nfc_pairing_status`].
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub fn get_nfc_pairing_status() -> NfcPairingStatus {
    pairing::get_nfc_pairing_status()
}
