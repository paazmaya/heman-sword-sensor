//! Pure logic module for He-Man Sword Sensor
//!
//! This module contains testable functions that don't depend on embedded hardware.
//! All embedded-specific code remains in main.rs.

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

/// Sensor data packet for BLE transmission (12 bytes)
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
    /// Create a new SensorData instance
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

    /// Get the size of this struct (should be exactly 12 bytes)
    pub fn size() -> usize {
        core::mem::size_of::<SensorData>()
    }
}

// ============================================================================
// MOTION DETECTION
// ============================================================================

/// Detect upward thrust based on Z-axis acceleration
/// Returns true if acceleration exceeds THRUST_THRESHOLD
pub fn detect_upward_thrust(accel_z: i16) -> bool {
    accel_z > THRUST_THRESHOLD
}

/// Classify motion based on acceleration magnitude
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

/// Validate sensor data for reasonable values
/// Returns true if data is within expected ranges
pub fn validate_sensor_data(data: &SensorData) -> bool {
    // Accelerometer: typically ±16g, ±2g = ±20480 in raw values, so ±5000 is safe
    let accel_valid =
        data.accel_x.abs() < 10000 && data.accel_y.abs() < 10000 && data.accel_z.abs() < 10000;

    // Gyroscope: typically ±2000 dps, raw value limit is ±32767
    let gyro_valid =
        data.gyro_x.abs() < 20000 && data.gyro_y.abs() < 20000 && data.gyro_z.abs() < 20000;

    accel_valid && gyro_valid
}

// ============================================================================
// NFC PAIRING MODULE
// ============================================================================

/// NFC Pairing Status
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NfcPairingStatus {
    /// NFC pairing is idle, waiting for field
    Idle,
    /// NFC field is being scanned
    Scanning,
    /// NFC device is being authenticated
    Authenticating,
    /// NFC pairing completed successfully
    Success,
    /// NFC pairing failed
    Failed,
    /// NFC pairing timed out
    Timeout,
}

/// NFC Bonded Device Storage (6 bytes for MAC address)
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
pub struct BondedDevice {
    /// MAC address of the bonded device (6 bytes)
    pub mac: [u8; 6],
    /// Timestamp of the last pairing (Unix timestamp)
    pub timestamp: u32,
    /// Flags: bit 0 = active, bit 1 = paired, bit 2 = verified
    pub flags: u8,
}

impl BondedDevice {
    /// Create a new bonded device with default values
    pub fn new(mac: [u8; 6]) -> Self {
        BondedDevice {
            mac,
            timestamp: 0,
            flags: 0b001, // active only
        }
    }

    /// Check if the device is active
    pub fn is_active(&self) -> bool {
        self.flags & 0b001 != 0
    }

    /// Check if the device is paired
    pub fn is_paired(&self) -> bool {
        self.flags & 0b010 != 0
    }

    /// Check if the device is verified
    pub fn is_verified(&self) -> bool {
        self.flags & 0b100 != 0
    }

    /// Set the active flag
    pub fn set_active(&mut self, active: bool) {
        self.flags |= active as u8;
    }

    /// Set the paired flag
    pub fn set_paired(&mut self, paired: bool) {
        self.flags |= (paired as u8) << 1;
    }

    /// Set the verified flag
    pub fn set_verified(&mut self, verified: bool) {
        self.flags |= (verified as u8) << 2;
    }

    /// Get the current timestamp
    pub fn timestamp(&self) -> u32 {
        self.timestamp
    }

    /// Set the timestamp
    pub fn set_timestamp(&mut self, timestamp: u32) {
        self.timestamp = timestamp;
    }
}

/// NFC field and tag interaction.
///
/// Data types and synchronous helpers are available under both the `nfc` and
/// `embedded` features so they can be used in desktop tests.  Async functions
/// that depend on `embassy_time` are gated to `embedded` only.
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub mod nfc {
    #[cfg(feature = "embedded")]
    use super::*;

    /// NFC Field Detection State
    #[cfg_attr(feature = "embedded", derive(defmt::Format))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum NfcFieldState {
        /// No NFC field detected
        Idle,
        /// Scanning for NFC field
        Scanning,
        /// NFC field detected
        Detected,
        /// Error occurred
        Error,
    }

    /// Get current NFC field state (always `Idle` until hardware drives it)
    pub fn get_field_state() -> NfcFieldState {
        NfcFieldState::Idle
    }

    // ------------------------------------------------------------------
    // Async helpers — only compiled when the embassy runtime is present
    // ------------------------------------------------------------------

    /// Detect NFC field presence with a custom timeout.
    ///
    /// The actual hardware activation is handled in `main.rs` via
    /// `NfcT::activate()`.  This library-level stub exists so higher-level
    /// pairing logic can call it without importing embassy-nrf directly.
    ///
    /// # Arguments
    /// * `timeout_ms` – Maximum time to wait in milliseconds
    ///
    /// # Returns
    /// `true` if an NFC field was detected within the timeout, `false` otherwise.
    #[cfg(feature = "embedded")]
    pub async fn detect_field_with_timeout(timeout_ms: u64) -> bool {
        use embassy_time::Timer;

        #[cfg(feature = "embedded")]
        info!("📡 NFC Field Detection – timeout {} ms", timeout_ms);

        // Hardware activation lives in main.rs (detect_nfc_field_real).
        // This stub parks for one poll cycle and returns false so the
        // higher-level pairing loop can keep polling with its own deadline.
        Timer::after_millis(100).await;
        false
    }

    /// Detect NFC field presence with the default 500 ms timeout.
    #[cfg(feature = "embedded")]
    pub async fn detect_field() -> bool {
        detect_field_with_timeout(500).await
    }

    /// Read the 10-byte UID from the active NFC tag.
    ///
    /// The UID is available on the `NfcT` instance in `main.rs` after
    /// `activate()` completes; this stub returns a placeholder until the
    /// full NFCT read path is wired up.
    #[cfg(feature = "embedded")]
    pub async fn read_nfc_uid() -> Option<[u8; 10]> {
        #[cfg(feature = "embedded")]
        info!("📡 Reading NFC UID…");

        // Placeholder: real implementation reads from NfcT after activation.
        let uid = [0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];

        #[cfg(feature = "embedded")]
        info!("✅ NFC UID (stub): {:?}", uid);

        Some(uid)
    }

    /// Read 4 bytes from a Type 2 tag page (pages 4–19).
    #[cfg(feature = "embedded")]
    pub async fn read_nfc_page(page: u8) -> Option<[u8; 4]> {
        #[cfg(feature = "embedded")]
        info!("📡 Reading NFC page {}", page);

        let data = [0x00, 0x01, 0x02, 0x03];

        #[cfg(feature = "embedded")]
        info!("✅ NFC page {} data (stub): {:?}", page, data);

        Some(data)
    }

    /// Write 4 bytes to a Type 2 tag page (pages 4–19).
    #[cfg(feature = "embedded")]
    pub async fn write_nfc_page(page: u8, data: &[u8; 4]) -> bool {
        #[cfg(feature = "embedded")]
        {
            info!("📡 Writing NFC page {}", page);
            info!("   Data: {:?}", data);
            info!("✅ NFC page write (stub)");
        }
        #[cfg(not(feature = "embedded"))]
        let _ = (page, data);
        true
    }
}

/// NFC pairing and bonded-device management.
///
/// Pure data helpers and synchronous logic are available under both the `nfc`
/// and `embedded` features.  Async pairing orchestration requires `embedded`.
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub mod pairing {
    use super::*;

    // ------------------------------------------------------------------
    // Flash storage layout constants
    // ------------------------------------------------------------------

    /// Start address of the bonded-device storage region in flash.
    pub const FLASH_BONDED_DEVICE_START: u32 = 0x2000;
    /// Size of the bonded-device storage region (8 KB).
    pub const FLASH_BONDED_DEVICE_SIZE: usize = 8 * 1024;
    /// Byte size of one serialised `BondedDevice` (6 MAC + 4 timestamp + 1 flags).
    pub const BONDED_DEVICE_STRUCT_SIZE: usize = 11;

    // ------------------------------------------------------------------
    // Synchronous helpers (available on host for testing)
    // ------------------------------------------------------------------

    /// Read the bonded device MAC from flash (stub; returns simulated value).
    ///
    /// Real implementation will use the nRF52840 NVMC peripheral.
    pub fn read_bonded_device_mac() -> Option<[u8; 6]> {
        #[cfg(feature = "embedded")]
        {
            info!(
                "🔑 Reading bonded MAC from flash (0x{:04X})",
                FLASH_BONDED_DEVICE_START
            );
        }
        Some([0x00, 11, 22, 33, 44, 55])
    }

    /// Read the full `BondedDevice` structure from flash (stub).
    pub fn read_bonded_device() -> Option<BondedDevice> {
        Some(BondedDevice {
            mac: [0x00, 11, 22, 33, 44, 55],
            timestamp: 1_234_567_890,
            flags: 0b111,
        })
    }

    /// Authenticate a device by comparing its MAC against the stored record.
    ///
    /// A device passes authentication when:
    /// - Its MAC matches the stored MAC exactly.
    /// - The stored record is active (`flags` bit 0 set).
    /// - The stored record is marked as paired (`flags` bit 1 set).
    pub fn authenticate_bonded_device(mac: &[u8; 6]) -> bool {
        #[cfg(feature = "embedded")]
        info!("🔐 Authenticating device MAC: {:?}", mac);

        match read_bonded_device() {
            Some(device) if device.mac == *mac && device.is_active() && device.is_paired() => {
                #[cfg(feature = "embedded")]
                info!("✅ Authentication successful");
                true
            }
            _ => {
                #[cfg(feature = "embedded")]
                warn!("❌ Authentication failed – MAC not found or device not active/paired");
                false
            }
        }
    }

    /// Return the current pairing status.
    ///
    /// This is a stateless stub; a full implementation would track state
    /// in a shared atomic or mutex-protected variable.
    pub fn get_nfc_pairing_status() -> NfcPairingStatus {
        NfcPairingStatus::Idle
    }

    /// Write the bonded device MAC to flash (stub; logs intent).
    pub fn write_bonded_device_to_flash(mac: &[u8; 6]) -> bool {
        #[cfg(feature = "embedded")]
        {
            info!(
                "💾 Writing bonded MAC to flash (0x{:04X})",
                FLASH_BONDED_DEVICE_START
            );
            info!("   MAC: {:?}", mac);
        }
        #[cfg(not(feature = "embedded"))]
        let _ = mac;
        true
    }

    /// Write the full `BondedDevice` structure to flash (stub).
    pub fn write_bonded_device_to_flash_full(device: &BondedDevice) -> bool {
        #[cfg(feature = "embedded")]
        {
            info!(
                "💾 Writing BondedDevice to flash (0x{:04X})",
                FLASH_BONDED_DEVICE_START
            );
            info!("   Device: {:?}", device);
        }
        #[cfg(not(feature = "embedded"))]
        let _ = device;
        true
    }

    /// Register a new bonded device (stub; in production writes to NVMC).
    pub fn register_bonded_device(mac: &[u8; 6]) -> bool {
        #[cfg(feature = "embedded")]
        info!("📝 Registering bonded device MAC: {:?}", mac);
        write_bonded_device_to_flash(mac)
    }

    /// Unregister a bonded device by MAC (stub).
    pub fn unregister_bonded_device(mac: &[u8; 6]) -> bool {
        #[cfg(feature = "embedded")]
        info!("🗑️  Unregistering bonded device MAC: {:?}", mac);
        #[cfg(not(feature = "embedded"))]
        let _ = mac;
        true
    }

    /// Return all registered bonded devices (stub; returns two simulated entries).
    pub fn get_bonded_devices() -> [BondedDevice; 2] {
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

    /// Return `true` if `mac` is in the bonded device list.
    pub fn is_bonded_device(mac: &[u8; 6]) -> bool {
        get_bonded_devices().iter().any(|d| d.mac == *mac)
    }

    // ------------------------------------------------------------------
    // Async pairing orchestration — requires embassy runtime
    // ------------------------------------------------------------------

    /// Run the full NFC pairing gate.
    ///
    /// Waits up to `NFC_PAIRING_TIMEOUT_SECS` for an NFC field, then
    /// reads and authenticates the bonded device MAC.  Returns `true` on
    /// success, `false` on timeout or authentication failure.
    #[cfg(feature = "embedded")]
    pub async fn pairing_mode() -> bool {
        use embassy_time::{Duration, Instant, Timer};

        info!("═══════════════════════════════════════════");
        info!("🔌 NFC Pairing Mode – Primary Authentication Gate");
        info!("   Timeout: {} s", NFC_PAIRING_TIMEOUT_SECS);
        info!("═══════════════════════════════════════════");

        let start = Instant::now();
        let timeout = Duration::from_secs(NFC_PAIRING_TIMEOUT_SECS);

        loop {
            if nfc::detect_field().await {
                info!("✅ NFC field detected – initiating pairing…");

                match read_bonded_device_mac() {
                    Some(mac) => {
                        if authenticate_bonded_device(&mac) {
                            return true;
                        } else {
                            warn!("⚠️  Bonded device authentication failed");
                            return false;
                        }
                    }
                    None => {
                        warn!("⚠️  No bonded device MAC in flash – pairing aborted");
                        return false;
                    }
                }
            }

            if start.elapsed() > timeout {
                info!("⏱️  NFC pairing timeout – falling back to BLE advertising");
                return false;
            }

            Timer::after_millis(100).await;
        }
    }
}

// ============================================================================
// TOP-LEVEL NFC API RE-EXPORTS
//
// The README documents these as top-level crate functions so callers don't
// need to know whether they live in `pairing` or some other sub-module.
// ============================================================================

/// Verify a device MAC against the stored bonded device record.
///
/// Returns `true` when the MAC matches the stored record and the record is
/// both active and paired.  Delegates to [`pairing::authenticate_bonded_device`].
///
/// # Example
/// ```rust,ignore
/// if authenticate_bonded_device(&mac) {
///     // allow BLE connection
/// }
/// ```
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub fn authenticate_bonded_device(mac: &[u8; 6]) -> bool {
    pairing::authenticate_bonded_device(mac)
}

/// Return the current NFC pairing status.
///
/// Delegates to [`pairing::get_nfc_pairing_status`].  A full implementation
/// would track state across the pairing flow; this stub always returns
/// [`NfcPairingStatus::Idle`].
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub fn get_nfc_pairing_status() -> NfcPairingStatus {
    pairing::get_nfc_pairing_status()
}
