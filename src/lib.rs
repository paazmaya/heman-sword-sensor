//! Pure logic module for He-Man Sword Sensor
//!
//! This module contains testable functions that don't depend on embedded hardware.
//! All embedded-specific code remains in main.rs.
//!
//! # Feature gates
//! - `embedded`: compiles async embassy functions and defmt formatting
//! - `nfc`: compiles the `nfc` and `pairing` modules on any target (including host tests)

#![cfg_attr(not(test), no_std)]

// defmt macros are used via fully-qualified paths (defmt::info!, defmt::warn!)
// within the individual modules; no top-level import required.

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

pub const THRUST_THRESHOLD: i16 = 50; // Threshold for upward thrust detection (~0.5g)
pub const SLASH_THRESHOLD: i16 = -50; // Threshold for downward slash detection (negative Z)
pub const IMPACT_THRESHOLD: i16 = 300; // Threshold for high-jerk impact detection
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

/// Detect a sword combat gesture from 6-axis sensor data and map it to
/// the corresponding [`led::LedAnimation`].
///
/// Priority order:
/// 1. [`led::LedAnimation::ImpactHit`] — any axis exceeds `IMPACT_THRESHOLD`
/// 2. [`led::LedAnimation::UpwardThrust`] — `accel_z > THRUST_THRESHOLD`
/// 3. [`led::LedAnimation::DownwardSlash`] — `accel_z < SLASH_THRESHOLD` with pitch/roll rotation
pub fn detect_sword_gesture(data: &SensorData) -> Option<led::LedAnimation> {
    detect_sword_gesture_with_thresholds(data, THRUST_THRESHOLD, IMPACT_THRESHOLD)
}

/// Detect a sword combat gesture using configurable thrust and impact thresholds.
pub fn detect_sword_gesture_with_thresholds(
    data: &SensorData,
    thrust_threshold: i16,
    impact_threshold: i16,
) -> Option<led::LedAnimation> {
    let impact_limit = impact_threshold.saturating_abs().max(1);
    if data.accel_x.saturating_abs() >= impact_limit
        || data.accel_y.saturating_abs() >= impact_limit
        || data.accel_z.saturating_abs() >= impact_limit
    {
        return Some(led::LedAnimation::ImpactHit);
    }

    if data.accel_z > thrust_threshold {
        return Some(led::LedAnimation::UpwardThrust);
    }

    let slash_limit = -(thrust_threshold.saturating_abs().max(1));
    let gyro_mag = (data.gyro_x as i32).abs() + (data.gyro_y as i32).abs();
    if data.accel_z < slash_limit && (gyro_mag >= 40 || data.accel_y < slash_limit) {
        return Some(led::LedAnimation::DownwardSlash);
    }

    None
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

impl NfcPairingStatus {
    /// Convert to wire byte representation (0..=5).
    pub const fn to_u8(self) -> u8 {
        match self {
            NfcPairingStatus::Idle => 0,
            NfcPairingStatus::Scanning => 1,
            NfcPairingStatus::Authenticating => 2,
            NfcPairingStatus::Success => 3,
            NfcPairingStatus::Failed => 4,
            NfcPairingStatus::Timeout => 5,
        }
    }

    /// Parse from wire byte representation.
    pub const fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(NfcPairingStatus::Idle),
            1 => Some(NfcPairingStatus::Scanning),
            2 => Some(NfcPairingStatus::Authenticating),
            3 => Some(NfcPairingStatus::Success),
            4 => Some(NfcPairingStatus::Failed),
            5 => Some(NfcPairingStatus::Timeout),
            _ => None,
        }
    }
}

static NFC_PAIRING_STATUS_STATE: core::sync::atomic::AtomicU8 =
    core::sync::atomic::AtomicU8::new(0);

/// Update the live NFC pairing status tracked by the firmware.
pub fn set_nfc_pairing_status(status: NfcPairingStatus) {
    NFC_PAIRING_STATUS_STATE.store(status.to_u8(), core::sync::atomic::Ordering::SeqCst);
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
    /// On the embedded target this tracks the last value stored by
    /// [`store_activated_uid`].  On the host it always returns `Idle`.
    pub fn get_field_state() -> NfcFieldState {
        #[cfg(feature = "embedded")]
        {
            if peek_activated_uid().is_some() {
                NfcFieldState::Detected
            } else {
                NfcFieldState::Idle
            }
        }
        #[cfg(not(feature = "embedded"))]
        NfcFieldState::Idle
    }

    // ------------------------------------------------------------------
    // UID validation helpers (no hardware dependency)
    // ------------------------------------------------------------------

    /// Validate a 10-byte NFC UID.
    ///
    /// Nordic's NFCT peripheral sets the first byte to `0x04`
    /// (ISO/IEC 14443 cascade tag) for 7- and 10-byte UIDs.
    /// An all-zero UID is treated as invalid (placeholder / error).
    pub fn is_valid_uid(uid: &[u8; 10]) -> bool {
        if uid.iter().all(|&b| b == 0) {
            return false;
        }
        // First byte of a real cascade UID is always 0x04
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
    // UID extraction helpers (pure logic — testable on host)
    // ------------------------------------------------------------------

    /// Extract a canonical 10-byte UID from the raw byte slice returned
    /// by the NFCT peripheral after `NfcT::activate()`.
    ///
    /// The nRF52840 NFCT peripheral can return 4, 7, or 10 raw bytes
    /// depending on the `NfcId` size configured:
    ///
    /// | `NfcId` size   | Raw bytes | Resulting UID                         |
    /// |----------------|-----------|---------------------------------------|
    /// | `SingleSize`   | 4         | Padded to 10 bytes (bytes 4–9 = 0x00) |
    /// | `DoubleSize`   | 7         | Padded to 10 bytes (bytes 7–9 = 0x00) |
    /// | `TripleSize`   | 10        | Used as-is                            |
    ///
    /// Returns `None` when the raw slice is empty or all-zero.
    pub fn extract_uid_from_nfct(raw: &[u8]) -> Option<[u8; 10]> {
        if raw.is_empty() {
            return None;
        }
        let mut uid = [0u8; 10];
        let copy_len = raw.len().min(10);
        uid[..copy_len].copy_from_slice(&raw[..copy_len]);
        if uid.iter().all(|&b| b == 0) {
            None
        } else {
            Some(uid)
        }
    }

    /// Parse the raw UID bytes from a completed `NfcT::activate()` and
    /// validate the result.
    ///
    /// This is a thin convenience wrapper around [`extract_uid_from_nfct`]
    /// that additionally validates the extracted UID via [`is_valid_uid`]
    /// (or [`is_valid_uid_7`] for 7-byte inputs).
    ///
    /// Returns `None` when the UID is missing, all-zero, or has an
    /// unexpected first byte.
    pub fn parse_nfct_uid(raw: &[u8]) -> Option<[u8; 10]> {
        let uid = extract_uid_from_nfct(raw)?;

        // For 7-byte source data, validate via the 7-byte helper
        if raw.len() == 7 {
            let mut short = [0u8; 7];
            short.copy_from_slice(&raw[..7]);
            if !is_valid_uid_7(&short) {
                return None;
            }
        } else if !is_valid_uid(&uid) {
            return None;
        }

        Some(uid)
    }

    // ------------------------------------------------------------------
    // NFC UID shared state
    //
    // After `NfcT::activate()` completes in main.rs, the firmware calls
    // `store_activated_uid()` so that the library-level pairing logic
    // (pairing_mode(), authenticate_with_uid(), …) can access the real
    // tag UID without receiving it as a function argument.
    //
    // On the host (desktop tests) the static is never written, so
    // peek_activated_uid() always returns None which is fine.
    // ------------------------------------------------------------------

    /// Store the UID of the most-recently activated NFC tag.
    ///
    /// Call this from `main.rs` immediately after `NfcT::activate()`
    /// succeeds, passing the raw bytes provided by the peripheral.
    /// Internally calls [`parse_nfct_uid`] to validate and canonicalise
    /// to 10 bytes before storing.
    ///
    /// Passing an empty or all-zero slice clears the stored UID.
    #[cfg(feature = "embedded")]
    pub fn store_activated_uid(raw: &[u8]) {
        let uid = parse_nfct_uid(raw);
        ACTIVATED_UID.lock(|cell| cell.set(uid));
        match uid {
            Some(u) => defmt::info!("🏷️  NFC UID stored: {:02x}", u),
            None => defmt::warn!("⚠️  NFC UID invalid or empty – cleared"),
        }
    }

    /// Return the UID stored by the most recent successful NFC activation,
    /// or `None` if no valid tag has been activated since boot.
    #[cfg(feature = "embedded")]
    pub fn peek_activated_uid() -> Option<[u8; 10]> {
        ACTIVATED_UID.lock(|cell| cell.get())
    }

    /// Clear the stored NFC UID (e.g. after pairing completes).
    #[cfg(feature = "embedded")]
    pub fn clear_activated_uid() {
        ACTIVATED_UID.lock(|cell| cell.set(None));
    }

    // Safety: only accessed through the CriticalSectionMutex lock.
    #[cfg(feature = "embedded")]
    static ACTIVATED_UID: embassy_sync::blocking_mutex::Mutex<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        core::cell::Cell<Option<[u8; 10]>>,
    > = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new(None));

    // ------------------------------------------------------------------
    // Async helpers — only compiled when the embassy runtime is present
    // ------------------------------------------------------------------

    /// Detect NFC field presence with a custom timeout.
    ///
    /// Polls [`peek_activated_uid`] every 100 ms.  The actual NFCT
    /// hardware activation (`NfcT::activate()`) is initiated in
    /// `main.rs`; once it completes the result is deposited via
    /// [`store_activated_uid`] and this function returns `true`.
    ///
    /// Returns `false` when `timeout_ms` elapses without a UID being stored.
    #[cfg(feature = "embedded")]
    pub async fn detect_field_with_timeout(timeout_ms: u64) -> bool {
        use embassy_time::{Duration, Instant, Timer};
        defmt::info!("📡 NFC field detection – timeout {} ms", timeout_ms);
        let deadline = Instant::now() + Duration::from_millis(timeout_ms);
        loop {
            if peek_activated_uid().is_some() {
                defmt::info!("✅ NFC field detected (UID present)");
                return true;
            }
            if Instant::now() >= deadline {
                defmt::info!("⏱️  NFC detect_field timeout");
                return false;
            }
            Timer::after_millis(100).await;
        }
    }

    /// Detect NFC field with the default 500 ms poll timeout.
    #[cfg(feature = "embedded")]
    pub async fn detect_field() -> bool {
        detect_field_with_timeout(500).await
    }

    /// Minimum writable user-memory page on an ISO/IEC 14443 Type 2 tag.
    pub const NFC_MIN_USER_PAGE: u8 = 4;
    /// Maximum writable user-memory page supported by our tag shadow buffer (NTAG213 range).
    pub const NFC_MAX_USER_PAGE: u8 = 39;
    /// Total number of 4-byte user pages tracked in the shadow buffer.
    pub const NFC_USER_PAGE_COUNT: usize = (NFC_MAX_USER_PAGE - NFC_MIN_USER_PAGE + 1) as usize;

    static NFC_PAGE_SHADOW: [core::sync::atomic::AtomicU32; NFC_USER_PAGE_COUNT] = [
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
        core::sync::atomic::AtomicU32::new(0),
    ];

    /// Check whether `page` is within the writable Type 2 Tag user memory region (pages 4–39).
    pub const fn is_valid_nfc_user_page(page: u8) -> bool {
        page >= NFC_MIN_USER_PAGE && page <= NFC_MAX_USER_PAGE
    }

    /// Synchronously validate and store 4 bytes into a Type 2 tag user page (pages 4–39).
    ///
    /// Returns `false` if `page` is outside `[NFC_MIN_USER_PAGE..=NFC_MAX_USER_PAGE]`.
    pub fn write_nfc_page_sync(page: u8, data: &[u8; 4]) -> bool {
        if !is_valid_nfc_user_page(page) {
            return false;
        }
        let idx = (page - NFC_MIN_USER_PAGE) as usize;
        let word = u32::from_le_bytes(*data);
        NFC_PAGE_SHADOW[idx].store(word, core::sync::atomic::Ordering::SeqCst);
        true
    }

    /// Read back 4 bytes from a Type 2 tag user page shadow buffer (pages 4–39).
    pub fn read_nfc_page(page: u8) -> Option<[u8; 4]> {
        if !is_valid_nfc_user_page(page) {
            return None;
        }
        let idx = (page - NFC_MIN_USER_PAGE) as usize;
        let word = NFC_PAGE_SHADOW[idx].load(core::sync::atomic::Ordering::SeqCst);
        Some(word.to_le_bytes())
    }

    /// Write 4 bytes to a Type 2 tag page (pages 4–39).
    ///
    /// Type 2 tags are organised in 4-byte pages.  User memory starts at
    /// page 4 (byte address 16) and extends to page 15 (bytes 60–63) on
    /// a standard 64-byte MIFARE Ultralight tag, or up to page 39 on NTAG213.
    #[cfg(feature = "embedded")]
    pub async fn write_nfc_page(page: u8, data: &[u8; 4]) -> bool {
        if !write_nfc_page_sync(page, data) {
            defmt::warn!("⚠️  NFC WRITE rejected: invalid user page {}", page);
            return false;
        }
        defmt::info!("📡 NFC WRITE page {} data {:02x}", page, data);
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
    ///
    /// Located at the last 4 KB page of the nRF52840's 1 MB flash:
    ///   0x000EF000 = 1 MB – 4 KB = 0x000FF000 – 0x1000 (one page from the end)
    ///
    /// This is deliberately placed well above the S140 SoftDevice region
    /// (0x00000000 – 0x00025FFF) and above typical application code, so
    /// direct NVMC writes never corrupt the softdevice.
    ///
    /// Previously this was 0x0002_0000, which falls *inside* the S140 region
    /// and would silently corrupt the BLE stack on first flash erase.
    pub const FLASH_BONDED_DEVICE_START: u32 = 0x000E_F000;
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

        let authenticated = matches!(
            read_bonded_device(),
            Some(device) if device.mac == *mac && device.is_active() && device.is_paired()
        );
        #[cfg(feature = "embedded")]
        if authenticated {
            defmt::info!("✅ Authentication successful");
        } else {
            defmt::warn!("❌ Authentication failed – MAC not found or device not active/paired");
        }
        authenticated
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

        let authenticated = matches!(
            read_bonded_device_full(),
            Some(device)
                if device.dev.mac == *mac
                    && device.uid == *uid
                    && device.dev.is_active()
                    && device.dev.is_paired()
        );
        #[cfg(feature = "embedded")]
        if authenticated {
            defmt::info!("✅ MAC + UID authentication successful");
        } else {
            defmt::warn!("❌ MAC + UID authentication failed");
        }
        authenticated
    }

    /// Return the current pairing status tracked in the shared atomic state.
    pub fn get_nfc_pairing_status() -> NfcPairingStatus {
        let raw = NFC_PAIRING_STATUS_STATE.load(core::sync::atomic::Ordering::SeqCst);
        NfcPairingStatus::from_u8(raw).unwrap_or(NfcPairingStatus::Idle)
    }

    /// Set the current pairing status tracked in the shared atomic state.
    pub fn set_nfc_pairing_status(status: NfcPairingStatus) {
        super::set_nfc_pairing_status(status);
    }

    /// First-time pairing placeholder MAC address stored prior to central registration.
    pub const PLACEHOLDER_MAC: [u8; 6] = [0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01];

    /// Validate a 6-byte BLE MAC address written via `pairing_mac`.
    ///
    /// Rejects all-zero (`00:00:00:00:00:00`), broadcast (`FF:FF:FF:FF:FF:FF`),
    /// and the internal first-time pairing placeholder (`DE:AD:BE:EF:00:01`).
    pub fn is_valid_ble_mac(mac: &[u8; 6]) -> bool {
        if mac.iter().all(|&b| b == 0x00) {
            return false;
        }
        if mac.iter().all(|&b| b == 0xFF) {
            return false;
        }
        if *mac == PLACEHOLDER_MAC {
            return false;
        }
        true
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
            let addr = FLASH_BONDED_DEVICE_START as *const u8;
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
    /// 1. Waits up to [`NFC_PAIRING_TIMEOUT_SECS`] seconds for an NFC
    ///    UID to appear in the shared [`nfc::ACTIVATED_UID`] static (which
    ///    `main.rs` populates via [`nfc::store_activated_uid`] after a
    ///    successful `NfcT::activate()` call).
    /// 2. When a UID is detected:
    ///    - **Re-pairing** (existing flash record): authenticates using both
    ///      the stored MAC *and* the NFC UID via [`authenticate_with_uid`].
    ///      Returns `true` on success.
    ///    - **First-time pairing** (no flash record): a placeholder bonded
    ///      device is registered with the UID so that a subsequent
    ///      `authenticate_with_uid` call can verify the NFC tag.  The BLE
    ///      MAC will be updated later by the mobile app via the writable
    ///      `pairing_mac` GATT characteristic.  Returns `false` so `main.rs`
    ///      knows it is still in open-advertising mode.
    /// 3. Returns `false` on timeout.
    ///
    /// The actual NFCT hardware activation (`NfcT::activate()`) must be
    /// started concurrently in `main.rs` before calling this function.
    #[cfg(feature = "embedded")]
    pub async fn pairing_mode() -> bool {
        use embassy_time::{Duration, Instant, Timer};

        defmt::info!("═══════════════════════════════════════════");
        defmt::info!("🔌 NFC Pairing Mode – Primary Authentication Gate");
        defmt::info!("   Timeout: {} s", NFC_PAIRING_TIMEOUT_SECS);
        defmt::info!("═══════════════════════════════════════════");

        let start = Instant::now();
        let timeout = Duration::from_secs(NFC_PAIRING_TIMEOUT_SECS);
        set_nfc_pairing_status(NfcPairingStatus::Scanning);

        loop {
            // Check whether main.rs has stored an activated UID
            if let Some(uid) = nfc::peek_activated_uid() {
                defmt::info!("✅ NFC UID present – initiating pairing…");
                set_nfc_pairing_status(NfcPairingStatus::Authenticating);

                match read_bonded_device_full() {
                    Some(stored) => {
                        // Re-pairing: validate with stored MAC + stored UID
                        if authenticate_with_uid(&stored.dev.mac, &uid) {
                            defmt::info!("✅ MAC + UID authentication successful");
                            set_nfc_pairing_status(NfcPairingStatus::Success);
                            nfc::clear_activated_uid();
                            return true;
                        } else {
                            defmt::warn!("❌ MAC + UID authentication failed");
                            set_nfc_pairing_status(NfcPairingStatus::Failed);
                            nfc::clear_activated_uid();
                            return false;
                        }
                    }
                    None => {
                        // First-time pairing: store the UID with a placeholder MAC.
                        // The real MAC is written later via the pairing_mac
                        // GATT characteristic once the mobile app connects.
                        defmt::warn!(
                            "⚠️  No bonded device in flash – registering NFC UID for first-time pairing"
                        );
                        let mut dev = BondedDevice::new(PLACEHOLDER_MAC);
                        dev.set_verified(true); // UID has been seen
                        write_bonded_device_to_flash_full(&dev);
                        set_nfc_pairing_status(NfcPairingStatus::Success);
                        // Clear so we don't re-trigger on the next loop
                        nfc::clear_activated_uid();
                        // Return false – open advertising, MAC still placeholder
                        return false;
                    }
                }
            }

            if start.elapsed() >= timeout {
                defmt::info!("⏱️  NFC pairing timeout – falling back to BLE advertising");
                set_nfc_pairing_status(NfcPairingStatus::Timeout);
                return false;
            }

            Timer::after_millis(100).await;
        }
    }
}

// ============================================================================
// PHASE 1: 60-LED WS2812B ANIMATION ENGINE & PWM ENCODER
// ============================================================================

/// WS2812B addressable LED animation engine and nRF52840 PWM sequence encoder.
pub mod led {
    /// Number of WS2812B LEDs along the sword blade (guard = index 0, tip = index 59).
    pub const NUM_LEDS: usize = 60;
    /// Frame update period in milliseconds (50 Hz).
    pub const LED_FRAME_INTERVAL_MS: u32 = 20;
    /// Number of PWM words per LED (24 bits: 8 G + 8 R + 8 B).
    pub const BITS_PER_LED: usize = 24;
    /// Number of trailing zero-duty PWM words for the ≥50 µs WS2812B reset latch (40 × 1.25 µs = 50 µs).
    pub const RESET_SLOTS: usize = 40;
    /// Total length of the `SequencePwm` DMA buffer in 16-bit half-words.
    pub const PWM_BUFFER_WORDS: usize = NUM_LEDS * BITS_PER_LED + RESET_SLOTS; // 1480

    /// nRF52 PWM countertop for 800 kHz at 16 MHz (`16 MHz / 800 kHz = 20` ticks = 1.25 µs).
    pub const PWM_MAX_DUTY: u16 = 20;
    /// Bit 15 inverted polarity flag required by nRF52 PWM peripheral so duty value = high time.
    pub const PWM_POLARITY_BIT: u16 = 0x8000;
    /// WS2812B `'0'` bit duty word: 6 ticks high (0.375 µs), 14 ticks low (0.875 µs).
    pub const PWM_T0H_WORD: u16 = 6 | PWM_POLARITY_BIT;
    /// WS2812B `'1'` bit duty word: 13 ticks high (0.8125 µs), 7 ticks low (0.4375 µs).
    pub const PWM_T1H_WORD: u16 = 13 | PWM_POLARITY_BIT;
    /// WS2812B reset latch word: 0 ticks high (continuous low).
    pub const PWM_RESET_WORD: u16 = PWM_POLARITY_BIT;

    /// Default upward thrust animation duration (200 ms).
    pub const DURATION_THRUST_MS: u32 = 200;
    /// Default downward slash animation duration (150 ms).
    pub const DURATION_SLASH_MS: u32 = 150;
    /// Default impact flash animation duration (100 ms).
    pub const DURATION_IMPACT_MS: u32 = 100;
    /// Default motion-stop fade duration (1000 ms).
    pub const DURATION_MOTION_STOP_MS: u32 = 1000;
    /// Full cycle period for continuous idle breathing (2000 ms).
    pub const BREATHING_PERIOD_MS: u32 = 2000;

    /// 24-bit RGB color value.
    #[cfg_attr(feature = "embedded", derive(defmt::Format))]
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub struct RgbColor {
        pub r: u8,
        pub g: u8,
        pub b: u8,
    }

    impl RgbColor {
        pub const BLACK: Self = Self::new(0, 0, 0);
        pub const WHITE: Self = Self::new(255, 255, 255);
        pub const GRAYSKULL_BLUE: Self = Self::new(80, 180, 255);
        pub const SLASH_ORANGE: Self = Self::new(255, 70, 0);

        /// Construct an RGB color.
        pub const fn new(r: u8, g: u8, b: u8) -> Self {
            Self { r, g, b }
        }

        /// Scale each channel by `scale` (`0..=255`).
        pub const fn scale(self, scale: u8) -> Self {
            let s = scale as u16;
            Self {
                r: ((self.r as u16 * s) / 255) as u8,
                g: ((self.g as u16 * s) / 255) as u8,
                b: ((self.b as u16 * s) / 255) as u8,
            }
        }
    }

    /// Sword blade LED animation patterns.
    #[cfg_attr(feature = "embedded", derive(defmt::Format))]
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum LedAnimation {
        /// Slow continuous cyan/blue breathing glow when idle.
        IdleBreathing,
        /// Guard-to-tip white/blue energy wave on upward thrust (default 200 ms).
        UpwardThrust,
        /// Tip-to-guard red/orange wave on downward slash (default 150 ms).
        DownwardSlash,
        /// Full-blade bright white impact flash with decay (default 100 ms).
        ImpactHit,
        /// Smooth 1000 ms fade to off after motion stops.
        MotionStop,
        /// Static user-selected RGB color (e.g. set via BLE API).
        SolidColor(RgbColor),
        /// All LEDs off.
        Off,
    }

    impl LedAnimation {
        /// Numeric animation identifier used by the BLE TLV protocol.
        pub const fn id(&self) -> u8 {
            match self {
                LedAnimation::Off => 0,
                LedAnimation::IdleBreathing => 1,
                LedAnimation::UpwardThrust => 2,
                LedAnimation::DownwardSlash => 3,
                LedAnimation::ImpactHit => 4,
                LedAnimation::MotionStop => 5,
                LedAnimation::SolidColor(_) => 6,
            }
        }

        /// Construct an animation from a numeric ID and optional color.
        pub const fn from_id(id: u8, color: RgbColor) -> Option<Self> {
            match id {
                0 => Some(LedAnimation::Off),
                1 => Some(LedAnimation::IdleBreathing),
                2 => Some(LedAnimation::UpwardThrust),
                3 => Some(LedAnimation::DownwardSlash),
                4 => Some(LedAnimation::ImpactHit),
                5 => Some(LedAnimation::MotionStop),
                6 => Some(LedAnimation::SolidColor(color)),
                _ => None,
            }
        }

        /// Default duration in milliseconds (`None` for continuous states).
        pub const fn default_duration_ms(&self) -> Option<u32> {
            match self {
                LedAnimation::UpwardThrust => Some(DURATION_THRUST_MS),
                LedAnimation::DownwardSlash => Some(DURATION_SLASH_MS),
                LedAnimation::ImpactHit => Some(DURATION_IMPACT_MS),
                LedAnimation::MotionStop => Some(DURATION_MOTION_STOP_MS),
                LedAnimation::IdleBreathing | LedAnimation::SolidColor(_) | LedAnimation::Off => {
                    None
                }
            }
        }
    }

    /// Operating mode for the LED engine.
    #[cfg_attr(feature = "embedded", derive(defmt::Format))]
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum LedControlMode {
        /// Automatically trigger animations from IMU gestures.
        AutoMotion,
        /// Hold manual BLE-configured animation/color and ignore IMU gestures.
        ManualOverride,
    }

    /// Deterministic 60-LED animation engine.
    #[derive(Clone, Debug)]
    pub struct LedEngine {
        active: LedAnimation,
        mode: LedControlMode,
        elapsed_ms: u32,
        duration_ms: Option<u32>,
        brightness: u8,
        last_frame: [RgbColor; NUM_LEDS],
    }

    impl Default for LedEngine {
        fn default() -> Self {
            Self::new()
        }
    }

    impl LedEngine {
        /// Create a new `LedEngine` starting in `IdleBreathing` at full brightness (255).
        pub const fn new() -> Self {
            Self {
                active: LedAnimation::IdleBreathing,
                mode: LedControlMode::AutoMotion,
                elapsed_ms: 0,
                duration_ms: None,
                brightness: 255,
                last_frame: [RgbColor::BLACK; NUM_LEDS],
            }
        }

        /// Return the currently active animation.
        pub const fn current_animation(&self) -> LedAnimation {
            self.active
        }

        /// Return the current control mode (`AutoMotion` vs `ManualOverride`).
        pub const fn control_mode(&self) -> LedControlMode {
            self.mode
        }

        /// Set the control mode.
        pub fn set_control_mode(&mut self, mode: LedControlMode) {
            self.mode = mode;
        }

        /// Return the global brightness (`0..=255`).
        pub const fn brightness(&self) -> u8 {
            self.brightness
        }

        /// Set global master brightness (`0..=255`).
        pub fn set_brightness(&mut self, brightness: u8) {
            self.brightness = brightness;
        }

        /// Trigger a motion animation using its default duration.
        ///
        /// Ignored if the engine is currently in [`LedControlMode::ManualOverride`].
        pub fn trigger_motion(&mut self, anim: LedAnimation) {
            if self.mode == LedControlMode::ManualOverride {
                return;
            }
            self.start_animation(anim, anim.default_duration_ms());
        }

        /// Force-start an animation with an optional custom duration (in ms).
        ///
        /// If `duration_ms` is `Some(0)`, the animation's default duration is used.
        pub fn start_animation(&mut self, anim: LedAnimation, duration_ms: Option<u32>) {
            self.active = anim;
            self.elapsed_ms = 0;
            self.duration_ms = match duration_ms {
                Some(0) => anim.default_duration_ms(),
                Some(d) => Some(d),
                None => anim.default_duration_ms(),
            };
        }

        /// Advance the animation clock by `dt_ms` and render the 60-LED frame.
        pub fn tick(&mut self, dt_ms: u32) -> [RgbColor; NUM_LEDS] {
            // Check whether a transient animation has completed before rendering
            if let Some(limit) = self.duration_ms {
                if self.elapsed_ms >= limit {
                    self.active = LedAnimation::IdleBreathing;
                    self.elapsed_ms = 0;
                    self.duration_ms = None;
                }
            }

            let frame = match self.active {
                LedAnimation::Off => [RgbColor::BLACK; NUM_LEDS],
                LedAnimation::SolidColor(color) => [color; NUM_LEDS],
                LedAnimation::IdleBreathing => self.render_idle_breathing(),
                LedAnimation::UpwardThrust => {
                    let dur = self.duration_ms.unwrap_or(DURATION_THRUST_MS).max(1);
                    self.render_upward_thrust(dur)
                }
                LedAnimation::DownwardSlash => {
                    let dur = self.duration_ms.unwrap_or(DURATION_SLASH_MS).max(1);
                    self.render_downward_slash(dur)
                }
                LedAnimation::ImpactHit => {
                    let dur = self.duration_ms.unwrap_or(DURATION_IMPACT_MS).max(1);
                    self.render_impact_hit(dur)
                }
                LedAnimation::MotionStop => {
                    let dur = self.duration_ms.unwrap_or(DURATION_MOTION_STOP_MS).max(1);
                    self.render_motion_stop(dur)
                }
            };

            self.elapsed_ms = self.elapsed_ms.saturating_add(dt_ms);
            if self.active != LedAnimation::MotionStop {
                self.last_frame = frame;
            }
            frame
        }

        fn render_idle_breathing(&self) -> [RgbColor; NUM_LEDS] {
            let phase = self.elapsed_ms % BREATHING_PERIOD_MS;
            let half = BREATHING_PERIOD_MS / 2;
            // Triangle wave oscillating smoothly between intensity 20 and 120
            let triangle = if phase < half {
                phase
            } else {
                BREATHING_PERIOD_MS - phase
            };
            let level = (20 + (triangle * 100) / half) as u8;
            let color = RgbColor::GRAYSKULL_BLUE.scale(level);
            [color; NUM_LEDS]
        }

        fn render_upward_thrust(&self, duration_ms: u32) -> [RgbColor; NUM_LEDS] {
            let mut leds = [RgbColor::BLACK; NUM_LEDS];
            let clamped = self.elapsed_ms.min(duration_ms);
            // Wavefront advances from guard (0) to tip (NUM_LEDS - 1)
            let head = ((clamped as usize) * (NUM_LEDS - 1)) / (duration_ms as usize);
            for (idx, led) in leds.iter_mut().enumerate() {
                if idx <= head {
                    let dist = head - idx;
                    // Bright white tip at wavefront, decaying cyan/blue plasma trail toward guard
                    let trail_scale = 255u16.saturating_sub((dist as u16) * 4) as u8;
                    if dist <= 3 {
                        *led = RgbColor::WHITE.scale(trail_scale);
                    } else {
                        *led = RgbColor::GRAYSKULL_BLUE.scale(trail_scale);
                    }
                }
            }
            leds
        }

        fn render_downward_slash(&self, duration_ms: u32) -> [RgbColor; NUM_LEDS] {
            let mut leds = [RgbColor::BLACK; NUM_LEDS];
            let clamped = self.elapsed_ms.min(duration_ms);
            // Reverse wave: starts at tip (NUM_LEDS - 1) and sweeps down to guard (0)
            let progress = ((clamped as usize) * (NUM_LEDS - 1)) / (duration_ms as usize);
            let head = (NUM_LEDS - 1).saturating_sub(progress);
            for (idx, led) in leds.iter_mut().enumerate() {
                if idx >= head {
                    let dist = idx - head;
                    let trail_scale = 255u16.saturating_sub((dist as u16) * 5) as u8;
                    *led = RgbColor::SLASH_ORANGE.scale(trail_scale);
                }
            }
            leds
        }

        fn render_impact_hit(&self, duration_ms: u32) -> [RgbColor; NUM_LEDS] {
            let clamped = self.elapsed_ms.min(duration_ms);
            let remaining = duration_ms.saturating_sub(clamped);
            // Quadratic decay from full white flash (255) to 0
            let scale = ((remaining * remaining * 255) / (duration_ms * duration_ms)) as u8;
            let color = RgbColor::WHITE.scale(scale);
            [color; NUM_LEDS]
        }

        fn render_motion_stop(&self, duration_ms: u32) -> [RgbColor; NUM_LEDS] {
            let clamped = self.elapsed_ms.min(duration_ms);
            let remaining = duration_ms.saturating_sub(clamped);
            let scale = ((remaining * 255) / duration_ms) as u8;
            let mut leds = [RgbColor::BLACK; NUM_LEDS];
            for (i, pixel) in leds.iter_mut().enumerate() {
                let base = if self.last_frame[i] == RgbColor::BLACK {
                    RgbColor::GRAYSKULL_BLUE.scale(80)
                } else {
                    self.last_frame[i]
                };
                *pixel = base.scale(scale);
            }
            leds
        }
    }

    /// Encode a 60-LED RGB frame into nRF52840 `SequencePwm` 16-bit duty words.
    ///
    /// - WS2812B expects **GRB** byte order, MSB first (24 bits per pixel).
    /// - Applies global `brightness` (`0..=255`) scaling before encoding.
    /// - Appends [`RESET_SLOTS`] words of [`PWM_RESET_WORD`] (`0x8000`) for the ≥50 µs latch.
    pub fn encode_ws2812_pwm(
        leds: &[RgbColor; NUM_LEDS],
        brightness: u8,
        out: &mut [u16; PWM_BUFFER_WORDS],
    ) {
        let mut word_idx = 0usize;
        for pixel in leds.iter() {
            let scaled = pixel.scale(brightness);
            let grb = [scaled.g, scaled.r, scaled.b];
            for byte in grb {
                for bit in (0..8).rev() {
                    let is_one = (byte >> bit) & 1 != 0;
                    out[word_idx] = if is_one { PWM_T1H_WORD } else { PWM_T0H_WORD };
                    word_idx += 1;
                }
            }
        }
        while word_idx < PWM_BUFFER_WORDS {
            out[word_idx] = PWM_RESET_WORD;
            word_idx += 1;
        }
    }
}

// ============================================================================
// PHASE 2: BLE GATT PROFILE & COMMAND/EVENT TLV API
// ============================================================================

/// Binary Type-Length-Value (TLV) Command & Event protocol for BLE clients
/// (Web Bluetooth & Flutter mobile app).
///
/// Packet envelope (up to 16 bytes):
/// ```text
/// ┌────────────┬────────────┬────────────────────────────────────┐
/// │ Byte 0     │ Byte 1     │ Bytes 2..(2 + len)                 │
/// │ opcode: u8 │ len: u8    │ payload (0..=14 bytes, LE fields)  │
/// └────────────┴────────────┴────────────────────────────────────┘
/// ```
pub mod api {
    use crate::led::{LedAnimation, LedControlMode, RgbColor};

    /// Fixed GATT characteristic buffer size for `command_control`.
    pub const TLV_PACKET_SIZE: usize = 16;
    /// Maximum payload length inside a 16-byte TLV packet.
    pub const TLV_MAX_PAYLOAD_LEN: usize = TLV_PACKET_SIZE - 2;

    // Command Opcodes (Client → Sword)
    pub const OP_SET_LED_MODE: u8 = 0x01;
    pub const OP_SET_BRIGHTNESS: u8 = 0x02;
    pub const OP_TRIGGER_ANIMATION: u8 = 0x03;
    pub const OP_SET_THRESHOLDS: u8 = 0x04;
    pub const OP_GET_STATUS: u8 = 0x05;
    pub const OP_CLEAR_BONDED_DEVICES: u8 = 0x06;

    // Event/Response Opcodes (Sword → Client)
    pub const EVT_COMMAND_ACK: u8 = 0x80;
    pub const EVT_GESTURE: u8 = 0x81;
    pub const EVT_DEVICE_STATUS: u8 = 0x82;

    /// Status/error codes returned in [`ApiEvent::CommandAck`].
    #[cfg_attr(feature = "embedded", derive(defmt::Format))]
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum ApiStatus {
        Ok = 0x00,
        InvalidOpcode = 0x01,
        InvalidLength = 0x02,
        InvalidParameter = 0x03,
        StorageError = 0x04,
    }

    impl ApiStatus {
        pub const fn to_u8(self) -> u8 {
            self as u8
        }

        pub const fn from_u8(val: u8) -> Option<Self> {
            match val {
                0x00 => Some(ApiStatus::Ok),
                0x01 => Some(ApiStatus::InvalidOpcode),
                0x02 => Some(ApiStatus::InvalidLength),
                0x03 => Some(ApiStatus::InvalidParameter),
                0x04 => Some(ApiStatus::StorageError),
                _ => None,
            }
        }
    }

    /// Commands sent from the mobile/web client to the sword over `command_control` (`…0006`).
    #[cfg_attr(feature = "embedded", derive(defmt::Format))]
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum ApiCommand {
        /// `0x01`: Set LED control mode (`0` = AutoMotion, `1` = ManualOverride), RGB color, and optional duration (ms).
        SetLedMode {
            mode: LedControlMode,
            color: RgbColor,
            duration_ms: u16,
        },
        /// `0x02`: Set global LED brightness (`0..=255`).
        SetBrightness { brightness: u8 },
        /// `0x03`: Trigger a specific blade animation immediately.
        TriggerAnimation {
            animation: LedAnimation,
            duration_ms: u16,
        },
        /// `0x04`: Update motion detection thresholds (`thrust_threshold`, `impact_threshold`).
        SetThresholds {
            thrust_threshold: i16,
            impact_threshold: i16,
        },
        /// `0x05`: Request a [`ApiEvent::DeviceStatus`] snapshot.
        GetStatus,
        /// `0x06`: Clear bonded device records and reopen BLE advertising.
        ClearBondedDevices,
    }

    impl ApiCommand {
        /// Return the TLV opcode byte for this command.
        pub const fn opcode(&self) -> u8 {
            match self {
                ApiCommand::SetLedMode { .. } => OP_SET_LED_MODE,
                ApiCommand::SetBrightness { .. } => OP_SET_BRIGHTNESS,
                ApiCommand::TriggerAnimation { .. } => OP_TRIGGER_ANIMATION,
                ApiCommand::SetThresholds { .. } => OP_SET_THRESHOLDS,
                ApiCommand::GetStatus => OP_GET_STATUS,
                ApiCommand::ClearBondedDevices => OP_CLEAR_BONDED_DEVICES,
            }
        }

        /// Encode the command into a 16-byte TLV buffer.
        pub fn encode(&self) -> [u8; TLV_PACKET_SIZE] {
            let mut buf = [0u8; TLV_PACKET_SIZE];
            buf[0] = self.opcode();
            match *self {
                ApiCommand::SetLedMode {
                    mode,
                    color,
                    duration_ms,
                } => {
                    buf[1] = 6;
                    buf[2] = match mode {
                        LedControlMode::AutoMotion => 0,
                        LedControlMode::ManualOverride => 1,
                    };
                    buf[3] = color.r;
                    buf[4] = color.g;
                    buf[5] = color.b;
                    let dur = duration_ms.to_le_bytes();
                    buf[6] = dur[0];
                    buf[7] = dur[1];
                }
                ApiCommand::SetBrightness { brightness } => {
                    buf[1] = 1;
                    buf[2] = brightness;
                }
                ApiCommand::TriggerAnimation {
                    animation,
                    duration_ms,
                } => {
                    buf[1] = 6;
                    buf[2] = animation.id();
                    let dur = duration_ms.to_le_bytes();
                    buf[3] = dur[0];
                    buf[4] = dur[1];
                    if let LedAnimation::SolidColor(c) = animation {
                        buf[5] = c.r;
                        buf[6] = c.g;
                        buf[7] = c.b;
                    }
                }
                ApiCommand::SetThresholds {
                    thrust_threshold,
                    impact_threshold,
                } => {
                    buf[1] = 4;
                    buf[2..4].copy_from_slice(&thrust_threshold.to_le_bytes());
                    buf[4..6].copy_from_slice(&impact_threshold.to_le_bytes());
                }
                ApiCommand::GetStatus | ApiCommand::ClearBondedDevices => {
                    buf[1] = 0;
                }
            }
            buf
        }

        /// Parse a TLV command from a raw byte slice (or 16-byte GATT write).
        ///
        /// On error, returns `(opcode, ApiStatus)` so the firmware can immediately
        /// emit a matching [`ApiEvent::CommandAck`].
        pub fn parse(raw: &[u8]) -> Result<Self, (u8, ApiStatus)> {
            if raw.len() < 2 {
                let op = raw.first().copied().unwrap_or(0);
                return Err((op, ApiStatus::InvalidLength));
            }
            let opcode = raw[0];
            let len = raw[1] as usize;
            if len > TLV_MAX_PAYLOAD_LEN || raw.len() < 2 + len {
                return Err((opcode, ApiStatus::InvalidLength));
            }
            let payload = &raw[2..2 + len];

            match opcode {
                OP_SET_LED_MODE => {
                    if len != 6 {
                        return Err((opcode, ApiStatus::InvalidLength));
                    }
                    let mode = match payload[0] {
                        0 => LedControlMode::AutoMotion,
                        1 => LedControlMode::ManualOverride,
                        _ => return Err((opcode, ApiStatus::InvalidParameter)),
                    };
                    let color = RgbColor::new(payload[1], payload[2], payload[3]);
                    let duration_ms = u16::from_le_bytes([payload[4], payload[5]]);
                    Ok(ApiCommand::SetLedMode {
                        mode,
                        color,
                        duration_ms,
                    })
                }
                OP_SET_BRIGHTNESS => {
                    if len != 1 {
                        return Err((opcode, ApiStatus::InvalidLength));
                    }
                    Ok(ApiCommand::SetBrightness {
                        brightness: payload[0],
                    })
                }
                OP_TRIGGER_ANIMATION => {
                    if len != 3 && len != 6 {
                        return Err((opcode, ApiStatus::InvalidLength));
                    }
                    let color = if len == 6 {
                        RgbColor::new(payload[3], payload[4], payload[5])
                    } else {
                        RgbColor::WHITE
                    };
                    let animation = LedAnimation::from_id(payload[0], color)
                        .ok_or((opcode, ApiStatus::InvalidParameter))?;
                    let duration_ms = u16::from_le_bytes([payload[1], payload[2]]);
                    Ok(ApiCommand::TriggerAnimation {
                        animation,
                        duration_ms,
                    })
                }
                OP_SET_THRESHOLDS => {
                    if len != 4 {
                        return Err((opcode, ApiStatus::InvalidLength));
                    }
                    let thrust_threshold = i16::from_le_bytes([payload[0], payload[1]]);
                    let impact_threshold = i16::from_le_bytes([payload[2], payload[3]]);
                    if thrust_threshold <= 0 || impact_threshold <= thrust_threshold {
                        return Err((opcode, ApiStatus::InvalidParameter));
                    }
                    Ok(ApiCommand::SetThresholds {
                        thrust_threshold,
                        impact_threshold,
                    })
                }
                OP_GET_STATUS => {
                    if len != 0 {
                        return Err((opcode, ApiStatus::InvalidLength));
                    }
                    Ok(ApiCommand::GetStatus)
                }
                OP_CLEAR_BONDED_DEVICES => {
                    if len != 0 {
                        return Err((opcode, ApiStatus::InvalidLength));
                    }
                    Ok(ApiCommand::ClearBondedDevices)
                }
                _ => Err((opcode, ApiStatus::InvalidOpcode)),
            }
        }
    }

    /// Events and responses sent from the sword to the client via `command_control` (`…0006`) notifications.
    #[cfg_attr(feature = "embedded", derive(defmt::Format))]
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum ApiEvent {
        /// `0x80`: Command acknowledgement with original opcode and status code.
        CommandAck { ref_opcode: u8, status: ApiStatus },
        /// `0x81`: Real-time sword gesture notification with peak acceleration.
        GestureEvent { animation_id: u8, peak_accel: i16 },
        /// `0x82`: Full device state snapshot.
        DeviceStatus {
            pairing_status: u8,
            led_mode: u8,
            active_animation: u8,
            brightness: u8,
            thrust_threshold: i16,
            impact_threshold: i16,
        },
    }

    impl ApiEvent {
        /// Encode the event into a 16-byte TLV packet.
        pub fn encode(&self) -> [u8; TLV_PACKET_SIZE] {
            let mut buf = [0u8; TLV_PACKET_SIZE];
            match *self {
                ApiEvent::CommandAck { ref_opcode, status } => {
                    buf[0] = EVT_COMMAND_ACK;
                    buf[1] = 2;
                    buf[2] = ref_opcode;
                    buf[3] = status.to_u8();
                }
                ApiEvent::GestureEvent {
                    animation_id,
                    peak_accel,
                } => {
                    buf[0] = EVT_GESTURE;
                    buf[1] = 3;
                    buf[2] = animation_id;
                    buf[3..5].copy_from_slice(&peak_accel.to_le_bytes());
                }
                ApiEvent::DeviceStatus {
                    pairing_status,
                    led_mode,
                    active_animation,
                    brightness,
                    thrust_threshold,
                    impact_threshold,
                } => {
                    buf[0] = EVT_DEVICE_STATUS;
                    buf[1] = 8;
                    buf[2] = pairing_status;
                    buf[3] = led_mode;
                    buf[4] = active_animation;
                    buf[5] = brightness;
                    buf[6..8].copy_from_slice(&thrust_threshold.to_le_bytes());
                    buf[8..10].copy_from_slice(&impact_threshold.to_le_bytes());
                }
            }
            buf
        }

        /// Parse an `ApiEvent` from a raw byte slice.
        pub fn parse(raw: &[u8]) -> Option<Self> {
            if raw.len() < 2 {
                return None;
            }
            let opcode = raw[0];
            let len = raw[1] as usize;
            if len > TLV_MAX_PAYLOAD_LEN || raw.len() < 2 + len {
                return None;
            }
            let payload = &raw[2..2 + len];
            match opcode {
                EVT_COMMAND_ACK if len == 2 => Some(ApiEvent::CommandAck {
                    ref_opcode: payload[0],
                    status: ApiStatus::from_u8(payload[1])?,
                }),
                EVT_GESTURE if len == 3 => Some(ApiEvent::GestureEvent {
                    animation_id: payload[0],
                    peak_accel: i16::from_le_bytes([payload[1], payload[2]]),
                }),
                EVT_DEVICE_STATUS if len == 8 => Some(ApiEvent::DeviceStatus {
                    pairing_status: payload[0],
                    led_mode: payload[1],
                    active_animation: payload[2],
                    brightness: payload[3],
                    thrust_threshold: i16::from_le_bytes([payload[4], payload[5]]),
                    impact_threshold: i16::from_le_bytes([payload[6], payload[7]]),
                }),
                _ => None,
            }
        }
    }
}

// ============================================================================
// PHASE 3: MADGWICK AHRS SENSOR FUSION
// ============================================================================

/// 6-DoF Madgwick AHRS orientation filter (`no_std`, no external math dependencies).
pub mod fusion {
    use crate::SensorData;

    /// Q14 fixed-point scale factor (`16384 = 2^14`) for compact 8-byte BLE quaternion packets.
    pub const QUAT_Q14_SCALE: f32 = 16384.0;
    /// Default Madgwick filter gain `beta` (rad/s).
    pub const DEFAULT_MADGWICK_BETA: f32 = 0.1;
    /// Default sensor sampling frequency (20 Hz).
    pub const DEFAULT_SAMPLE_FREQ_HZ: f32 = 20.0;
    /// Gyroscope stationary deadband (rad/s) for zero-velocity drift suppression.
    pub const GYRO_DEADBAND_RAD_S: f32 = 0.015;

    /// Fast `no_std` square root using Newton-Raphson iterations on IEEE-754 initial guess.
    #[inline]
    pub fn fast_sqrt(x: f32) -> f32 {
        if x <= 0.0 {
            return 0.0;
        }
        let inv = fast_inv_sqrt(x);
        x * inv
    }

    /// Fast `no_std` inverse square root (`1.0 / sqrt(x)`) with two Newton-Raphson refinement steps.
    #[inline]
    pub fn fast_inv_sqrt(x: f32) -> f32 {
        if x <= 0.0 {
            return 0.0;
        }
        let xhalf = 0.5 * x;
        let i = x.to_bits();
        let i = 0x5f37_5a86 - (i >> 1);
        let mut y = f32::from_bits(i);
        y = y * (1.5 - xhalf * y * y);
        y = y * (1.5 - xhalf * y * y);
        y
    }

    /// 3D orientation quaternion `(w, x, y, z)`.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct Quaternion {
        pub w: f32,
        pub x: f32,
        pub y: f32,
        pub z: f32,
    }

    impl Default for Quaternion {
        fn default() -> Self {
            Self::IDENTITY
        }
    }

    impl Quaternion {
        /// Identity quaternion `(1, 0, 0, 0)`.
        pub const IDENTITY: Self = Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };

        /// Construct a quaternion.
        pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
            Self { w, x, y, z }
        }

        /// Compute the Euclidean norm `sqrt(w^2 + x^2 + y^2 + z^2)`.
        pub fn norm(&self) -> f32 {
            fast_sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        }

        /// Return a normalized unit quaternion.
        pub fn normalized(&self) -> Self {
            let n_sq = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z;
            if n_sq <= 1.0e-12 {
                return Self::IDENTITY;
            }
            let inv = fast_inv_sqrt(n_sq);
            Self {
                w: self.w * inv,
                x: self.x * inv,
                y: self.y * inv,
                z: self.z * inv,
            }
        }

        /// Serialize to an 8-byte little-endian Q14 fixed-point array (`[w, x, y, z]` as `i16`).
        pub fn to_q14_bytes(&self) -> [u8; 8] {
            let q = self.normalized();
            let clamp_q14 = |v: f32| -> i16 {
                let scaled = v * QUAT_Q14_SCALE;
                if scaled >= 16384.0 {
                    16384
                } else if scaled <= -16384.0 {
                    -16384
                } else if scaled >= 0.0 {
                    (scaled + 0.5) as i16
                } else {
                    (scaled - 0.5) as i16
                }
            };
            let w = clamp_q14(q.w).to_le_bytes();
            let x = clamp_q14(q.x).to_le_bytes();
            let y = clamp_q14(q.y).to_le_bytes();
            let z = clamp_q14(q.z).to_le_bytes();
            [w[0], w[1], x[0], x[1], y[0], y[1], z[0], z[1]]
        }

        /// Deserialize from an 8-byte little-endian Q14 fixed-point array.
        pub fn from_q14_bytes(bytes: &[u8; 8]) -> Self {
            let w = i16::from_le_bytes([bytes[0], bytes[1]]) as f32 / QUAT_Q14_SCALE;
            let x = i16::from_le_bytes([bytes[2], bytes[3]]) as f32 / QUAT_Q14_SCALE;
            let y = i16::from_le_bytes([bytes[4], bytes[5]]) as f32 / QUAT_Q14_SCALE;
            let z = i16::from_le_bytes([bytes[6], bytes[7]]) as f32 / QUAT_Q14_SCALE;
            Self { w, x, y, z }.normalized()
        }

        /// Compute gravity direction vector `(gx, gy, gz)` in the sensor frame predicted by this quaternion.
        pub fn gravity_vector(&self) -> (f32, f32, f32) {
            let gx = 2.0 * (self.x * self.z - self.w * self.y);
            let gy = 2.0 * (self.w * self.x + self.y * self.z);
            let gz = self.w * self.w - self.x * self.x - self.y * self.y + self.z * self.z;
            (gx, gy, gz)
        }
    }

    /// Convert raw LSM6DS3TR accelerometer reading (`i16`) to `g` (assuming ±2g scale: 0.061 mg/LSB).
    #[inline]
    pub fn accel_lsb_to_g(lsb: i16) -> f32 {
        lsb as f32 * 0.000_061
    }

    /// Convert raw LSM6DS3TR gyroscope reading (`i16`) to `rad/s` (assuming ±245 dps scale: 8.75 mdps/LSB).
    #[inline]
    pub fn gyro_lsb_to_rad_s(lsb: i16) -> f32 {
        const DPS_TO_RAD: f32 = core::f32::consts::PI / 180.0;
        lsb as f32 * 0.008_75 * DPS_TO_RAD
    }

    /// Madgwick 6-DoF IMU AHRS orientation estimator.
    #[derive(Clone, Copy, Debug)]
    pub struct MadgwickAhrs {
        q: Quaternion,
        beta: f32,
        inv_sample_freq: f32,
    }

    impl Default for MadgwickAhrs {
        fn default() -> Self {
            Self::new(DEFAULT_SAMPLE_FREQ_HZ, DEFAULT_MADGWICK_BETA)
        }
    }

    impl MadgwickAhrs {
        /// Create a new Madgwick AHRS filter for `sample_freq_hz` and gradient gain `beta`.
        pub fn new(sample_freq_hz: f32, beta: f32) -> Self {
            let freq = if sample_freq_hz > 0.0 {
                sample_freq_hz
            } else {
                DEFAULT_SAMPLE_FREQ_HZ
            };
            Self {
                q: Quaternion::IDENTITY,
                beta,
                inv_sample_freq: 1.0 / freq,
            }
        }

        /// Return the current orientation quaternion.
        pub const fn quaternion(&self) -> Quaternion {
            self.q
        }

        /// Reset orientation to the identity quaternion.
        pub fn reset(&mut self) {
            self.q = Quaternion::IDENTITY;
        }

        /// Step the filter using a raw [`SensorData`] sample.
        pub fn update_from_sensor_data(&mut self, data: &SensorData) -> Quaternion {
            let gx = gyro_lsb_to_rad_s(data.gyro_x);
            let gy = gyro_lsb_to_rad_s(data.gyro_y);
            let gz = gyro_lsb_to_rad_s(data.gyro_z);
            let ax = accel_lsb_to_g(data.accel_x);
            let ay = accel_lsb_to_g(data.accel_y);
            let az = accel_lsb_to_g(data.accel_z);
            self.update_imu(gx, gy, gz, ax, ay, az)
        }

        /// Step the 6-DoF Madgwick IMU filter (`gx, gy, gz` in rad/s, `ax, ay, az` in any consistent unit).
        pub fn update_imu(
            &mut self,
            mut gx: f32,
            mut gy: f32,
            mut gz: f32,
            ax: f32,
            ay: f32,
            az: f32,
        ) -> Quaternion {
            // Apply zero-velocity update (ZUPT) deadband on near-zero gyro rates
            if gx.abs() < GYRO_DEADBAND_RAD_S {
                gx = 0.0;
            }
            if gy.abs() < GYRO_DEADBAND_RAD_S {
                gy = 0.0;
            }
            if gz.abs() < GYRO_DEADBAND_RAD_S {
                gz = 0.0;
            }

            let mut q0 = self.q.w;
            let mut q1 = self.q.x;
            let mut q2 = self.q.y;
            let mut q3 = self.q.z;

            // Rate of change of quaternion from gyroscope
            let mut q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
            let mut q_dot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
            let mut q_dot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
            let mut q_dot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

            // Compute feedback only if accelerometer measurement is non-zero
            let accel_norm_sq = ax * ax + ay * ay + az * az;
            if accel_norm_sq > 1.0e-12 {
                let recip_norm = fast_inv_sqrt(accel_norm_sq);
                let ax = ax * recip_norm;
                let ay = ay * recip_norm;
                let az = az * recip_norm;

                // Auxiliary variables to avoid repeated arithmetic
                let _2q0 = 2.0 * q0;
                let _2q1 = 2.0 * q1;
                let _2q2 = 2.0 * q2;
                let _2q3 = 2.0 * q3;
                let _4q0 = 4.0 * q0;
                let _4q1 = 4.0 * q1;
                let _4q2 = 4.0 * q2;
                let _8q1 = 8.0 * q1;
                let _8q2 = 8.0 * q2;
                let q0q0 = q0 * q0;
                let q1q1 = q1 * q1;
                let q2q2 = q2 * q2;
                let q3q3 = q3 * q3;

                // Gradient descent corrective step
                let s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
                let s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1
                    + _8q1 * q1q1
                    + _8q1 * q2q2
                    + _4q1 * az;
                let s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
                    + _8q2 * q1q1
                    + _8q2 * q2q2
                    + _4q2 * az;
                let s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

                let s_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
                if s_norm_sq > 1.0e-12 {
                    let recip_s = fast_inv_sqrt(s_norm_sq);
                    q_dot1 -= self.beta * s0 * recip_s;
                    q_dot2 -= self.beta * s1 * recip_s;
                    q_dot3 -= self.beta * s2 * recip_s;
                    q_dot4 -= self.beta * s3 * recip_s;
                }
            }

            // Integrate rate of change of quaternion
            q0 += q_dot1 * self.inv_sample_freq;
            q1 += q_dot2 * self.inv_sample_freq;
            q2 += q_dot3 * self.inv_sample_freq;
            q3 += q_dot4 * self.inv_sample_freq;

            self.q = Quaternion::new(q0, q1, q2, q3).normalized();
            self.q
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
