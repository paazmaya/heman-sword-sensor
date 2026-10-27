//! Hardware-independent library for the He-Man Sword Sensor.
//!
//! Sensor packets, gesture rules, LED animations, BLE TLV messages, fusion,
//! NFC UID helpers, and bonded-device storage codecs live in focused modules.
//! The embedded binary can use these APIs without pulling hardware policy into
//! host-side tests.
//!
//! # Features
//! - `nfc`: enable host-testable NFC and pairing logic.
//! - `embedded`: enable embassy-dependent NFC/pairing helpers and `defmt`.

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs, rustdoc::broken_intra_doc_links)]

/// BLE command and event TLV protocol.
pub mod api;
/// Command application state and deterministic control outcomes.
pub mod control;
/// Six-axis Madgwick orientation filter.
pub mod fusion;
/// WS2812 LED animation engine and PWM encoder.
pub mod led;
/// Raw-count gesture classification.
pub mod motion;
/// NFC UID and tag-shadow helpers (host-testable with `nfc`).
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub mod nfc;
/// NFC pairing and flash operations.
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub mod pairing;
/// Raw sensor packet type and validation.
pub mod sensor;
/// Bonded-device codecs and backend-driven persistence operations.
pub mod storage;

pub use motion::{
    classify_motion, detect_sword_gesture, detect_sword_gesture_with_thresholds,
    detect_upward_thrust, MotionType,
};
pub use sensor::{validate_sensor_data, SensorData};
pub use storage::{BondedDevice, BONDED_DEVICE_STRUCT_SIZE};

/// Default raw-count threshold for upward thrust detection.
///
/// This historic setting is intentionally preserved; it is uncalibrated and
/// depends on the accelerometer range configured by the hardware adapter.
pub const THRUST_THRESHOLD: i16 = 50;
/// Default raw-count threshold for downward slash detection (negative Z).
///
/// This historic setting is intentionally preserved and uncalibrated.
pub const SLASH_THRESHOLD: i16 = -50;
/// Default raw-count threshold for impact detection.
///
/// This historic setting is intentionally preserved and uncalibrated.
pub const IMPACT_THRESHOLD: i16 = 300;
/// NFC pairing timeout used by the embedded pairing flow, in seconds.
pub const NFC_PAIRING_TIMEOUT_SECS: u64 = 15;
/// Nominal sensor sampling interval, in milliseconds (20 Hz).
pub const SENSOR_SAMPLING_INTERVAL_MS: u64 = 50;

/// Lifecycle states reported by the NFC pairing gate.
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NfcPairingStatus {
    /// Pairing gate has not started.
    Idle,
    /// Waiting for an external reader to select this device's tag.
    Scanning,
    /// The local tag was selected and stored-record rules are being evaluated.
    Authenticating,
    /// NFC interaction completed; this is not cryptographic BLE authorization.
    Success,
    /// Pairing evaluation failed.
    Failed,
    /// No reader selected the tag before timeout; caller may use open advertising.
    Timeout,
}

impl NfcPairingStatus {
    /// Return the stable wire representation (`0` through `5`).
    pub const fn to_u8(self) -> u8 {
        match self {
            Self::Idle => 0,
            Self::Scanning => 1,
            Self::Authenticating => 2,
            Self::Success => 3,
            Self::Failed => 4,
            Self::Timeout => 5,
        }
    }

    /// Parse the stable wire representation, returning `None` for unknown values.
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Idle),
            1 => Some(Self::Scanning),
            2 => Some(Self::Authenticating),
            3 => Some(Self::Success),
            4 => Some(Self::Failed),
            5 => Some(Self::Timeout),
            _ => None,
        }
    }
}

/// Independently owned NFC pairing status for stateful callers and tests.
pub struct NfcPairingStatusCell(core::sync::atomic::AtomicU8);

impl NfcPairingStatusCell {
    /// Create a status cell initialized to [`NfcPairingStatus::Idle`].
    pub const fn new() -> Self {
        Self(core::sync::atomic::AtomicU8::new(0))
    }

    /// Store the current status.
    pub fn set(&self, status: NfcPairingStatus) {
        self.0
            .store(status.to_u8(), core::sync::atomic::Ordering::SeqCst);
    }

    /// Read the current status.
    pub fn get(&self) -> NfcPairingStatus {
        NfcPairingStatus::from_u8(self.0.load(core::sync::atomic::Ordering::SeqCst))
            .unwrap_or(NfcPairingStatus::Idle)
    }
}

impl Default for NfcPairingStatusCell {
    fn default() -> Self {
        Self::new()
    }
}

static NFC_PAIRING_STATUS_STATE: NfcPairingStatusCell = NfcPairingStatusCell::new();

/// Update the process-wide pairing status for compatibility with firmware callers.
pub fn set_nfc_pairing_status(status: NfcPairingStatus) {
    NFC_PAIRING_STATUS_STATE.set(status);
}

/// Read the process-wide NFC pairing status.
pub fn nfc_pairing_status() -> NfcPairingStatus {
    NFC_PAIRING_STATUS_STATE.get()
}

/// Compare a MAC against the current bonded-device record.
///
/// See [`pairing::authenticate_bonded_device`]. Without the embedded feature
/// there is no implicit sample record, so this returns `false`. A matching
/// address is not cryptographic proof of BLE central identity.
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub fn authenticate_bonded_device(mac: &[u8; 6]) -> bool {
    pairing::authenticate_bonded_device(mac)
}

/// Read the process-wide NFC pairing status.
#[cfg(any(feature = "nfc", feature = "embedded"))]
pub fn get_nfc_pairing_status() -> NfcPairingStatus {
    pairing::get_nfc_pairing_status()
}
