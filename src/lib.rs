//! Pure logic module for He-Man Sword Sensor
//! 
//! This module contains testable functions that don't depend on embedded hardware.
//! All embedded-specific code remains in main.rs.

#![cfg_attr(not(test), no_std)]

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

pub const THRUST_THRESHOLD: i16 = 50;  // Threshold for upward thrust detection (~0.5g)
pub const NFC_PAIRING_TIMEOUT_SECS: u64 = 15;
pub const SENSOR_SAMPLING_INTERVAL_MS: u64 = 50;  // 20 Hz

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
    let magnitude_sq = (accel_x as i32).pow(2)
        + (accel_y as i32).pow(2)
        + (accel_z as i32).pow(2);

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
    let accel_valid = data.accel_x.abs() < 10000
        && data.accel_y.abs() < 10000
        && data.accel_z.abs() < 10000;

    // Gyroscope: typically ±2000 dps, raw value limit is ±32767
    let gyro_valid = data.gyro_x.abs() < 20000
        && data.gyro_y.abs() < 20000
        && data.gyro_z.abs() < 20000;

    accel_valid && gyro_valid
}
