//! Raw-count motion classification and gesture detection.

use crate::{led, SensorData, IMPACT_THRESHOLD, THRUST_THRESHOLD};

// ============================================================================

/// Detect upward thrust based on Z-axis acceleration.
/// Returns `true` if acceleration exceeds [`THRUST_THRESHOLD`].
pub fn detect_upward_thrust(accel_z: i16) -> bool {
    accel_z > THRUST_THRESHOLD
}

/// Classify motion based on acceleration magnitude in uncalibrated raw counts.
///
/// The fixed historical boundaries do not remove gravity; a stationary
/// approximately 1 g sample can therefore classify as intense.
/// Upward thrust has priority; otherwise squared magnitude `>= 10_000` is
/// intense, `>= 900` is moderate, and smaller values are idle.
pub fn classify_motion(accel_x: i16, accel_y: i16, accel_z: i16) -> MotionType {
    let magnitude_sq = (accel_x as i64).pow(2) + (accel_y as i64).pow(2) + (accel_z as i64).pow(2);

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
    /// Below the moderate raw-count magnitude boundary.
    Idle,
    /// Magnitude squared is at least 900 and below the intense boundary.
    Moderate,
    /// Magnitude squared is at least 10,000.
    Intense,
    /// Z acceleration strictly exceeds the configured thrust threshold.
    UpwardThrust,
}

/// Detect a sword combat gesture from 6-axis sensor data and map it to
/// the corresponding [`led::LedAnimation`].
///
/// Thresholds are uncalibrated raw counts. Gravity compensation is not
/// performed, so a stationary 1 g sample can be classified as an impact.
///
/// Priority order:
/// 1. [`led::LedAnimation::ImpactHit`] — any absolute axis meets `IMPACT_THRESHOLD`
/// 2. [`led::LedAnimation::UpwardThrust`] — `accel_z > THRUST_THRESHOLD`
/// 3. [`led::LedAnimation::DownwardSlash`] — `accel_z < SLASH_THRESHOLD` with pitch/roll rotation
pub fn detect_sword_gesture(data: &SensorData) -> Option<led::LedAnimation> {
    detect_sword_gesture_with_thresholds(data, THRUST_THRESHOLD, IMPACT_THRESHOLD)
}

/// Detect a sword combat gesture using configurable raw-count thresholds.
///
/// The impact comparison is inclusive and precedes thrust/slash checks.
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
