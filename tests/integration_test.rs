use xiao_nrf52840_sword::*;

// ============================================================================
// SensorData Structure Tests
// ============================================================================

#[test]
fn test_sensor_data_size() {
    assert_eq!(core::mem::size_of::<SensorData>(), 12);
    assert_eq!(SensorData::size(), 12);
}

#[test]
fn test_sensor_data_alignment() {
    assert_eq!(core::mem::align_of::<SensorData>(), 2);
}

#[test]
fn test_sensor_data_creation() {
    let data = SensorData::new(100, -200, 300, 10, -20, 30);
    assert_eq!(data.accel_x, 100);
    assert_eq!(data.accel_y, -200);
    assert_eq!(data.accel_z, 300);
    assert_eq!(data.gyro_x, 10);
    assert_eq!(data.gyro_y, -20);
    assert_eq!(data.gyro_z, 30);
}

#[test]
fn test_sensor_data_clone() {
    let data1 = SensorData::new(100, 200, 300, 10, 20, 30);
    let data2 = data1;
    assert_eq!(data1, data2);
}

#[test]
fn test_sensor_data_equality() {
    let data1 = SensorData::new(100, 200, 300, 10, 20, 30);
    let data2 = SensorData::new(100, 200, 300, 10, 20, 30);
    let data3 = SensorData::new(100, 200, 300, 10, 20, 31);
    assert_eq!(data1, data2);
    assert_ne!(data1, data3);
}

// ============================================================================
// SensorData serialization Tests
// ============================================================================

#[test]
fn test_sensor_data_to_bytes_round_trip() {
    let data = SensorData::new(100, -200, 300, -10, 20, -30);
    let bytes = data.to_bytes();
    let restored = SensorData::from_bytes(&bytes);
    assert_eq!(data, restored);
}

#[test]
fn test_sensor_data_to_bytes_length() {
    let data = SensorData::new(1, 2, 3, 4, 5, 6);
    let bytes = data.to_bytes();
    assert_eq!(bytes.len(), 12);
}

#[test]
fn test_sensor_data_to_bytes_little_endian() {
    // accel_x = 0x0102 → bytes [0x02, 0x01] at positions [0..2]
    let data = SensorData::new(0x0102, 0, 0, 0, 0, 0);
    let bytes = data.to_bytes();
    assert_eq!(bytes[0], 0x02);
    assert_eq!(bytes[1], 0x01);
}

#[test]
fn sensor_wire_codec_matches_independent_full_frame_fixture() {
    let sample = SensorData::new(0x0102, -3, i16::MIN, 0x1234, -1, i16::MAX);
    assert_eq!(
        sample.to_bytes(),
        [0x02, 0x01, 0xFD, 0xFF, 0x00, 0x80, 0x34, 0x12, 0xFF, 0xFF, 0xFF, 0x7F]
    );
    assert_eq!(
        SensorData::from_bytes(&[
            0x02, 0x01, 0xFD, 0xFF, 0x00, 0x80, 0x34, 0x12, 0xFF, 0xFF, 0xFF, 0x7F
        ]),
        sample
    );
}

#[test]
fn test_sensor_data_zero_bytes() {
    let data = SensorData::new(0, 0, 0, 0, 0, 0);
    let bytes = data.to_bytes();
    assert_eq!(bytes, [0u8; 12]);
}

#[test]
fn test_sensor_data_max_values_round_trip() {
    let data = SensorData::new(i16::MAX, i16::MIN, i16::MAX, i16::MIN, i16::MAX, i16::MIN);
    assert_eq!(data, SensorData::from_bytes(&data.to_bytes()));
}

// ============================================================================
// Motion Detection Tests
// ============================================================================

#[test]
fn test_detect_upward_thrust_positive() {
    assert!(detect_upward_thrust(THRUST_THRESHOLD + 1));
    assert!(detect_upward_thrust(100));
    assert!(detect_upward_thrust(i16::MAX));
}

#[test]
fn test_detect_upward_thrust_negative() {
    assert!(!detect_upward_thrust(THRUST_THRESHOLD - 1));
    assert!(!detect_upward_thrust(0));
    assert!(!detect_upward_thrust(-100));
    assert!(!detect_upward_thrust(i16::MIN));
}

#[test]
fn test_detect_upward_thrust_boundary() {
    assert!(!detect_upward_thrust(THRUST_THRESHOLD));
    assert!(detect_upward_thrust(THRUST_THRESHOLD + 1));
}

#[test]
fn test_classify_motion_idle() {
    assert_eq!(classify_motion(1, 1, 1), MotionType::Idle);
    assert_eq!(classify_motion(10, 10, 10), MotionType::Idle);
}

#[test]
fn test_classify_motion_moderate() {
    assert_eq!(classify_motion(30, 0, 0), MotionType::Moderate);
}

#[test]
fn test_classify_motion_intense() {
    assert_eq!(classify_motion(100, 0, 0), MotionType::Intense);
}

#[test]
fn classify_motion_uses_wide_intermediates_for_full_i16_domain() {
    assert_eq!(classify_motion(i16::MIN, i16::MIN, 0), MotionType::Intense);
    assert_eq!(classify_motion(i16::MAX, i16::MAX, 0), MotionType::Intense);
}

#[test]
fn test_classify_motion_upward_thrust() {
    assert_eq!(
        classify_motion(0, 0, THRUST_THRESHOLD + 10),
        MotionType::UpwardThrust
    );
}

// Upward thrust takes priority even when XY magnitude would be Intense
#[test]
fn test_classify_motion_upward_thrust_priority_over_intense() {
    assert_eq!(
        classify_motion(100, 100, THRUST_THRESHOLD + 1),
        MotionType::UpwardThrust
    );
}

// ============================================================================
// Validation Tests
// ============================================================================

#[test]
fn test_validate_sensor_data_valid() {
    let data = SensorData::new(100, -200, 300, 10, -20, 30);
    assert!(validate_sensor_data(&data));
}

#[test]
fn validator_accepts_one_g_raw_sample_at_documented_scale() {
    let approximately_one_g = SensorData::new(0, 0, 16_393, 0, 0, 0);
    assert!(validate_sensor_data(&approximately_one_g));
}

#[test]
fn validator_accepts_full_hardware_i16_domain_without_abs_overflow() {
    let extremes = SensorData::new(i16::MIN, i16::MAX, i16::MIN, i16::MAX, i16::MIN, i16::MAX);
    assert!(validate_sensor_data(&extremes));
}

#[test]
fn test_validate_sensor_data_zeros() {
    let data = SensorData::new(0, 0, 0, 0, 0, 0);
    assert!(validate_sensor_data(&data));
}

#[test]
fn test_validate_sensor_data_max_valid() {
    let data = SensorData::new(i16::MAX, i16::MAX, i16::MAX, i16::MAX, i16::MAX, i16::MAX);
    assert!(validate_sensor_data(&data));
}

#[test]
fn raw_threshold_defaults_remain_uncalibrated() {
    assert_eq!(THRUST_THRESHOLD, 50);
    assert_eq!(SLASH_THRESHOLD, -50);
    assert_eq!(IMPACT_THRESHOLD, 300);
    assert_eq!(
        classify_motion(16_393, 0, 0),
        MotionType::Intense,
        "classifier uses magnitude but preserves raw-count defaults"
    );
    assert_eq!(
        classify_motion(0, 0, 16_393),
        MotionType::UpwardThrust,
        "the preserved positive Z threshold has priority"
    );
    assert_eq!(
        detect_sword_gesture(&SensorData::new(0, 0, 16_393, 0, 0, 0)),
        Some(led::LedAnimation::ImpactHit),
        "gesture detection does not remove a stationary gravity component"
    );
}

// ============================================================================
// Configuration Constants Tests
// ============================================================================

#[test]
fn test_configuration_values() {
    const { assert!(THRUST_THRESHOLD > 0) };
    assert_eq!(NFC_PAIRING_TIMEOUT_SECS, 15);
    assert_eq!(SENSOR_SAMPLING_INTERVAL_MS, 50);
}

// ============================================================================
// BondedDevice serialization tests (no feature gate required)
// ============================================================================

#[test]
fn test_bonded_device_struct_size() {
    // repr(C) in-memory size varies by platform (padding depends on alignment).
    // The important invariant is the *serialised* wire size used for flash storage.
    assert_eq!(BONDED_DEVICE_STRUCT_SIZE, 11); // serialised: 6 MAC + 4 timestamp + 1 flags
                                               // In-memory size must be at least the serialised size
    assert!(core::mem::size_of::<BondedDevice>() >= BONDED_DEVICE_STRUCT_SIZE);
}

#[test]
fn test_bonded_device_to_bytes_from_bytes_round_trip() {
    let dev = BondedDevice {
        mac: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
        timestamp: 0xDEAD_BEEF,
        flags: 0b111,
    };
    let bytes = dev.to_bytes();
    let restored = BondedDevice::from_bytes(&bytes);
    assert_eq!(dev, restored);
}

#[test]
fn test_bonded_device_to_bytes_mac_first() {
    let dev = BondedDevice::new([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
    let bytes = dev.to_bytes();
    assert_eq!(&bytes[0..6], &[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
}

#[test]
fn test_bonded_device_to_bytes_timestamp_le() {
    let mut dev = BondedDevice::new([0; 6]);
    dev.set_timestamp(0x0102_0304);
    let bytes = dev.to_bytes();
    // Timestamp is at bytes[6..10] in little-endian
    assert_eq!(&bytes[6..10], &[0x04, 0x03, 0x02, 0x01]);
}

#[test]
fn test_bonded_device_to_bytes_flags_last() {
    let dev = BondedDevice {
        mac: [0; 6],
        timestamp: 0,
        flags: 0b101,
    };
    let bytes = dev.to_bytes();
    assert_eq!(bytes[10], 0b101);
}

#[test]
fn test_bonded_device_set_active_clear() {
    let mut dev = BondedDevice::new([0; 6]);
    assert!(dev.is_active());
    dev.set_active(false);
    assert!(!dev.is_active());
    dev.set_active(true);
    assert!(dev.is_active());
}

#[test]
fn test_bonded_device_set_paired_clear() {
    let mut dev = BondedDevice::new([0; 6]);
    dev.set_paired(true);
    assert!(dev.is_paired());
    dev.set_paired(false);
    assert!(!dev.is_paired());
}

#[test]
fn test_bonded_device_set_verified_clear() {
    let mut dev = BondedDevice::new([0; 6]);
    dev.set_verified(true);
    assert!(dev.is_verified());
    dev.set_verified(false);
    assert!(!dev.is_verified());
}
