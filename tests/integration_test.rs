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
    let data2 = data1.clone();
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
fn test_classify_motion_upward_thrust() {
    assert_eq!(
        classify_motion(0, 0, THRUST_THRESHOLD + 10),
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
fn test_validate_sensor_data_accel_out_of_range() {
    let data = SensorData::new(15000, 200, 300, 10, 20, 30);
    assert!(!validate_sensor_data(&data));
}

#[test]
fn test_validate_sensor_data_gyro_out_of_range() {
    let data = SensorData::new(100, 200, 300, 25000, 20, 30);
    assert!(!validate_sensor_data(&data));
}

#[test]
fn test_validate_sensor_data_zeros() {
    let data = SensorData::new(0, 0, 0, 0, 0, 0);
    assert!(validate_sensor_data(&data));
}

#[test]
fn test_validate_sensor_data_max_valid() {
    let data = SensorData::new(9999, 9999, 9999, 19999, 19999, 19999);
    assert!(validate_sensor_data(&data));
}

// ============================================================================
// Configuration Constants Tests
// ============================================================================

#[test]
fn test_configuration_values() {
    assert!(THRUST_THRESHOLD > 0);
    assert_eq!(NFC_PAIRING_TIMEOUT_SECS, 15);
    assert_eq!(SENSOR_SAMPLING_INTERVAL_MS, 50);
}
