use xiao_nrf52840_sword::*;

// Note: NFC-related types (BondedDevice, NfcPairingStatus) are only available
// with the embedded feature. Since integration tests run without the embedded
// feature to enable standard library support, we cannot test those types here.
// The NFC functionality is tested in the embedded firmware through hardware testing.

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

// ============================================================================
// NFC Types and Pairing Tests
// These tests require the `nfc` feature: `cargo test --features nfc`
// They run on the host target with no embedded hardware needed.
// ============================================================================

#[cfg(feature = "nfc")]
mod nfc_tests {
    use xiao_nrf52840_sword::{
        authenticate_bonded_device, get_nfc_pairing_status,
        nfc::NfcFieldState,
        pairing::{
            authenticate_bonded_device as pairing_auth, get_bonded_devices,
            get_nfc_pairing_status as pairing_status, is_bonded_device, read_bonded_device,
            read_bonded_device_mac, register_bonded_device, unregister_bonded_device,
            write_bonded_device_to_flash, write_bonded_device_to_flash_full,
            BONDED_DEVICE_STRUCT_SIZE, FLASH_BONDED_DEVICE_SIZE, FLASH_BONDED_DEVICE_START,
        },
        BondedDevice, NfcPairingStatus,
    };

    // -----------------------------------------------------------------------
    // NfcPairingStatus
    // -----------------------------------------------------------------------

    #[test]
    fn test_nfc_pairing_status_variants_are_distinct() {
        assert_ne!(NfcPairingStatus::Idle, NfcPairingStatus::Scanning);
        assert_ne!(NfcPairingStatus::Scanning, NfcPairingStatus::Authenticating);
        assert_ne!(NfcPairingStatus::Authenticating, NfcPairingStatus::Success);
        assert_ne!(NfcPairingStatus::Success, NfcPairingStatus::Failed);
        assert_ne!(NfcPairingStatus::Failed, NfcPairingStatus::Timeout);
    }

    #[test]
    fn test_get_nfc_pairing_status_top_level_returns_idle() {
        assert_eq!(get_nfc_pairing_status(), NfcPairingStatus::Idle);
    }

    #[test]
    fn test_pairing_module_get_status_returns_idle() {
        assert_eq!(pairing_status(), NfcPairingStatus::Idle);
    }

    // -----------------------------------------------------------------------
    // NfcFieldState
    // -----------------------------------------------------------------------

    #[test]
    fn test_nfc_field_state_get_field_state_returns_idle() {
        assert_eq!(
            xiao_nrf52840_sword::nfc::get_field_state(),
            NfcFieldState::Idle
        );
    }

    #[test]
    fn test_nfc_field_state_variants_are_distinct() {
        assert_ne!(NfcFieldState::Idle, NfcFieldState::Scanning);
        assert_ne!(NfcFieldState::Scanning, NfcFieldState::Detected);
        assert_ne!(NfcFieldState::Detected, NfcFieldState::Error);
    }

    // -----------------------------------------------------------------------
    // BondedDevice construction and flag logic
    // -----------------------------------------------------------------------

    #[test]
    fn test_bonded_device_new_sets_active_flag_only() {
        let mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let dev = BondedDevice::new(mac);

        assert_eq!(dev.mac, mac);
        assert_eq!(dev.timestamp, 0);
        assert!(dev.is_active(), "new device should be active");
        assert!(!dev.is_paired(), "new device should not be paired yet");
        assert!(!dev.is_verified(), "new device should not be verified yet");
    }

    #[test]
    fn test_bonded_device_set_paired_flag() {
        let mut dev = BondedDevice::new([0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        assert!(!dev.is_paired());
        dev.set_paired(true);
        assert!(dev.is_paired());
    }

    #[test]
    fn test_bonded_device_set_verified_flag() {
        let mut dev = BondedDevice::new([0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        assert!(!dev.is_verified());
        dev.set_verified(true);
        assert!(dev.is_verified());
    }

    #[test]
    fn test_bonded_device_set_timestamp() {
        let mut dev = BondedDevice::new([0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        assert_eq!(dev.timestamp(), 0);
        dev.set_timestamp(1_700_000_000);
        assert_eq!(dev.timestamp(), 1_700_000_000);
    }

    #[test]
    fn test_bonded_device_all_flags_set() {
        let dev = BondedDevice {
            mac: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
            timestamp: 42,
            flags: 0b111,
        };
        assert!(dev.is_active());
        assert!(dev.is_paired());
        assert!(dev.is_verified());
    }

    #[test]
    fn test_bonded_device_no_flags_set() {
        let dev = BondedDevice {
            mac: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
            timestamp: 0,
            flags: 0b000,
        };
        assert!(!dev.is_active());
        assert!(!dev.is_paired());
        assert!(!dev.is_verified());
    }

    #[test]
    fn test_bonded_device_equality() {
        let a = BondedDevice {
            mac: [1, 2, 3, 4, 5, 6],
            timestamp: 100,
            flags: 0b111,
        };
        let b = BondedDevice {
            mac: [1, 2, 3, 4, 5, 6],
            timestamp: 100,
            flags: 0b111,
        };
        let c = BondedDevice {
            mac: [9, 2, 3, 4, 5, 6],
            timestamp: 100,
            flags: 0b111,
        };
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    // -----------------------------------------------------------------------
    // authenticate_bonded_device — top-level and pairing module
    // -----------------------------------------------------------------------

    #[test]
    fn test_authenticate_bonded_device_matching_mac_succeeds() {
        // The stub stores [0x00, 11, 22, 33, 44, 55] as the bonded MAC.
        let bonded_mac = [0x00u8, 11, 22, 33, 44, 55];
        assert!(authenticate_bonded_device(&bonded_mac));
    }

    #[test]
    fn test_authenticate_bonded_device_wrong_mac_fails() {
        let wrong_mac = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        assert!(!authenticate_bonded_device(&wrong_mac));
    }

    #[test]
    fn test_pairing_module_authenticate_matching_mac_succeeds() {
        let bonded_mac = [0x00u8, 11, 22, 33, 44, 55];
        assert!(pairing_auth(&bonded_mac));
    }

    #[test]
    fn test_pairing_module_authenticate_wrong_mac_fails() {
        let wrong_mac = [0x00, 0x00, 0x00, 0x00, 0x00, 0x01];
        assert!(!pairing_auth(&wrong_mac));
    }

    // -----------------------------------------------------------------------
    // Flash storage helpers (stubs — verify they return expected values)
    // -----------------------------------------------------------------------

    #[test]
    fn test_flash_storage_constants() {
        assert_eq!(FLASH_BONDED_DEVICE_START, 0x2000);
        assert_eq!(FLASH_BONDED_DEVICE_SIZE, 8 * 1024);
        assert_eq!(BONDED_DEVICE_STRUCT_SIZE, 11); // 6 MAC + 4 timestamp + 1 flags
    }

    #[test]
    fn test_read_bonded_device_mac_returns_some() {
        assert!(read_bonded_device_mac().is_some());
    }

    #[test]
    fn test_read_bonded_device_mac_matches_expected_stub() {
        let mac = read_bonded_device_mac().unwrap();
        assert_eq!(mac, [0x00, 11, 22, 33, 44, 55]);
    }

    #[test]
    fn test_read_bonded_device_returns_active_paired_verified() {
        let dev = read_bonded_device().unwrap();
        assert!(dev.is_active());
        assert!(dev.is_paired());
        assert!(dev.is_verified());
    }

    #[test]
    fn test_write_bonded_device_to_flash_returns_true() {
        let mac = [0x01u8, 0x02, 0x03, 0x04, 0x05, 0x06];
        assert!(write_bonded_device_to_flash(&mac));
    }

    #[test]
    fn test_write_bonded_device_full_returns_true() {
        let dev = BondedDevice {
            mac: [1, 2, 3, 4, 5, 6],
            timestamp: 0,
            flags: 0b001,
        };
        assert!(write_bonded_device_to_flash_full(&dev));
    }

    #[test]
    fn test_register_bonded_device_returns_true() {
        let mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        assert!(register_bonded_device(&mac));
    }

    #[test]
    fn test_unregister_bonded_device_returns_true() {
        let mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        assert!(unregister_bonded_device(&mac));
    }

    // -----------------------------------------------------------------------
    // get_bonded_devices / is_bonded_device
    // -----------------------------------------------------------------------

    #[test]
    fn test_get_bonded_devices_returns_two_entries() {
        let devices = get_bonded_devices();
        assert_eq!(devices.len(), 2);
    }

    #[test]
    fn test_get_bonded_devices_all_active_paired_verified() {
        for dev in &get_bonded_devices() {
            assert!(dev.is_active());
            assert!(dev.is_paired());
            assert!(dev.is_verified());
        }
    }

    #[test]
    fn test_is_bonded_device_known_mac_returns_true() {
        let known = [0x00u8, 11, 22, 33, 44, 55];
        assert!(is_bonded_device(&known));
    }

    #[test]
    fn test_is_bonded_device_second_known_mac_returns_true() {
        let known = [0x00u8, 12, 23, 34, 45, 56];
        assert!(is_bonded_device(&known));
    }

    #[test]
    fn test_is_bonded_device_unknown_mac_returns_false() {
        let unknown = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        assert!(!is_bonded_device(&unknown));
    }
}
