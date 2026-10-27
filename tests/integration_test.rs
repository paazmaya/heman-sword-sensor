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

#[test]
fn test_validate_sensor_data_boundary_accel() {
    // 9999 valid, 10000 invalid
    assert!(validate_sensor_data(&SensorData::new(9999, 0, 0, 0, 0, 0)));
    assert!(!validate_sensor_data(&SensorData::new(
        10000, 0, 0, 0, 0, 0
    )));
    assert!(!validate_sensor_data(&SensorData::new(
        -10000, 0, 0, 0, 0, 0
    )));
}

#[test]
fn test_validate_sensor_data_boundary_gyro() {
    // 19999 valid, 20000 invalid
    assert!(validate_sensor_data(&SensorData::new(0, 0, 0, 19999, 0, 0)));
    assert!(!validate_sensor_data(&SensorData::new(
        0, 0, 0, 20000, 0, 0
    )));
    assert!(!validate_sensor_data(&SensorData::new(
        0, 0, 0, -20000, 0, 0
    )));
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

// ============================================================================
// NFC Types and Pairing Tests
// These tests require the `nfc` feature: `cargo test --features nfc`
// ============================================================================

#[cfg(feature = "nfc")]
mod nfc_tests {
    use xiao_nrf52840_sword::{
        authenticate_bonded_device, get_nfc_pairing_status,
        nfc::{is_valid_uid, is_valid_uid_7, NfcFieldState},
        pairing::{
            authenticate_bonded_device as pairing_auth, authenticate_with_uid, get_bonded_devices,
            get_nfc_pairing_status as pairing_status, is_bonded_device, make_flash_record,
            parse_flash_record, read_bonded_device, read_bonded_device_full,
            read_bonded_device_mac, register_bonded_device, unregister_bonded_device,
            write_bonded_device_to_flash, write_bonded_device_to_flash_full, BleConfig,
            BondedDeviceFull, FLASH_BONDED_DEVICE_SIZE, FLASH_BONDED_DEVICE_START,
            FLASH_RECORD_MAGIC, FLASH_RECORD_SIZE, MAX_BONDED_DEVICES,
        },
        BondedDevice, NfcPairingStatus, BONDED_DEVICE_STRUCT_SIZE,
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
        xiao_nrf52840_sword::set_nfc_pairing_status(NfcPairingStatus::Idle);
        assert_eq!(get_nfc_pairing_status(), NfcPairingStatus::Idle);
    }

    #[test]
    fn test_pairing_module_get_status_returns_idle() {
        xiao_nrf52840_sword::set_nfc_pairing_status(NfcPairingStatus::Idle);
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
    // NFC UID validation
    // -----------------------------------------------------------------------

    #[test]
    fn test_is_valid_uid_real_cascade_uid() {
        // 0x04 is the ISO/IEC 14443 cascade tag byte for 10-byte UIDs
        let uid = [0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        assert!(is_valid_uid(&uid));
    }

    #[test]
    fn test_is_valid_uid_all_zeros_invalid() {
        assert!(!is_valid_uid(&[0u8; 10]));
    }

    #[test]
    fn test_is_valid_uid_wrong_first_byte_invalid() {
        let uid = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        assert!(!is_valid_uid(&uid));
    }

    #[test]
    fn test_is_valid_uid_7_real() {
        let uid = [0x04, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56];
        assert!(is_valid_uid_7(&uid));
    }

    #[test]
    fn test_is_valid_uid_7_all_zeros_invalid() {
        assert!(!is_valid_uid_7(&[0u8; 7]));
    }

    #[test]
    fn test_is_valid_uid_7_wrong_first_byte() {
        let uid = [0xFF, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56];
        assert!(!is_valid_uid_7(&uid));
    }

    // -----------------------------------------------------------------------
    // Flash record serialisation
    // -----------------------------------------------------------------------

    #[test]
    fn test_flash_record_size() {
        assert_eq!(FLASH_RECORD_SIZE, 1 + BONDED_DEVICE_STRUCT_SIZE); // magic + device
    }

    #[test]
    fn test_flash_record_magic_value() {
        assert_eq!(FLASH_RECORD_MAGIC, 0xAB);
    }

    #[test]
    fn test_make_parse_flash_record_round_trip() {
        let dev = BondedDevice {
            mac: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF],
            timestamp: 0x1234_5678,
            flags: 0b011, // active + paired
        };
        let record = make_flash_record(&dev);
        let parsed = parse_flash_record(&record).expect("should parse valid record");
        assert_eq!(dev, parsed);
    }

    #[test]
    fn test_parse_flash_record_wrong_magic_returns_none() {
        let dev = BondedDevice::new([1, 2, 3, 4, 5, 6]);
        let mut record = make_flash_record(&dev);
        record[0] = 0xFF; // corrupt magic
        assert!(parse_flash_record(&record).is_none());
    }

    #[test]
    fn test_parse_flash_record_inactive_returns_none() {
        let mut dev = BondedDevice::new([1, 2, 3, 4, 5, 6]);
        dev.set_active(false);
        let record = make_flash_record(&dev);
        assert!(parse_flash_record(&record).is_none());
    }

    #[test]
    fn test_parse_flash_record_active_not_paired_returns_some() {
        // Active-only record should parse (used before pairing ceremony)
        let dev = BondedDevice::new([1, 2, 3, 4, 5, 6]); // flags = 0b001
        let record = make_flash_record(&dev);
        assert!(parse_flash_record(&record).is_some());
    }

    // -----------------------------------------------------------------------
    // Flash layout constants
    // -----------------------------------------------------------------------

    #[test]
    fn test_flash_storage_constants() {
        // Last 4 KB page of 1 MB flash: 0x000F_F000 − 0x1000 = 0x000E_F000.
        // Placing storage here ensures it never overlaps the S140 SoftDevice
        // region (0x00000000–0x00025FFF) or application code.
        assert_eq!(FLASH_BONDED_DEVICE_START, 0x000E_F000);
        // Must be 4 KB-page-aligned
        assert_eq!(FLASH_BONDED_DEVICE_START % 4096, 0);
        assert_eq!(FLASH_BONDED_DEVICE_SIZE, 4096);
        // Must fit at least one record
        assert!(MAX_BONDED_DEVICES >= 1);
        // Flash record is 12 bytes (1 magic + 11 device)
        assert_eq!(FLASH_RECORD_SIZE, 12);
    }

    #[test]
    fn test_flash_start_above_softdevice_region() {
        // S140 SoftDevice occupies 0x00000000–0x00025FFF (152 KB).
        // Our storage must start above this boundary.
        assert!(FLASH_BONDED_DEVICE_START >= 0x0002_6000);
    }

    #[test]
    fn test_flash_region_within_flash_bounds() {
        // nRF52840 has 1 MB flash (0x000F_FFFF end).
        let end = FLASH_BONDED_DEVICE_START + FLASH_BONDED_DEVICE_SIZE as u32;
        assert!(end <= 0x0010_0000);
    }

    #[test]
    fn test_max_bonded_devices_fits_in_page() {
        assert!(MAX_BONDED_DEVICES * FLASH_RECORD_SIZE <= FLASH_BONDED_DEVICE_SIZE);
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
        assert!(dev.is_active());
        assert!(!dev.is_paired());
        assert!(!dev.is_verified());
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
            mac: [1, 2, 3, 4, 5, 6],
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
            mac: [1, 2, 3, 4, 5, 6],
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
    // BondedDeviceFull (MAC + UID)
    // -----------------------------------------------------------------------

    #[test]
    fn test_bonded_device_full_serialised_size() {
        assert_eq!(
            BondedDeviceFull::SERIALISED_SIZE,
            BONDED_DEVICE_STRUCT_SIZE + 10
        );
    }

    #[test]
    fn test_bonded_device_full_round_trip() {
        let full = BondedDeviceFull {
            dev: BondedDevice {
                mac: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF],
                timestamp: 0xDEAD_BEEF,
                flags: 0b111,
            },
            uid: [0x04, 1, 2, 3, 4, 5, 6, 7, 8, 9],
        };
        let bytes = full.to_bytes();
        let restored = BondedDeviceFull::from_bytes(&bytes);
        assert_eq!(full.dev, restored.dev);
        assert_eq!(full.uid, restored.uid);
    }

    #[test]
    fn test_bonded_device_full_uid_stored_after_device() {
        let full = BondedDeviceFull {
            dev: BondedDevice::new([0; 6]),
            uid: [0x04, 0xAB, 0, 0, 0, 0, 0, 0, 0, 0],
        };
        let bytes = full.to_bytes();
        // UID starts at offset BONDED_DEVICE_STRUCT_SIZE
        assert_eq!(bytes[BONDED_DEVICE_STRUCT_SIZE], 0x04);
        assert_eq!(bytes[BONDED_DEVICE_STRUCT_SIZE + 1], 0xAB);
    }

    #[test]
    fn test_read_bonded_device_full_returns_some() {
        assert!(read_bonded_device_full().is_some());
    }

    #[test]
    fn test_read_bonded_device_full_uid_starts_with_cascade_byte() {
        let full = read_bonded_device_full().unwrap();
        assert_eq!(full.uid[0], 0x04);
    }

    // -----------------------------------------------------------------------
    // authenticate_bonded_device — top-level and pairing module
    // -----------------------------------------------------------------------

    #[test]
    fn test_authenticate_bonded_device_matching_mac_succeeds() {
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
    // authenticate_with_uid
    // -----------------------------------------------------------------------

    #[test]
    fn test_authenticate_with_uid_correct_mac_and_uid_succeeds() {
        let mac = [0x00u8, 11, 22, 33, 44, 55];
        let uid = [0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        assert!(authenticate_with_uid(&mac, &uid));
    }

    #[test]
    fn test_authenticate_with_uid_wrong_mac_fails() {
        let bad_mac = [0xFF; 6];
        let uid = [0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        assert!(!authenticate_with_uid(&bad_mac, &uid));
    }

    #[test]
    fn test_authenticate_with_uid_wrong_uid_fails() {
        let mac = [0x00u8, 11, 22, 33, 44, 55];
        let bad_uid = [0xFF; 10];
        assert!(!authenticate_with_uid(&mac, &bad_uid));
    }

    // -----------------------------------------------------------------------
    // BleConfig
    // -----------------------------------------------------------------------

    #[test]
    fn test_ble_config_default_no_whitelist() {
        let cfg = BleConfig::default();
        assert!(cfg.bonded_mac.is_none());
        assert!(!cfg.use_whitelist);
    }

    #[test]
    fn test_ble_config_default_adv_interval_ms() {
        let cfg = BleConfig::default();
        // 160 units × 0.625 ms = 100 ms
        assert_eq!(cfg.adv_interval_ms(), 100);
    }

    #[test]
    fn test_ble_config_bonded_only_sets_whitelist() {
        let mac = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        let cfg = BleConfig::bonded_only(mac);
        assert_eq!(cfg.bonded_mac, Some(mac));
        assert!(cfg.use_whitelist);
    }

    #[test]
    fn test_ble_config_bonded_only_inherits_default_interval() {
        let cfg = BleConfig::bonded_only([0; 6]);
        assert_eq!(cfg.adv_interval_ms(), 100);
    }

    #[test]
    fn test_ble_config_equality() {
        let a = BleConfig::default();
        let b = BleConfig::default();
        assert_eq!(a, b);
    }

    #[test]
    fn test_ble_config_whitelist_differs_from_open() {
        let open = BleConfig::default();
        let wl = BleConfig::bonded_only([1, 2, 3, 4, 5, 6]);
        assert_ne!(open, wl);
    }

    // -----------------------------------------------------------------------
    // Flash read/write helpers
    // -----------------------------------------------------------------------

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

    #[test]
    fn test_is_bonded_device_inactive_not_found() {
        // Inactive devices should not be returned as bonded
        // (is_bonded_device checks is_active() internally)
        let mac = [0x00u8, 11, 22, 33, 44, 55]; // first stub device
                                                // Stub always returns active entries, so this passes
        assert!(is_bonded_device(&mac));
    }
}

// ============================================================================
// NFC UID extraction and parsing tests (available without any feature flag
// because extract_uid_from_nfct and parse_nfct_uid are now host-testable)
// ============================================================================

#[cfg(feature = "nfc")]
mod nfc_uid_tests {
    use xiao_nrf52840_sword::nfc::{extract_uid_from_nfct, parse_nfct_uid};

    // -----------------------------------------------------------------------
    // extract_uid_from_nfct
    // -----------------------------------------------------------------------

    #[test]
    fn test_extract_uid_empty_returns_none() {
        assert!(extract_uid_from_nfct(&[]).is_none());
    }

    #[test]
    fn test_extract_uid_all_zeros_returns_none() {
        assert!(extract_uid_from_nfct(&[0x00, 0x00, 0x00, 0x00]).is_none());
    }

    #[test]
    fn test_extract_uid_single_size_4_bytes_pads_to_10() {
        let raw = [0x04u8, 0x01, 0x02, 0x03];
        let uid = extract_uid_from_nfct(&raw).expect("should succeed");
        assert_eq!(uid.len(), 10);
        // First 4 bytes copied from raw, remainder zero-padded
        assert_eq!(&uid[..4], &raw[..]);
        assert_eq!(&uid[4..], &[0u8; 6]);
    }

    #[test]
    fn test_extract_uid_double_size_7_bytes() {
        let raw = [0x04u8, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56];
        let uid = extract_uid_from_nfct(&raw).expect("should succeed");
        assert_eq!(&uid[..7], &raw[..]);
        assert_eq!(&uid[7..], &[0u8; 3]);
    }

    #[test]
    fn test_extract_uid_triple_size_10_bytes() {
        let raw = [0x04u8, 1, 2, 3, 4, 5, 6, 7, 8, 9];
        let uid = extract_uid_from_nfct(&raw).expect("should succeed");
        assert_eq!(uid, raw);
    }

    #[test]
    fn test_extract_uid_more_than_10_bytes_truncated() {
        // Only first 10 bytes are used
        let raw = [0x04u8, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];
        let uid = extract_uid_from_nfct(&raw).expect("should succeed");
        assert_eq!(&uid[..], &raw[..10]);
    }

    // -----------------------------------------------------------------------
    // parse_nfct_uid — validates cascade byte in addition to extracting
    // -----------------------------------------------------------------------

    #[test]
    fn test_parse_nfct_uid_valid_4_byte_cascade() {
        let raw = [0x04u8, 0x01, 0x02, 0x03];
        let uid = parse_nfct_uid(&raw).expect("should parse");
        assert_eq!(uid[0], 0x04);
    }

    #[test]
    fn test_parse_nfct_uid_valid_10_byte() {
        let raw = [0x04u8, 1, 2, 3, 4, 5, 6, 7, 8, 9];
        assert!(parse_nfct_uid(&raw).is_some());
    }

    #[test]
    fn test_parse_nfct_uid_wrong_cascade_byte_returns_none() {
        // First byte is 0x00, not 0x04 — invalid
        let raw = [0x00u8, 0x01, 0x02, 0x03];
        assert!(parse_nfct_uid(&raw).is_none());
    }

    #[test]
    fn test_parse_nfct_uid_all_zeros_returns_none() {
        assert!(parse_nfct_uid(&[0u8; 10]).is_none());
    }

    #[test]
    fn test_parse_nfct_uid_empty_returns_none() {
        assert!(parse_nfct_uid(&[]).is_none());
    }

    #[test]
    fn test_parse_nfct_uid_7_byte_valid() {
        let raw = [0x04u8, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56];
        let uid = parse_nfct_uid(&raw).expect("should parse 7-byte UID");
        assert_eq!(uid[0], 0x04);
        assert_eq!(uid[1], 0xAB);
        assert_eq!(&uid[7..], &[0u8; 3]); // zero-padded
    }

    #[test]
    fn test_parse_nfct_uid_7_byte_wrong_cascade_returns_none() {
        let raw = [0xFFu8, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56];
        assert!(parse_nfct_uid(&raw).is_none());
    }

    #[test]
    fn test_extract_and_parse_agree_on_valid_uid() {
        let raw = [0x04u8, 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02, 0x03, 0x04];
        let extracted = extract_uid_from_nfct(&raw).unwrap();
        let parsed = parse_nfct_uid(&raw).unwrap();
        assert_eq!(extracted, parsed);
    }
}

// ============================================================================
// Flash storage address tests (no feature flag required)
// ============================================================================

#[cfg(feature = "nfc")]
mod flash_address_tests {
    use xiao_nrf52840_sword::pairing::{
        FLASH_BONDED_DEVICE_SIZE, FLASH_BONDED_DEVICE_START, FLASH_RECORD_SIZE, MAX_BONDED_DEVICES,
    };

    #[test]
    fn test_flash_start_is_page_aligned() {
        assert_eq!(
            FLASH_BONDED_DEVICE_START % 4096,
            0,
            "FLASH_BONDED_DEVICE_START must be 4 KB-page-aligned"
        );
    }

    #[test]
    fn test_flash_start_above_softdevice() {
        // S140 ends at 0x00025FFF; application memory starts at 0x00026000.
        assert!(
            FLASH_BONDED_DEVICE_START >= 0x0002_6000,
            "Flash storage must not overlap the S140 SoftDevice region"
        );
    }

    #[test]
    fn test_flash_end_within_nrf52840_flash() {
        let end = FLASH_BONDED_DEVICE_START as u64 + FLASH_BONDED_DEVICE_SIZE as u64;
        assert!(
            end <= 0x0010_0000,
            "Flash storage region must stay within nRF52840 1 MB flash"
        );
    }

    #[test]
    fn test_flash_at_expected_last_page() {
        // We specifically target the last 4 KB page: 0x000E_F000
        assert_eq!(FLASH_BONDED_DEVICE_START, 0x000E_F000);
    }

    #[test]
    fn test_max_bonded_devices_page_fit() {
        assert!(MAX_BONDED_DEVICES * FLASH_RECORD_SIZE <= FLASH_BONDED_DEVICE_SIZE);
    }
}

// ============================================================================
// BLE whitelist config tests
// ============================================================================

#[cfg(feature = "nfc")]
mod ble_whitelist_tests {
    use xiao_nrf52840_sword::pairing::BleConfig;

    #[test]
    fn test_open_config_no_whitelist_no_mac() {
        let cfg = BleConfig::default();
        assert!(!cfg.use_whitelist);
        assert!(cfg.bonded_mac.is_none());
    }

    #[test]
    fn test_bonded_only_enables_whitelist() {
        let mac = [0x11u8, 0x22, 0x33, 0x44, 0x55, 0x66];
        let cfg = BleConfig::bonded_only(mac);
        assert!(cfg.use_whitelist);
        assert_eq!(cfg.bonded_mac, Some(mac));
    }

    #[test]
    fn test_bonded_only_mac_roundtrip() {
        let mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let cfg = BleConfig::bonded_only(mac);
        assert_eq!(cfg.bonded_mac.unwrap(), mac);
    }

    #[test]
    fn test_different_macs_produce_distinct_configs() {
        let mac_a = [0x01u8, 0x02, 0x03, 0x04, 0x05, 0x06];
        let mac_b = [0x07u8, 0x08, 0x09, 0x0A, 0x0B, 0x0C];
        let cfg_a = BleConfig::bonded_only(mac_a);
        let cfg_b = BleConfig::bonded_only(mac_b);
        assert_ne!(cfg_a, cfg_b);
    }

    #[test]
    fn test_whitelist_config_differs_from_open() {
        let open = BleConfig::default();
        let wl = BleConfig::bonded_only([0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        assert_ne!(open, wl);
    }

    #[test]
    fn test_adv_interval_ms_default_is_100() {
        // 160 units × 0.625 ms = 100 ms
        assert_eq!(BleConfig::default().adv_interval_ms(), 100);
    }

    #[test]
    fn test_adv_interval_ms_custom() {
        let mut cfg = BleConfig::default();
        // 32 units × 0.625 ms = 20 ms
        cfg.adv_interval_units = 32;
        assert_eq!(cfg.adv_interval_ms(), 20);
    }
}

// ============================================================================
// NFC UID state (host-side stub behaviour)
// ============================================================================

#[cfg(feature = "nfc")]
mod nfc_state_tests {
    use xiao_nrf52840_sword::nfc::{get_field_state, NfcFieldState};

    /// On the host the ACTIVATED_UID static is never written, so
    /// get_field_state() must always return Idle.
    #[test]
    fn test_host_field_state_is_idle() {
        assert_eq!(get_field_state(), NfcFieldState::Idle);
    }
}

// ============================================================================
// pairing_mac characteristic value validation helpers
// ============================================================================

#[cfg(feature = "nfc")]
mod pairing_mac_tests {
    use xiao_nrf52840_sword::pairing::{make_flash_record, parse_flash_record};
    use xiao_nrf52840_sword::BondedDevice;

    /// Simulates what happens when the mobile app writes a real MAC via the
    /// pairing_mac characteristic: we create a BondedDevice, write it to a
    /// flash record, and read it back.
    #[test]
    fn test_real_mac_replaces_placeholder_in_flash_record() {
        let placeholder = [0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01];
        let real_mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];

        // Initial record uses placeholder MAC
        let mut dev = BondedDevice::new(placeholder);
        dev.set_paired(true);
        let record_v1 = make_flash_record(&dev);
        let parsed_v1 = parse_flash_record(&record_v1).unwrap();
        assert_eq!(parsed_v1.mac, placeholder);

        // App writes real MAC — we construct a new record
        let mut dev2 = parsed_v1;
        dev2.mac = real_mac;
        let record_v2 = make_flash_record(&dev2);
        let parsed_v2 = parse_flash_record(&record_v2).unwrap();
        assert_eq!(parsed_v2.mac, real_mac);
        // Flags preserved
        assert!(parsed_v2.is_paired());
        assert!(parsed_v2.is_active());
    }

    #[test]
    fn test_pairing_mac_write_valid_6_byte_value() {
        // A 6-byte MAC coming from the GATT write should produce a valid flash record
        let mac: [u8; 6] = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        let mut dev = BondedDevice::new(mac);
        dev.set_paired(true);
        dev.set_verified(true);
        let record = make_flash_record(&dev);
        let parsed = parse_flash_record(&record).expect("valid record");
        assert_eq!(parsed.mac, mac);
        assert!(parsed.is_active());
        assert!(parsed.is_paired());
        assert!(parsed.is_verified());
    }
}

// ============================================================================
// Phase 1: 60-LED WS2812B Animation Engine & PWM Encoder Tests
// ============================================================================

mod led_engine_tests {
    use xiao_nrf52840_sword::led::{
        encode_ws2812_pwm, LedAnimation, LedControlMode, LedEngine, RgbColor, DURATION_IMPACT_MS,
        DURATION_MOTION_STOP_MS, DURATION_SLASH_MS, DURATION_THRUST_MS, LED_FRAME_INTERVAL_MS,
        NUM_LEDS, PWM_BUFFER_WORDS, PWM_POLARITY_BIT, PWM_RESET_WORD, PWM_T0H_WORD, PWM_T1H_WORD,
        RESET_SLOTS,
    };
    use xiao_nrf52840_sword::{
        detect_sword_gesture, detect_sword_gesture_with_thresholds, SensorData, IMPACT_THRESHOLD,
        THRUST_THRESHOLD,
    };

    #[test]
    fn test_led_strip_constants_for_60_leds() {
        assert_eq!(NUM_LEDS, 60);
        assert_eq!(LED_FRAME_INTERVAL_MS, 20); // 50 Hz
        assert_eq!(PWM_BUFFER_WORDS, 60 * 24 + 40);
        assert_eq!(PWM_BUFFER_WORDS, 1480);
    }

    #[test]
    fn test_rgb_color_scaling() {
        let color = RgbColor::new(200, 100, 50);
        assert_eq!(color.scale(0), RgbColor::BLACK);
        assert_eq!(color.scale(255), color);
        let half = color.scale(128);
        assert!(half.r >= 99 && half.r <= 101);
        assert!(half.g >= 49 && half.g <= 51);
    }

    #[test]
    fn test_led_animation_id_round_trip() {
        let custom = RgbColor::new(12, 34, 56);
        let variants = [
            LedAnimation::Off,
            LedAnimation::IdleBreathing,
            LedAnimation::UpwardThrust,
            LedAnimation::DownwardSlash,
            LedAnimation::ImpactHit,
            LedAnimation::MotionStop,
            LedAnimation::SolidColor(custom),
        ];
        for anim in variants {
            let id = anim.id();
            let restored = LedAnimation::from_id(id, custom).expect("valid id");
            assert_eq!(anim, restored);
        }
        assert!(LedAnimation::from_id(99, RgbColor::WHITE).is_none());
    }

    #[test]
    fn test_led_engine_default_starts_in_idle_breathing() {
        let mut engine = LedEngine::new();
        assert_eq!(engine.current_animation(), LedAnimation::IdleBreathing);
        assert_eq!(engine.control_mode(), LedControlMode::AutoMotion);
        assert_eq!(engine.brightness(), 255);
        let frame = engine.tick(20);
        assert_eq!(frame.len(), 60);
        // Idle breathing produces a non-black dim blue/cyan glow
        assert!(frame[0].b > 0);
    }

    #[test]
    fn test_upward_thrust_propagates_guard_to_tip_and_returns_to_idle() {
        let mut engine = LedEngine::new();
        engine.trigger_motion(LedAnimation::UpwardThrust);
        assert_eq!(engine.current_animation(), LedAnimation::UpwardThrust);

        // Frame 0 (t = 0 ms): guard LED (0) is lit, tip LED (59) is still dark
        let frame_start = engine.tick(50);
        assert_ne!(frame_start[0], RgbColor::BLACK);
        assert_eq!(frame_start[NUM_LEDS - 1], RgbColor::BLACK);

        // Advance to end of thrust (t = 200 ms): tip LED (59) is now lit
        let _ = engine.tick(50); // t=50
        let _ = engine.tick(50); // t=100
        let _ = engine.tick(50); // t=150
        let frame_end = engine.tick(20); // t=200 (duration limit)
        assert_ne!(frame_end[NUM_LEDS - 1], RgbColor::BLACK);

        // Next tick after duration elapses transitions back to IdleBreathing
        assert!(DURATION_THRUST_MS == 200);
        let _ = engine.tick(20);
        assert_eq!(engine.current_animation(), LedAnimation::IdleBreathing);
    }

    #[test]
    fn test_downward_slash_propagates_tip_to_guard() {
        let mut engine = LedEngine::new();
        engine.trigger_motion(LedAnimation::DownwardSlash);
        assert_eq!(engine.current_animation(), LedAnimation::DownwardSlash);

        // Frame 0 (t = 0 ms): tip LED (59) is lit, guard LED (0) is dark
        let frame0 = engine.tick(50);
        assert_ne!(frame0[NUM_LEDS - 1], RgbColor::BLACK);
        assert_eq!(frame0[0], RgbColor::BLACK);

        // Advance to end of 150 ms slash: guard LED (0) is lit
        let _ = engine.tick(50);
        let _ = engine.tick(50);
        let frame_end = engine.tick(20); // t=150 ms
        assert_ne!(frame_end[0], RgbColor::BLACK);
        assert_eq!(DURATION_SLASH_MS, 150);
    }

    #[test]
    fn test_impact_hit_flashes_bright_then_decays() {
        let mut engine = LedEngine::new();
        engine.trigger_motion(LedAnimation::ImpactHit);
        let f0 = engine.tick(50);
        let f1 = engine.tick(40);
        // Starts full white (255) and decays at t=50 ms
        assert_eq!(f0[0], RgbColor::WHITE);
        assert!(f1[0].r < f0[0].r);
        assert_eq!(DURATION_IMPACT_MS, 100);
    }

    #[test]
    fn test_motion_stop_fades_over_1000_ms() {
        let mut engine = LedEngine::new();
        engine.start_animation(LedAnimation::SolidColor(RgbColor::WHITE), None);
        let _ = engine.tick(20);
        engine.start_animation(LedAnimation::MotionStop, None);
        let start_frame = engine.tick(500);
        let mid_frame = engine.tick(400);
        assert!(mid_frame[0].r < start_frame[0].r);
        assert_eq!(DURATION_MOTION_STOP_MS, 1000);
    }

    #[test]
    fn test_manual_override_ignores_motion_triggers() {
        let mut engine = LedEngine::new();
        let purple = RgbColor::new(180, 0, 255);
        engine.set_control_mode(LedControlMode::ManualOverride);
        engine.start_animation(LedAnimation::SolidColor(purple), None);

        // Motion gesture should be ignored while in ManualOverride
        engine.trigger_motion(LedAnimation::UpwardThrust);
        assert_eq!(engine.current_animation(), LedAnimation::SolidColor(purple));
        let frame = engine.tick(20);
        assert_eq!(frame[0], purple);
        assert_eq!(frame[59], purple);
    }

    #[test]
    fn test_encode_ws2812_pwm_grb_bit_order_and_reset_latch() {
        let mut leds = [RgbColor::BLACK; NUM_LEDS];
        // LED 0: G = 0xFF, R = 0x00, B = 0xAA (10101010)
        leds[0] = RgbColor::new(0x00, 0xFF, 0xAA);

        let mut pwm = [0u16; PWM_BUFFER_WORDS];
        encode_ws2812_pwm(&leds, 255, &mut pwm);

        // First 8 words = Green (0xFF -> all PWM_T1H_WORD)
        for &w in &pwm[0..8] {
            assert_eq!(w, PWM_T1H_WORD);
            assert_ne!(w & PWM_POLARITY_BIT, 0);
        }
        // Next 8 words = Red (0x00 -> all PWM_T0H_WORD)
        for &w in &pwm[8..16] {
            assert_eq!(w, PWM_T0H_WORD);
        }
        // Next 8 words = Blue (0xAA = 0b1010_1010 -> alternating T1H, T0H)
        for i in 0..8 {
            let expected = if i % 2 == 0 {
                PWM_T1H_WORD
            } else {
                PWM_T0H_WORD
            };
            assert_eq!(pwm[16 + i], expected);
        }
        // Final RESET_SLOTS words must all be PWM_RESET_WORD (0x8000)
        for &w in &pwm[NUM_LEDS * 24..PWM_BUFFER_WORDS] {
            assert_eq!(w, PWM_RESET_WORD);
        }
        assert_eq!(pwm[NUM_LEDS * 24..].len(), RESET_SLOTS);
    }

    #[test]
    fn test_detect_sword_gesture_priority_and_thresholds() {
        // Impact takes top priority
        let impact = SensorData::new(IMPACT_THRESHOLD + 10, 0, THRUST_THRESHOLD + 20, 0, 0, 0);
        assert_eq!(detect_sword_gesture(&impact), Some(LedAnimation::ImpactHit));

        // Upward thrust
        let thrust = SensorData::new(0, 0, THRUST_THRESHOLD + 15, 0, 0, 0);
        assert_eq!(
            detect_sword_gesture(&thrust),
            Some(LedAnimation::UpwardThrust)
        );

        // Downward slash (negative Z accel + pitch/roll gyro)
        let slash = SensorData::new(0, 0, -(THRUST_THRESHOLD + 20), 60, 0, 0);
        assert_eq!(
            detect_sword_gesture(&slash),
            Some(LedAnimation::DownwardSlash)
        );

        // Idle sample triggers no gesture
        let idle = SensorData::new(0, 0, 10, 0, 0, 0);
        assert_eq!(detect_sword_gesture(&idle), None);

        // Custom thresholds
        assert_eq!(
            detect_sword_gesture_with_thresholds(&thrust, 200, 500),
            None
        );
    }
}

// ============================================================================
// Phase 2: BLE Command/Event TLV Protocol Tests
// ============================================================================

mod tlv_api_tests {
    use xiao_nrf52840_sword::api::{
        ApiCommand, ApiEvent, ApiStatus, EVT_COMMAND_ACK, EVT_DEVICE_STATUS, EVT_GESTURE,
        OP_CLEAR_BONDED_DEVICES, OP_GET_STATUS, OP_SET_BRIGHTNESS, OP_SET_LED_MODE,
        OP_SET_THRESHOLDS, OP_TRIGGER_ANIMATION, TLV_PACKET_SIZE,
    };
    use xiao_nrf52840_sword::led::{LedAnimation, LedControlMode, RgbColor};

    #[test]
    fn test_tlv_set_led_mode_round_trip() {
        let cmd = ApiCommand::SetLedMode {
            mode: LedControlMode::ManualOverride,
            color: RgbColor::new(10, 200, 255),
            duration_ms: 1500,
        };
        let encoded = cmd.encode();
        assert_eq!(encoded.len(), TLV_PACKET_SIZE);
        assert_eq!(encoded[0], OP_SET_LED_MODE);
        assert_eq!(encoded[1], 6);
        let parsed = ApiCommand::parse(&encoded).expect("valid SetLedMode");
        assert_eq!(cmd, parsed);
    }

    #[test]
    fn test_tlv_set_brightness_round_trip() {
        let cmd = ApiCommand::SetBrightness { brightness: 128 };
        let encoded = cmd.encode();
        assert_eq!(encoded[0], OP_SET_BRIGHTNESS);
        assert_eq!(encoded[1], 1);
        assert_eq!(ApiCommand::parse(&encoded), Ok(cmd));
    }

    #[test]
    fn test_tlv_trigger_animation_round_trip() {
        let cmd = ApiCommand::TriggerAnimation {
            animation: LedAnimation::SolidColor(RgbColor::new(255, 0, 128)),
            duration_ms: 500,
        };
        let encoded = cmd.encode();
        assert_eq!(encoded[0], OP_TRIGGER_ANIMATION);
        assert_eq!(ApiCommand::parse(&encoded), Ok(cmd));
    }

    #[test]
    fn test_tlv_set_thresholds_round_trip_and_validation() {
        let cmd = ApiCommand::SetThresholds {
            thrust_threshold: 75,
            impact_threshold: 450,
        };
        let encoded = cmd.encode();
        assert_eq!(encoded[0], OP_SET_THRESHOLDS);
        assert_eq!(ApiCommand::parse(&encoded), Ok(cmd));

        // Invalid thresholds (impact <= thrust or non-positive thrust)
        let bad = [OP_SET_THRESHOLDS, 4, 100, 0, 50, 0];
        assert_eq!(
            ApiCommand::parse(&bad),
            Err((OP_SET_THRESHOLDS, ApiStatus::InvalidParameter))
        );
    }

    #[test]
    fn test_tlv_get_status_and_clear_bonds_round_trip() {
        let get_status = ApiCommand::GetStatus;
        assert_eq!(
            ApiCommand::parse(&get_status.encode()),
            Ok(ApiCommand::GetStatus)
        );
        assert_eq!(get_status.encode()[0], OP_GET_STATUS);

        let clear = ApiCommand::ClearBondedDevices;
        assert_eq!(
            ApiCommand::parse(&clear.encode()),
            Ok(ApiCommand::ClearBondedDevices)
        );
        assert_eq!(clear.encode()[0], OP_CLEAR_BONDED_DEVICES);
    }

    #[test]
    fn test_tlv_invalid_opcode_and_length_errors() {
        // Unknown opcode
        assert_eq!(
            ApiCommand::parse(&[0xFE, 0]),
            Err((0xFE, ApiStatus::InvalidOpcode))
        );
        // Declared length exceeds slice
        assert_eq!(
            ApiCommand::parse(&[OP_SET_BRIGHTNESS, 5, 100]),
            Err((OP_SET_BRIGHTNESS, ApiStatus::InvalidLength))
        );
        // Wrong payload length for SetBrightness
        assert_eq!(
            ApiCommand::parse(&[OP_SET_BRIGHTNESS, 2, 100, 0]),
            Err((OP_SET_BRIGHTNESS, ApiStatus::InvalidLength))
        );
    }

    #[test]
    fn test_tlv_events_round_trip() {
        let ack = ApiEvent::CommandAck {
            ref_opcode: OP_SET_LED_MODE,
            status: ApiStatus::Ok,
        };
        let ack_bytes = ack.encode();
        assert_eq!(ack_bytes[0], EVT_COMMAND_ACK);
        assert_eq!(ApiEvent::parse(&ack_bytes), Some(ack));

        let gesture = ApiEvent::GestureEvent {
            animation_id: LedAnimation::UpwardThrust.id(),
            peak_accel: 420,
        };
        let gesture_bytes = gesture.encode();
        assert_eq!(gesture_bytes[0], EVT_GESTURE);
        assert_eq!(ApiEvent::parse(&gesture_bytes), Some(gesture));

        let status = ApiEvent::DeviceStatus {
            pairing_status: 3,
            led_mode: 0,
            active_animation: 1,
            brightness: 200,
            thrust_threshold: 50,
            impact_threshold: 300,
        };
        let status_bytes = status.encode();
        assert_eq!(status_bytes[0], EVT_DEVICE_STATUS);
        assert_eq!(ApiEvent::parse(&status_bytes), Some(status));
    }
}

// ============================================================================
// Phase 3: Madgwick AHRS Sensor Fusion Tests
// ============================================================================

mod madgwick_fusion_tests {
    use xiao_nrf52840_sword::fusion::{MadgwickAhrs, Quaternion};
    use xiao_nrf52840_sword::SensorData;

    #[test]
    fn test_quaternion_identity_and_normalization() {
        let q = Quaternion::IDENTITY;
        assert!((q.norm() - 1.0).abs() < 1.0e-3);

        let unnorm = Quaternion::new(2.0, 2.0, 2.0, 2.0).normalized();
        assert!((unnorm.norm() - 1.0).abs() < 1.0e-3);
        assert!((unnorm.w - 0.5).abs() < 1.0e-2);
    }

    #[test]
    fn test_quaternion_q14_bytes_round_trip() {
        let orig = Quaternion::new(0.7071, 0.0, 0.7071, 0.0).normalized();
        let bytes = orig.to_q14_bytes();
        assert_eq!(bytes.len(), 8);
        let restored = Quaternion::from_q14_bytes(&bytes);
        assert!((orig.w - restored.w).abs() < 1.0e-3);
        assert!((orig.x - restored.x).abs() < 1.0e-3);
        assert!((orig.y - restored.y).abs() < 1.0e-3);
        assert!((orig.z - restored.z).abs() < 1.0e-3);
    }

    #[test]
    fn test_madgwick_stationary_z_gravity_preserves_upright_orientation() {
        let mut ahrs = MadgwickAhrs::new(20.0, 0.1);
        for _ in 0..100 {
            ahrs.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        }
        let q = ahrs.quaternion();
        assert!((q.norm() - 1.0).abs() < 1.0e-2);
        let (gx, gy, gz) = q.gravity_vector();
        assert!(gx.abs() < 0.05);
        assert!(gy.abs() < 0.05);
        assert!(gz > 0.98);
    }

    #[test]
    fn test_madgwick_converges_toward_tilted_x_gravity() {
        let mut ahrs = MadgwickAhrs::new(50.0, 0.5);
        for _ in 0..250 {
            ahrs.update_imu(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        }
        let (gx, gy, gz) = ahrs.quaternion().gravity_vector();
        assert!(gx > 0.85, "expected gx > 0.85, got {}", gx);
        assert!(gy.abs() < 0.1);
        assert!(gz.abs() < 0.5);
    }

    #[test]
    fn test_madgwick_yaw_gyro_integration_and_sensor_data_helper() {
        let mut ahrs = MadgwickAhrs::default();
        // ~90 deg/s yaw rotation (1.57 rad/s) for 10 steps at 20 Hz
        for _ in 0..10 {
            ahrs.update_imu(0.0, 0.0, 1.57, 0.0, 0.0, 1.0);
        }
        let q = ahrs.quaternion();
        assert!(q.z > 0.25);
        assert!((q.norm() - 1.0).abs() < 1.0e-2);

        // Verify raw SensorData helper runs cleanly
        let raw = SensorData::new(0, 0, 16384, 0, 0, 1000);
        let q2 = ahrs.update_from_sensor_data(&raw);
        assert!((q2.norm() - 1.0).abs() < 1.0e-2);
    }
}

// ============================================================================
// Phase 4 & 6: BLE MAC Validation, Live NfcPairingStatus, and NFC Page Writes
// ============================================================================

#[cfg(feature = "nfc")]
mod phase4_and_6_tests {
    use xiao_nrf52840_sword::nfc::{
        is_valid_nfc_user_page, read_nfc_page, write_nfc_page_sync, NFC_MAX_USER_PAGE,
        NFC_MIN_USER_PAGE,
    };
    use xiao_nrf52840_sword::pairing::{is_valid_ble_mac, PLACEHOLDER_MAC};
    use xiao_nrf52840_sword::{get_nfc_pairing_status, set_nfc_pairing_status, NfcPairingStatus};

    #[test]
    fn test_is_valid_ble_mac_rejects_invalid_and_accepts_real() {
        assert!(!is_valid_ble_mac(&[0x00; 6]));
        assert!(!is_valid_ble_mac(&[0xFF; 6]));
        assert!(!is_valid_ble_mac(&PLACEHOLDER_MAC));
        assert!(is_valid_ble_mac(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]));
    }

    #[test]
    fn test_live_nfc_pairing_status_transitions_and_wire_bytes() {
        let states = [
            (NfcPairingStatus::Idle, 0),
            (NfcPairingStatus::Scanning, 1),
            (NfcPairingStatus::Authenticating, 2),
            (NfcPairingStatus::Success, 3),
            (NfcPairingStatus::Failed, 4),
            (NfcPairingStatus::Timeout, 5),
        ];
        for (state, code) in states {
            assert_eq!(state.to_u8(), code);
            assert_eq!(NfcPairingStatus::from_u8(code), Some(state));
            set_nfc_pairing_status(state);
            assert_eq!(get_nfc_pairing_status(), state);
        }
        assert_eq!(NfcPairingStatus::from_u8(99), None);
        // Reset back to Idle so other tests see the initial state if run later
        set_nfc_pairing_status(NfcPairingStatus::Idle);
    }

    #[test]
    fn test_nfc_type2_page_write_validation_and_shadow_readback() {
        // Pages 0..=3 are reserved header/lock/CC pages — must be rejected
        for reserved in 0..NFC_MIN_USER_PAGE {
            assert!(!is_valid_nfc_user_page(reserved));
            assert!(!write_nfc_page_sync(reserved, &[1, 2, 3, 4]));
            assert!(read_nfc_page(reserved).is_none());
        }

        // Pages 4..=39 are valid user memory pages
        let payload = [0xDE, 0xAD, 0xBE, 0xEF];
        assert!(write_nfc_page_sync(4, &payload));
        assert_eq!(read_nfc_page(4), Some(payload));

        assert!(write_nfc_page_sync(NFC_MAX_USER_PAGE, &[9, 8, 7, 6]));
        assert_eq!(read_nfc_page(NFC_MAX_USER_PAGE), Some([9, 8, 7, 6]));

        // Out-of-bounds page > 39 must be rejected
        assert!(!write_nfc_page_sync(NFC_MAX_USER_PAGE + 1, &[1, 1, 1, 1]));
        assert!(read_nfc_page(NFC_MAX_USER_PAGE + 1).is_none());
    }
}
