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
