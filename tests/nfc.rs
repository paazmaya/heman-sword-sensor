#[cfg(feature = "nfc")]
mod common;

#[cfg(feature = "nfc")]
mod nfc_tests {
    use crate::common::MemoryFlash;
    use xiao_nrf52840_sword::{
        authenticate_bonded_device, get_nfc_pairing_status,
        nfc::{is_valid_uid, is_valid_uid_7, NfcFieldState},
        pairing::{
            authenticate_bonded_device as pairing_auth, authenticate_bonded_device_with_backend,
            authenticate_with_uid, authenticate_with_uid_backend, evaluate_nfc_pairing,
            get_bonded_devices, get_nfc_pairing_status as pairing_status, is_bonded_device,
            make_flash_record, parse_flash_record, read_bonded_device, read_bonded_device_full,
            read_bonded_device_mac, register_bonded_device, unregister_bonded_device,
            write_bonded_device_to_flash_full, BleConfig, BondedDeviceFull, PairingDecision,
            FLASH_BONDED_DEVICE_SIZE, FLASH_BONDED_DEVICE_START, FLASH_RECORD_MAGIC,
            FLASH_RECORD_SIZE, MAX_BONDED_DEVICES,
        },
        storage::{self, StorageError},
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
    fn compatibility_status_wrappers_share_one_coordinated_state() {
        let statuses = [
            NfcPairingStatus::Idle,
            NfcPairingStatus::Scanning,
            NfcPairingStatus::Authenticating,
            NfcPairingStatus::Success,
            NfcPairingStatus::Failed,
            NfcPairingStatus::Timeout,
        ];
        for status in statuses {
            xiao_nrf52840_sword::set_nfc_pairing_status(status);
            assert_eq!(get_nfc_pairing_status(), status);
            assert_eq!(pairing_status(), status);
        }
        xiao_nrf52840_sword::set_nfc_pairing_status(NfcPairingStatus::Idle);
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
    fn test_is_valid_uid_nonzero_identifier() {
        let uid = [0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        assert!(is_valid_uid(&uid));
    }

    #[test]
    fn test_is_valid_uid_all_zeros_invalid() {
        assert!(!is_valid_uid(&[0u8; 10]));
    }

    #[test]
    fn nonzero_uid_is_not_rejected_for_its_first_byte() {
        let uid = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        assert!(is_valid_uid(&uid));
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
    fn nonzero_seven_byte_uid_is_not_rejected_for_its_first_byte() {
        let uid = [0xFF, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56];
        assert!(is_valid_uid_7(&uid));
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
        // Existing reserved bond page; it is not the final page of 1 MiB flash.
        // The separate ELF check verifies application/storage disjointness.
        assert_eq!(FLASH_BONDED_DEVICE_START, 0x000E_F000);
        // Must be 4 KB-page-aligned
        assert_eq!(FLASH_BONDED_DEVICE_START % 4096, 0);
        assert_eq!(FLASH_BONDED_DEVICE_SIZE, 4096);
        // Must fit at least one record
        assert_eq!(MAX_BONDED_DEVICES, 1);
        // Flash record is 12 bytes (1 magic + 11 device)
        assert_eq!(FLASH_RECORD_SIZE, 12);
    }

    #[test]
    fn test_flash_start_above_softdevice_region() {
        // S140 SoftDevice occupies 0x00000000–0x00025FFF (152 KB).
        // Our storage must start above this boundary.
        const { assert!(FLASH_BONDED_DEVICE_START >= 0x0002_6000) };
    }

    #[test]
    fn test_flash_region_within_flash_bounds() {
        // nRF52840 has 1 MB flash (0x000F_FFFF end).
        let end = FLASH_BONDED_DEVICE_START + FLASH_BONDED_DEVICE_SIZE as u32;
        assert!(end <= 0x0010_0000);
    }

    #[test]
    fn test_max_bonded_devices_fits_in_page() {
        const { assert!(MAX_BONDED_DEVICES * FLASH_RECORD_SIZE <= FLASH_BONDED_DEVICE_SIZE) };
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
        assert_eq!(BondedDeviceFull::SERIALISED_SIZE, FLASH_RECORD_SIZE + 10);
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
        let restored = BondedDeviceFull::from_bytes(&bytes).unwrap();
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
        // The 12-byte primary envelope (magic + device) precedes the UID.
        assert_eq!(bytes[FLASH_RECORD_SIZE], 0x04);
        assert_eq!(bytes[FLASH_RECORD_SIZE + 1], 0xAB);
    }

    #[test]
    fn full_record_matches_independent_flash_wire_fixture() {
        let full = BondedDeviceFull {
            dev: BondedDevice {
                mac: [1, 2, 3, 4, 5, 6],
                timestamp: 0x0403_0201,
                flags: 0b111,
            },
            uid: [0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9],
        };
        assert_eq!(
            full.to_bytes(),
            [
                0xAB, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 0b111, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5,
                0xA6, 0xA7, 0xA8, 0xA9
            ]
        );
    }

    #[test]
    fn host_flash_reads_have_no_implicit_sample_device() {
        assert_eq!(read_bonded_device_full(), None);
        assert_eq!(read_bonded_device(), None);
        assert_eq!(read_bonded_device_mac(), None);
    }

    #[test]
    fn uid_prefix_is_not_mistaken_for_a_cascade_marker() {
        assert!(is_valid_uid(&[0x01, 1, 2, 3, 4, 5, 6, 7, 8, 9]));
    }

    // -----------------------------------------------------------------------
    // authenticate_bonded_device — top-level and pairing module
    // -----------------------------------------------------------------------

    #[test]
    fn implicit_authentication_without_backend_rejects_former_sample_mac() {
        let bonded_mac = [0x00u8, 11, 22, 33, 44, 55];
        assert!(!authenticate_bonded_device(&bonded_mac));
    }

    #[test]
    fn test_authenticate_bonded_device_wrong_mac_fails() {
        let wrong_mac = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        assert!(!authenticate_bonded_device(&wrong_mac));
    }

    #[test]
    fn pairing_implicit_authentication_without_backend_rejects_former_sample_mac() {
        let bonded_mac = [0x00u8, 11, 22, 33, 44, 55];
        assert!(!pairing_auth(&bonded_mac));
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
    fn implicit_uid_authentication_without_backend_cannot_succeed() {
        let mac = [0x00u8, 11, 22, 33, 44, 55];
        let uid = [0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        assert!(!authenticate_with_uid(&mac, &uid));
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
    fn backend_registration_write_read_and_clear_have_observable_effects() {
        let mut flash = MemoryFlash::default();
        let dev = BondedDevice {
            mac: [1, 2, 3, 4, 5, 6],
            timestamp: 0x1122_3344,
            flags: 0b111,
        };
        let uid = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13];

        write_bonded_device_to_flash_full(&mut flash, &dev, Some(&uid)).unwrap();
        assert_eq!(flash.erase_count, 1);
        assert_eq!(flash.write_count, 1);
        assert_eq!(storage::load_primary(&flash).unwrap(), Some(dev));
        assert_eq!(storage::load_full(&flash).unwrap(), Some((dev, uid)));
        assert!(authenticate_bonded_device_with_backend(&flash, &dev.mac).unwrap());
        assert!(authenticate_with_uid_backend(&flash, &dev.mac, &uid).unwrap());

        let updated = BondedDevice {
            timestamp: 0xAABB_CCDD,
            ..dev
        };
        register_bonded_device(&mut flash, &updated.mac).unwrap();
        assert_eq!(storage::load_primary(&flash).unwrap().unwrap().timestamp, 0);
        assert_eq!(flash.erase_count, 2);

        assert_eq!(
            unregister_bonded_device(&mut flash, &[9; 6]),
            Err(StorageError::NotFound)
        );
        unregister_bonded_device(&mut flash, &updated.mac).unwrap();
        assert_eq!(storage::load_primary(&flash).unwrap(), None);
        assert_eq!(flash.erase_count, 3);
    }

    // -----------------------------------------------------------------------
    // get_bonded_devices / is_bonded_device
    // -----------------------------------------------------------------------

    #[test]
    fn compatibility_enumeration_retains_two_slots_not_two_devices() {
        let devices = get_bonded_devices();
        assert_eq!(devices.len(), 2);
    }

    #[test]
    fn host_enumeration_is_empty_without_backend() {
        assert!(get_bonded_devices().iter().all(|dev| !dev.is_active()));
    }

    #[test]
    fn test_is_bonded_device_without_backend_returns_false() {
        let known = [0x00u8, 11, 22, 33, 44, 55];
        assert!(!is_bonded_device(&known));
    }

    #[test]
    fn test_is_bonded_device_unknown_mac_returns_false() {
        let unknown = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        assert!(!is_bonded_device(&unknown));
    }

    #[test]
    fn explicit_backend_read_write_failures_are_reported() {
        let dev = BondedDevice {
            mac: [1, 2, 3, 4, 5, 6],
            timestamp: 0,
            flags: 0b011,
        };
        let mut erase_failure = MemoryFlash {
            fail_erase: true,
            ..MemoryFlash::default()
        };
        assert_eq!(
            storage::store_device(&mut erase_failure, &dev, None),
            Err(StorageError::Erase)
        );
        assert_eq!(erase_failure.write_count, 0);

        let mut write_failure = MemoryFlash {
            fail_write: true,
            ..MemoryFlash::default()
        };
        assert_eq!(
            storage::store_device(&mut write_failure, &dev, None),
            Err(StorageError::Write)
        );
        assert_eq!(write_failure.erase_count, 1);
        assert_eq!(storage::load_primary(&write_failure).unwrap(), None);

        let read_failure = MemoryFlash {
            fail_read: true,
            ..MemoryFlash::default()
        };
        assert_eq!(
            storage::load_primary(&read_failure),
            Err(StorageError::Read)
        );
    }

    #[test]
    fn primary_full_and_padded_codecs_reject_absent_records_and_clear_state() {
        let device = BondedDevice {
            mac: [0x10, 0x20, 0x30, 0x40, 0x50, 0x60],
            timestamp: 0x0403_0201,
            flags: 0b011,
        };
        let primary = storage::encode_primary(&device);
        assert_eq!(
            primary,
            [0xAB, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 1, 2, 3, 4, 3]
        );
        assert_eq!(storage::decode_primary(&primary), Some(device));
        let mut bad_magic = primary;
        bad_magic[0] = 0;
        assert_eq!(storage::decode_primary(&bad_magic), None);
        let mut inactive = primary;
        inactive[11] = 0;
        assert_eq!(storage::decode_primary(&inactive), None);

        let padded = storage::encode_full_padded(&device, None);
        assert_eq!(&padded[..12], &primary);
        assert_eq!(&padded[12..], &[0xFF; 12]);
        assert_eq!(
            storage::decode_full_padded(&padded),
            Some((device, [0xFF; 10]))
        );
        let uid = [4, 3, 2, 1, 0, 9, 8, 7, 6, 5];
        let full = storage::encode_full(&device, &uid);
        assert_eq!(storage::decode_full(&full), Some((device, uid)));

        let mut flash = MemoryFlash::default();
        storage::store_device(&mut flash, &device, None).unwrap();
        assert_eq!(
            storage::load_full(&flash).unwrap(),
            Some((device, [0xFF; 10]))
        );
        storage::clear_devices(&mut flash).unwrap();
        assert_eq!(storage::load_full(&flash).unwrap(), None);
    }

    #[test]
    fn pairing_decision_uses_explicit_record_and_observed_values() {
        let record = BondedDeviceFull {
            dev: BondedDevice {
                mac: [1, 2, 3, 4, 5, 6],
                timestamp: 0,
                flags: 0b011,
            },
            uid: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
        };
        assert_eq!(
            evaluate_nfc_pairing(None, &record.dev.mac, &record.uid),
            PairingDecision::FirstPairOpenFallback
        );
        assert_eq!(
            evaluate_nfc_pairing(Some(&record), &record.dev.mac, &record.uid),
            PairingDecision::ExistingRecordMatched
        );
        let wrong_uid = [0xFF; 10];
        assert_eq!(
            evaluate_nfc_pairing(Some(&record), &record.dev.mac, &wrong_uid),
            PairingDecision::Rejected
        );
        let wrong_mac = [0xFF; 6];
        assert_eq!(
            evaluate_nfc_pairing(Some(&record), &wrong_mac, &record.uid),
            PairingDecision::Rejected
        );
    }
}
