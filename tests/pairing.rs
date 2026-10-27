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
        let cfg = BleConfig {
            adv_interval_units: 32,
            ..Default::default()
        };
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
