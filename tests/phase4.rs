#[cfg(feature = "nfc")]
mod phase4_and_6_tests {
    use xiao_nrf52840_sword::nfc::{
        is_valid_nfc_user_page, read_nfc_page, write_nfc_page_sync, NfcPageShadow,
        NFC_MAX_USER_PAGE, NFC_MIN_USER_PAGE,
    };
    use xiao_nrf52840_sword::pairing::{is_valid_ble_mac, PLACEHOLDER_MAC};
    use xiao_nrf52840_sword::{NfcPairingStatus, NfcPairingStatusCell};

    #[test]
    fn test_is_valid_ble_mac_rejects_invalid_and_accepts_real() {
        assert!(!is_valid_ble_mac(&[0x00; 6]));
        assert!(!is_valid_ble_mac(&[0xFF; 6]));
        assert!(!is_valid_ble_mac(&PLACEHOLDER_MAC));
        assert!(is_valid_ble_mac(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]));
    }

    #[test]
    fn test_live_nfc_pairing_status_transitions_and_wire_bytes() {
        let status_cell = NfcPairingStatusCell::new();
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
            status_cell.set(state);
            assert_eq!(status_cell.get(), state);
        }
        assert_eq!(NfcPairingStatus::from_u8(99), None);
        assert_eq!(
            NfcPairingStatusCell::default().get(),
            NfcPairingStatus::Idle
        );
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

    #[test]
    fn nfc_page_shadow_instances_are_isolated_and_fresh() {
        let first = NfcPageShadow::new();
        let second = NfcPageShadow::default();
        assert_eq!(first.read(4), Some([0; 4]));
        assert_eq!(second.read(4), Some([0; 4]));
        assert!(first.write(4, &[1, 2, 3, 4]));
        assert_eq!(first.read(4), Some([1, 2, 3, 4]));
        assert_eq!(second.read(4), Some([0; 4]));
        assert!(!first.write(3, &[5; 4]));
        assert_eq!(second.read(3), None);
    }
}
