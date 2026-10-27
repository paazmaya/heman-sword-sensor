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
    // parse_nfct_uid — rejects zero placeholders without claiming authenticity
    // -----------------------------------------------------------------------

    #[test]
    fn test_parse_nfct_uid_valid_4_byte_identifier() {
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
    fn parse_nfct_uid_accepts_any_nonzero_first_byte() {
        // A UID byte is not a cascade marker.
        let raw = [0x00u8, 0x01, 0x02, 0x03];
        assert!(parse_nfct_uid(&raw).is_some());
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
    fn parse_nfct_uid_7_byte_does_not_require_cascade_prefix() {
        let raw = [0xFFu8, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56];
        assert!(parse_nfct_uid(&raw).is_some());
    }

    #[test]
    fn test_extract_and_parse_agree_on_valid_uid() {
        let raw = [0x04u8, 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02, 0x03, 0x04];
        let extracted = extract_uid_from_nfct(&raw).unwrap();
        let parsed = parse_nfct_uid(&raw).unwrap();
        assert_eq!(extracted, parsed);
    }
}
