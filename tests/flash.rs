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
        const {
            assert!(
                FLASH_BONDED_DEVICE_START >= 0x0002_6000,
                "Flash storage must not overlap the S140 SoftDevice region"
            )
        };
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
    fn test_flash_at_existing_reserved_page() {
        // Preserve the deployed record address; memory.x reserves it explicitly.
        assert_eq!(FLASH_BONDED_DEVICE_START, 0x000E_F000);
    }

    #[test]
    fn test_max_bonded_devices_page_fit() {
        const { assert!(MAX_BONDED_DEVICES * FLASH_RECORD_SIZE <= FLASH_BONDED_DEVICE_SIZE) };
    }
}
