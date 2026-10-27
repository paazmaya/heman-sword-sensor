//! Embedded nfc adapter. Hardware-independent contracts live in the library.

use crate::firmware::flash::{
    flash_read_bonded_device_full, sd_write_bonded_device, sd_write_bonded_device_full,
};
use defmt::{info, warn};
use embassy_nrf::nfct::NfcT;
use embassy_time::Duration;
use nrf_softdevice::Flash as SdFlash;
use xiao_nrf52840_sword::{
    nfc::{parse_nfct_uid, store_activated_uid},
    pairing::{evaluate_nfc_pairing, BondedDeviceFull, PairingDecision, PLACEHOLDER_MAC},
    set_nfc_pairing_status, BondedDevice, NfcPairingStatus, NFC_PAIRING_TIMEOUT_SECS,
};

/// Local tag identifier, shared with NFCT configuration; never a reader identity.
pub(crate) const LOCAL_NFC_ID: [u8; 4] = [0x04, 0x01, 0x02, 0x03];

/// Wait for an NFC field using the real embassy-nrf NFCT peripheral.
/// Returns `true` if a field was detected within `timeout_ms` milliseconds.
pub(crate) async fn detect_nfc_field(nfct: &mut NfcT<'static>, timeout_ms: u64) -> bool {
    info!("📡 NFC scan – timeout {} ms", timeout_ms);
    set_nfc_pairing_status(NfcPairingStatus::Scanning);
    let result =
        embassy_time::with_timeout(Duration::from_millis(timeout_ms), nfct.activate()).await;
    match result {
        Ok(_) => {
            info!("✅ NFC field detected");
            set_nfc_pairing_status(NfcPairingStatus::Authenticating);
            true
        }
        Err(_) => {
            info!("⏱️  NFC timeout");
            set_nfc_pairing_status(NfcPairingStatus::Timeout);
            false
        }
    }
}

/// Run the boot-only selection gate; an NFC reader is not authenticated.
pub(crate) async fn pairing_gate(
    nfct: &mut NfcT<'static>,
    sd_flash: &mut SdFlash,
) -> Option<[u8; 6]> {
    let existing_device = flash_read_bonded_device_full().map(|(device, _)| device);
    info!(
        "   Flash: {}",
        if existing_device.is_some() {
            "bonded device found"
        } else {
            "no bonded device"
        }
    );

    let nfc_detected = detect_nfc_field(nfct, NFC_PAIRING_TIMEOUT_SECS * 1000).await;

    let nfc_uid: Option<[u8; 10]> = if nfc_detected {
        let uid = parse_nfct_uid(&LOCAL_NFC_ID);
        store_activated_uid(&LOCAL_NFC_ID);
        uid
    } else {
        None
    };

    // Decide pairing outcome using SoftDevice Flash API for any writes
    let bonded_mac: Option<[u8; 6]> = if nfc_detected {
        match existing_device {
            Some(ref dev) => {
                let uid_matches = nfc_uid.is_some_and(|uid| {
                    let stored = flash_read_bonded_device_full()
                        .map(|(dev, uid)| BondedDeviceFull { dev, uid });
                    evaluate_nfc_pairing(stored.as_ref(), &dev.mac, &uid)
                        == PairingDecision::ExistingRecordMatched
                });
                if uid_matches {
                    info!("✅ NFC selected – existing local tag record confirmed (not reader authentication)");
                    set_nfc_pairing_status(NfcPairingStatus::Success);
                    Some(dev.mac)
                } else {
                    warn!("❌ NFC selected – stored local tag record does not match");
                    set_nfc_pairing_status(NfcPairingStatus::Failed);
                    None
                }
            }
            None => {
                info!("🆕 NFC detected – first-time pairing: registering new device");
                let mut dev = BondedDevice::new(PLACEHOLDER_MAC);
                dev.set_paired(true);
                dev.set_verified(nfc_uid.is_some());

                let persisted = if let Some(uid) = nfc_uid {
                    sd_write_bonded_device_full(sd_flash, &dev, &uid).await
                } else {
                    sd_write_bonded_device(sd_flash, &dev).await
                };
                if persisted {
                    set_nfc_pairing_status(NfcPairingStatus::Success);
                    Some(PLACEHOLDER_MAC)
                } else {
                    set_nfc_pairing_status(NfcPairingStatus::Failed);
                    None
                }
            }
        }
    } else {
        info!("⏱️  NFC timeout – using existing flash record (if any)");
        set_nfc_pairing_status(NfcPairingStatus::Timeout);
        existing_device.map(|d| d.mac)
    };

    bonded_mac
}
