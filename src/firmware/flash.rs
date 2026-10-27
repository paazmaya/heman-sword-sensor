//! Embedded flash adapter. Hardware-independent contracts live in the library.

use crate::{
    firmware::ble::{emit_event, StorageCommand, STORAGE_CHANNEL, WHITELIST_UPDATE_SIGNAL},
    CONNECTION_EPOCH,
};
use core::sync::atomic::Ordering;
use defmt::{info, warn};
use embedded_storage_async::nor_flash::NorFlash;
use nrf_softdevice::Flash as SdFlash;
use xiao_nrf52840_sword::{
    api::{ApiEvent, ApiStatus, OP_CLEAR_BONDED_DEVICES},
    set_nfc_pairing_status,
    storage::{
        clear_devices_async, decode_full, store_device_async, AsyncBondedDeviceBackend,
        StorageError, FLASH_BONDED_DEVICE_SIZE, FLASH_BONDED_DEVICE_START, FULL_RECORD_SIZE,
    },
    BondedDevice, NfcPairingStatus,
};

struct SoftdeviceStorage<'a>(&'a mut SdFlash);

impl AsyncBondedDeviceBackend for SoftdeviceStorage<'_> {
    async fn erase_page(&mut self, address: u32) -> Result<(), StorageError> {
        self.0
            .erase(address, address + FLASH_BONDED_DEVICE_SIZE as u32)
            .await
            .map_err(|error| {
                warn!("SoftDevice erase failed: {:?}", error);
                StorageError::Erase
            })
    }

    async fn write(&mut self, address: u32, bytes: &[u8]) -> Result<(), StorageError> {
        self.0.write(address, bytes).await.map_err(|error| {
            warn!("SoftDevice write failed: {:?}", error);
            StorageError::Write
        })
    }
}

/// Read the bonded device + 10-byte UID directly from memory-mapped flash.
///
/// Flash is mapped at `0x0000_0000` on nRF52840, so reads can be performed
/// synchronously via `read_volatile` without disturbing the SoftDevice radio.
pub(crate) fn flash_read_bonded_device_full() -> Option<(BondedDevice, [u8; 10])> {
    let addr = FLASH_BONDED_DEVICE_START as *const u8;
    let mut buf = [0u8; FULL_RECORD_SIZE];
    // This fixed range is within the nRF52840's mapped, linker-reserved bond page.
    unsafe {
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = addr.add(i).read_volatile();
        }
    }
    decode_full(&buf)
}

/// Write one bonded device record to flash using the SoftDevice Flash API
/// (`sd_flash_page_erase` + `sd_flash_write` via [`nrf_softdevice::Flash`]).
pub(crate) async fn sd_write_bonded_device(flash: &mut SdFlash, device: &BondedDevice) -> bool {
    write_record(flash, device, None).await
}

/// Write a full bonded device record (MAC + 10-byte NFC UID) to flash via SoftDevice Flash API.
pub(crate) async fn sd_write_bonded_device_full(
    flash: &mut SdFlash,
    device: &BondedDevice,
    uid: &[u8; 10],
) -> bool {
    write_record(flash, device, Some(uid)).await
}

async fn write_record(flash: &mut SdFlash, device: &BondedDevice, uid: Option<&[u8; 10]>) -> bool {
    match store_device_async(&mut SoftdeviceStorage(flash), device, uid).await {
        Ok(()) => {
            info!(
                "💾 SoftDevice flash write OK – MAC {:?} @ 0x{:08X}",
                device.mac, FLASH_BONDED_DEVICE_START
            );
            true
        }
        Err(error) => {
            warn!("SoftDevice flash write failed (full record): {:?}", error);
            false
        }
    }
}

/// Erase the bonded-device flash page via SoftDevice Flash API.
pub(crate) async fn sd_clear_bonded_devices(flash: &mut SdFlash) -> bool {
    match clear_devices_async(&mut SoftdeviceStorage(flash)).await {
        Ok(()) => {
            info!(
                "🗑️  SoftDevice flash page erased @ 0x{:08X}",
                FLASH_BONDED_DEVICE_START
            );
            true
        }
        Err(error) => {
            warn!(
                "SoftDevice flash erase failed during bond clear: {:?}",
                error
            );
            false
        }
    }
}

/// Execute the next ordered persistence request without cancelling in-flight flash operations.
pub(crate) async fn process_storage_request(sd_flash: &mut SdFlash) {
    if let Ok(command) = STORAGE_CHANNEL.try_receive() {
        let epoch = match &command {
            StorageCommand::Clear { epoch } | StorageCommand::ReplaceMac { epoch, .. } => *epoch,
        };
        if epoch != CONNECTION_EPOCH.load(Ordering::Relaxed) {
            warn!("Discarding queued storage command from disconnected client");
            return;
        }
        match command {
            StorageCommand::Clear { epoch } => {
                info!("🗑️  Clearing bonded devices from flash");
                let cleared = sd_clear_bonded_devices(sd_flash).await;
                if cleared {
                    WHITELIST_UPDATE_SIGNAL.signal(None);
                }
                set_nfc_pairing_status(if cleared {
                    NfcPairingStatus::Idle
                } else {
                    NfcPairingStatus::Failed
                });
                emit_event(
                    epoch,
                    ApiEvent::CommandAck {
                        ref_opcode: OP_CLEAR_BONDED_DEVICES,
                        status: if cleared {
                            ApiStatus::Ok
                        } else {
                            ApiStatus::StorageError
                        },
                    },
                );
            }
            StorageCommand::ReplaceMac { mac: new_mac, .. } => {
                info!(
                    "🔄 Updating bonded MAC from placeholder to real: {:?}",
                    new_mac
                );
                let persisted = match flash_read_bonded_device_full() {
                    Some((mut dev, uid)) => {
                        dev.mac = new_mac;
                        sd_write_bonded_device_full(sd_flash, &dev, &uid).await
                    }
                    None => {
                        let mut dev = BondedDevice::new(new_mac);
                        dev.set_paired(true);
                        sd_write_bonded_device(sd_flash, &dev).await
                    }
                };

                if persisted {
                    info!("   ✅ MAC persisted via SoftDevice flash; hot-reloading BLE whitelist");
                    WHITELIST_UPDATE_SIGNAL.signal(Some(new_mac));
                }
                set_nfc_pairing_status(if persisted {
                    NfcPairingStatus::Success
                } else {
                    NfcPairingStatus::Failed
                });
            }
        }
    }
}
