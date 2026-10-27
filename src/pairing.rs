//! NFC pairing decisions, single-record persistence, and BLE configuration.

use crate::*;

/// Start address of the bonded-device storage region.
pub const FLASH_BONDED_DEVICE_START: u32 = crate::storage::FLASH_BONDED_DEVICE_START;
/// Size of the reserved bond page.
pub const FLASH_BONDED_DEVICE_SIZE: usize = crate::storage::FLASH_BONDED_DEVICE_SIZE;
/// This implementation persists one bonded device record.
pub const MAX_BONDED_DEVICES: usize = 1;
/// Magic marker written at the start of a valid flash record.
pub const FLASH_RECORD_MAGIC: u8 = crate::storage::FLASH_RECORD_MAGIC;
/// Total byte size of the primary flash record.
pub const FLASH_RECORD_SIZE: usize = crate::storage::FLASH_RECORD_SIZE;

// ------------------------------------------------------------------
// Pure helpers (no hardware, testable on host)
// ------------------------------------------------------------------

/// Parse a primary bonded-device flash record.
pub fn parse_flash_record(record: &[u8; FLASH_RECORD_SIZE]) -> Option<BondedDevice> {
    crate::storage::decode_primary(record)
}

/// Serialise a bonded device into the primary 12-byte flash envelope.
pub fn make_flash_record(device: &BondedDevice) -> [u8; FLASH_RECORD_SIZE] {
    crate::storage::encode_primary(device)
}

/// Compare a supplied MAC with the current active, paired stored record.
///
/// The device passes when:
/// - Its MAC matches the stored MAC exactly.
/// - The stored record is active (bit 0 set).
/// - The stored record is marked as paired (bit 1 set).
///
/// This software comparison does not prove a BLE central's identity.
pub fn authenticate_bonded_device(mac: &[u8; 6]) -> bool {
    #[cfg(feature = "embedded")]
    defmt::info!("🔐 Authenticating device MAC: {:?}", mac);

    let authenticated = matches!(
        read_bonded_device(),
        Some(device) if device.mac == *mac && device.is_active() && device.is_paired()
    );
    #[cfg(feature = "embedded")]
    if authenticated {
        defmt::info!("✅ Authentication successful");
    } else {
        defmt::warn!("❌ Authentication failed – MAC not found or device not active/paired");
    }
    authenticated
}

/// Outcome of evaluating an NFC UID against the single stored device record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PairingDecision {
    /// Existing active, paired record matched the supplied MAC and UID bytes.
    ExistingRecordMatched,
    /// No record exists; caller may continue with the agreed open fallback.
    FirstPairOpenFallback,
    /// A record exists but is inactive, unpaired, or has a different UID/MAC.
    Rejected,
}

/// Evaluate the pairing rule from explicit state without reading global flash.
///
/// The `observed_uid` is only a comparison value; equality is not cryptographic
/// proof of a BLE central's identity.
pub fn evaluate_nfc_pairing(
    stored: Option<&BondedDeviceFull>,
    observed_mac: &[u8; 6],
    observed_uid: &[u8; 10],
) -> PairingDecision {
    match stored {
        None => PairingDecision::FirstPairOpenFallback,
        Some(record)
            if record.dev.is_active()
                && record.dev.is_paired()
                && &record.dev.mac == observed_mac
                && &record.uid == observed_uid =>
        {
            PairingDecision::ExistingRecordMatched
        }
        Some(_) => PairingDecision::Rejected,
    }
}

/// Compare both the MAC address and 10-byte UID with the stored record.
///
/// Returns true only when both values match and the record is active and
/// paired. This is a software allow-list comparison, not cryptographic
/// authentication of the BLE central.
pub fn authenticate_with_uid(mac: &[u8; 6], uid: &[u8; 10]) -> bool {
    #[cfg(feature = "embedded")]
    {
        defmt::info!("🔐 Authenticating with MAC + NFC UID");
        defmt::info!("   MAC: {:?}", mac);
        defmt::info!("   UID: {:?}", uid);
    }

    let stored = read_bonded_device_full();
    let authenticated =
        evaluate_nfc_pairing(stored.as_ref(), mac, uid) == PairingDecision::ExistingRecordMatched;
    #[cfg(feature = "embedded")]
    if authenticated {
        defmt::info!("✅ MAC + UID authentication successful");
    } else {
        defmt::warn!("❌ MAC + UID authentication failed");
    }
    authenticated
}

/// Authenticate a MAC against an explicit backend, propagating read failures.
pub fn authenticate_bonded_device_with_backend(
    backend: &impl crate::storage::BondedDeviceBackend,
    mac: &[u8; 6],
) -> Result<bool, crate::storage::StorageError> {
    Ok(crate::storage::load_primary(backend)?
        .is_some_and(|device| device.mac == *mac && device.is_active() && device.is_paired()))
}

/// Authenticate a MAC and UID against an explicit backend.
pub fn authenticate_with_uid_backend(
    backend: &impl crate::storage::BondedDeviceBackend,
    mac: &[u8; 6],
    uid: &[u8; 10],
) -> Result<bool, crate::storage::StorageError> {
    Ok(
        crate::storage::load_full(backend)?.is_some_and(|(device, stored_uid)| {
            device.mac == *mac && stored_uid == *uid && device.is_active() && device.is_paired()
        }),
    )
}

/// Return the current pairing status tracked in the shared atomic state.
pub fn get_nfc_pairing_status() -> NfcPairingStatus {
    super::nfc_pairing_status()
}

/// Set the current pairing status tracked in the shared atomic state.
pub fn set_nfc_pairing_status(status: NfcPairingStatus) {
    super::set_nfc_pairing_status(status);
}

/// First-time pairing placeholder MAC address stored prior to central registration.
pub const PLACEHOLDER_MAC: [u8; 6] = [0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01];

/// Validate a 6-byte BLE MAC address written via `pairing_mac`.
///
/// Rejects all-zero (`00:00:00:00:00:00`), broadcast (`FF:FF:FF:FF:FF:FF`),
/// and the internal first-time pairing placeholder (`DE:AD:BE:EF:00:01`).
pub fn is_valid_ble_mac(mac: &[u8; 6]) -> bool {
    if mac.iter().all(|&b| b == 0x00) {
        return false;
    }
    if mac.iter().all(|&b| b == 0xFF) {
        return false;
    }
    if *mac == PLACEHOLDER_MAC {
        return false;
    }
    true
}

// ------------------------------------------------------------------
// Extended bonded device record (MAC + UID)
// ------------------------------------------------------------------

/// A bonded device record together with the UID stored in the extended record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BondedDeviceFull {
    /// Core device data.
    pub dev: BondedDevice,
    /// Ten UID bytes from the flash envelope; all `0xFF` denotes absent UID.
    pub uid: [u8; 10],
}

impl BondedDeviceFull {
    /// Length of the actual extended flash record, excluding two alignment bytes.
    pub const SERIALISED_SIZE: usize = crate::storage::FULL_RECORD_SIZE;

    /// Serialize as the 22-byte flash envelope (magic, device, UID).
    pub fn to_bytes(&self) -> [u8; Self::SERIALISED_SIZE] {
        crate::storage::encode_full(&self.dev, &self.uid)
    }

    /// Decode a 22-byte flash envelope.
    pub fn from_bytes(buf: &[u8; Self::SERIALISED_SIZE]) -> Option<Self> {
        let (dev, uid) = crate::storage::decode_full(buf)?;
        Some(Self { dev, uid })
    }
}

// ------------------------------------------------------------------
// Compatibility reads use mapped flash on the board. Writes require an explicit
// backend; host callers never receive an implicit sample device.
// ------------------------------------------------------------------

/// Read the primary bonded device MAC from flash.
///
/// On the nRF52840 target, flash is memory-mapped starting at 0x00000000.
/// We read [`BONDED_DEVICE_STRUCT_SIZE`] bytes at [`FLASH_BONDED_DEVICE_START`]
/// plus one magic byte offset.
pub fn read_bonded_device_mac() -> Option<[u8; 6]> {
    read_bonded_device().map(|d| d.mac)
}

/// Read the primary [`BondedDevice`] record from flash.
pub fn read_bonded_device() -> Option<BondedDevice> {
    #[cfg(feature = "embedded")]
    {
        // On the nRF52840, flash is directly readable as memory.
        // Read FLASH_RECORD_SIZE bytes at FLASH_BONDED_DEVICE_START.
        let addr = FLASH_BONDED_DEVICE_START as *const u8;
        let mut record = [0u8; FLASH_RECORD_SIZE];
        // Fixed range within the nRF52840 bond page, reserved by memory.x.
        unsafe {
            for (i, byte) in record.iter_mut().enumerate() {
                *byte = addr.add(i).read_volatile();
            }
        }
        parse_flash_record(&record)
    }

    #[cfg(not(feature = "embedded"))]
    {
        None
    }
}

/// Read the extended bonded device record (MAC + NFC UID) from flash.
pub fn read_bonded_device_full() -> Option<BondedDeviceFull> {
    #[cfg(feature = "embedded")]
    {
        let addr = FLASH_BONDED_DEVICE_START as *const u8;
        let mut buf = [0xFFu8; crate::storage::FULL_RECORD_SIZE];
        // Fixed range within the nRF52840 bond page, reserved by memory.x.
        unsafe {
            for (i, byte) in buf.iter_mut().enumerate() {
                *byte = addr.add(i).read_volatile();
            }
        }
        BondedDeviceFull::from_bytes(&buf).filter(|full| full.dev.is_paired())
    }

    #[cfg(not(feature = "embedded"))]
    {
        None
    }
}

/// Persist the primary device through an explicit backend.
pub fn write_bonded_device_to_flash(
    backend: &mut impl crate::storage::BondedDeviceBackend,
    mac: &[u8; 6],
) -> Result<(), crate::storage::StorageError> {
    let dev = BondedDevice {
        mac: *mac,
        timestamp: 0,
        flags: 0b011,
    };
    crate::storage::store_device(backend, &dev, None)
}

/// Persist a device and optional UID through an explicit backend.
pub fn write_bonded_device_to_flash_full(
    backend: &mut impl crate::storage::BondedDeviceBackend,
    device: &BondedDevice,
    uid: Option<&[u8; 10]>,
) -> Result<(), crate::storage::StorageError> {
    crate::storage::store_device(backend, device, uid)
}

/// Register a new bonded device through an explicit backend.
pub fn register_bonded_device(
    backend: &mut impl crate::storage::BondedDeviceBackend,
    mac: &[u8; 6],
) -> Result<(), crate::storage::StorageError> {
    write_bonded_device_to_flash(backend, mac)
}

/// Unregister a matching device through an explicit backend.
pub fn unregister_bonded_device(
    backend: &mut impl crate::storage::BondedDeviceBackend,
    mac: &[u8; 6],
) -> Result<(), crate::storage::StorageError> {
    crate::storage::remove_device(backend, mac)
}

/// Return all registered bonded devices from flash.
///
/// Return the single supported record in the first array slot, if present.
pub fn get_bonded_devices() -> [BondedDevice; 2] {
    #[cfg(feature = "embedded")]
    {
        let base = FLASH_BONDED_DEVICE_START as *const u8;
        let mut devices = [BondedDevice {
            mac: [0u8; 6],
            timestamp: 0,
            flags: 0,
        }; 2];
        let mut record = [0u8; FLASH_RECORD_SIZE];
        // Fixed range within the nRF52840 bond page, reserved by memory.x.
        unsafe {
            for (i, byte) in record.iter_mut().enumerate() {
                *byte = base.add(i).read_volatile();
            }
        }
        if let Some(dev) = parse_flash_record(&record) {
            devices[0] = dev;
        }
        devices
    }

    #[cfg(not(feature = "embedded"))]
    {
        [BondedDevice {
            mac: [0; 6],
            timestamp: 0,
            flags: 0,
        }; 2]
    }
}

/// Check whether `mac` matches any registered bonded device.
pub fn is_bonded_device(mac: &[u8; 6]) -> bool {
    get_bonded_devices()
        .iter()
        .any(|d| d.is_active() && d.mac == *mac)
}

// ------------------------------------------------------------------
// BLE configuration
// ------------------------------------------------------------------

/// Compatibility advertising settings for callers, not the live SoftDevice
/// configuration. Firmware configures advertising directly in its BLE adapter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BleConfig {
    /// Advertising interval in units of 0.625 ms (e.g. 160 = 100 ms).
    pub adv_interval_units: u16,
    /// Bonded device MAC address for directed advertising / whitelist.
    pub bonded_mac: Option<[u8; 6]>,
    /// Advisory connection timeout in seconds; the firmware does not enforce it.
    pub connection_timeout_secs: u32,
    /// Use whitelist (advertise only to bonded device).
    pub use_whitelist: bool,
}

impl Default for BleConfig {
    fn default() -> Self {
        BleConfig {
            adv_interval_units: 160, // 160 × 0.625 ms = 100 ms
            bonded_mac: None,
            connection_timeout_secs: 30,
            use_whitelist: false,
        }
    }
}

impl BleConfig {
    /// Create a whitelist-only config targeting the given bonded MAC.
    pub fn bonded_only(mac: [u8; 6]) -> Self {
        BleConfig {
            bonded_mac: Some(mac),
            use_whitelist: true,
            ..Default::default()
        }
    }

    /// Advertising interval in milliseconds (approximate).
    pub fn adv_interval_ms(&self) -> u32 {
        self.adv_interval_units as u32 * 625 / 1000
    }
}

// ------------------------------------------------------------------
// Async pairing orchestration — requires embassy runtime
// ------------------------------------------------------------------

/// Run the embedded NFC pairing gate.
///
/// The function waits up to [`NFC_PAIRING_TIMEOUT_SECS`] for the local tag UID
/// store to contain a value. An existing record is compared with that UID;
/// when no record exists, the agreed open-advertising fallback is preserved
/// and this function returns `false`. A timeout also returns `false`.
///
/// The NFCT activation must be started concurrently by the firmware adapter.
/// A local tag UID match is not cryptographic authentication of a BLE central.
#[cfg(feature = "embedded")]
pub async fn pairing_mode() -> bool {
    use embassy_time::{Duration, Instant, Timer};

    defmt::info!("═══════════════════════════════════════════");
    defmt::info!("🔌 NFC Pairing Mode – Primary Authentication Gate");
    defmt::info!("   Timeout: {} s", NFC_PAIRING_TIMEOUT_SECS);
    defmt::info!("═══════════════════════════════════════════");

    let start = Instant::now();
    let timeout = Duration::from_secs(NFC_PAIRING_TIMEOUT_SECS);
    set_nfc_pairing_status(NfcPairingStatus::Scanning);

    loop {
        // Check whether main.rs has stored an activated UID
        if let Some(uid) = nfc::peek_activated_uid() {
            defmt::info!("✅ NFC UID present – initiating pairing…");
            set_nfc_pairing_status(NfcPairingStatus::Authenticating);

            let stored = read_bonded_device_full();
            let observed_mac = stored
                .as_ref()
                .map(|record| record.dev.mac)
                .unwrap_or(PLACEHOLDER_MAC);
            match evaluate_nfc_pairing(stored.as_ref(), &observed_mac, &uid) {
                PairingDecision::ExistingRecordMatched => {
                    defmt::info!("✅ Stored UID comparison succeeded");
                    set_nfc_pairing_status(NfcPairingStatus::Success);
                    nfc::clear_activated_uid();
                    return true;
                }
                PairingDecision::FirstPairOpenFallback => {
                    // Persistence is performed only by the explicit flash
                    // backend used by the firmware.
                    defmt::warn!("⚠️  No bonded device in flash – continuing to open advertising");
                    set_nfc_pairing_status(NfcPairingStatus::Success);
                    nfc::clear_activated_uid();
                    return false;
                }
                PairingDecision::Rejected => {
                    defmt::warn!("❌ Stored UID comparison failed");
                    set_nfc_pairing_status(NfcPairingStatus::Failed);
                    nfc::clear_activated_uid();
                    return false;
                }
            }
        }

        if start.elapsed() >= timeout {
            defmt::info!("⏱️  NFC pairing timeout – falling back to BLE advertising");
            set_nfc_pairing_status(NfcPairingStatus::Timeout);
            return false;
        }

        Timer::after_millis(100).await;
    }
}
