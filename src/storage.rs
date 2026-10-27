//! Bonded-device flash codecs and backend-driven persistence.

/// Serialized size of the device data without the flash envelope byte.
pub const BONDED_DEVICE_STRUCT_SIZE: usize = 11;
/// Magic prefix in the 12-byte primary record stored at the bond-page origin.
pub const FLASH_RECORD_MAGIC: u8 = 0xAB;
/// Byte length of the primary flash envelope: magic plus the 11-byte record.
pub const FLASH_RECORD_SIZE: usize = 12;
/// Byte length of the extended flash envelope including its 10-byte UID.
pub const FULL_RECORD_SIZE: usize = FLASH_RECORD_SIZE + 10;
/// Four-byte aligned size used by the flash write API for extended records.
pub const FULL_RECORD_PADDED_SIZE: usize = 24;
/// nRF52840 bond page address (the page is reserved separately from application flash).
pub const FLASH_BONDED_DEVICE_START: u32 = 0x000E_F000;
/// Size of the reserved bond page.
pub const FLASH_BONDED_DEVICE_SIZE: usize = 4096;

/// Read, erase, and write failures for a bonded-device backend.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
pub enum StorageError {
    /// The backend could not read the requested range.
    Read,
    /// The backend could not erase the bond page.
    Erase,
    /// The backend could not write the encoded record.
    Write,
    /// No active record matches the requested device.
    NotFound,
}

/// Minimal backend contract for operations on the single reserved bond page.
///
/// Implementations must not report success until the corresponding operation
/// has completed. This trait deliberately leaves flash-driver and async policy
/// to the caller.
pub trait BondedDeviceBackend {
    /// Read bytes from the absolute flash address.
    fn read(&self, address: u32, output: &mut [u8]) -> Result<(), StorageError>;

    /// Erase the flash page containing the absolute address.
    fn erase_page(&mut self, address: u32) -> Result<(), StorageError>;

    /// Write bytes at an absolute flash address.
    fn write(&mut self, address: u32, bytes: &[u8]) -> Result<(), StorageError>;
}

/// Non-blocking erase/write adapter for the single bond page.
///
/// The returned futures must complete only when flash has finished. They must
/// not be cancelled while a hardware operation is in flight.
pub trait AsyncBondedDeviceBackend {
    /// Erase the reserved page at the supplied absolute address.
    fn erase_page(
        &mut self,
        address: u32,
    ) -> impl core::future::Future<Output = Result<(), StorageError>>;

    /// Write aligned bytes to an absolute address after the erase has completed.
    fn write(
        &mut self,
        address: u32,
        bytes: &[u8],
    ) -> impl core::future::Future<Output = Result<(), StorageError>>;
}

#[repr(C, align(4))]
struct AlignedFullRecord([u8; FULL_RECORD_PADDED_SIZE]);

/// A device record used by the firmware for BLE allow-list decisions.
///
/// Stored in flash starting at [`FLASH_BONDED_DEVICE_START`].
/// A matching address is not cryptographic authentication of a BLE central.
/// Flag bits:
/// - bit 0: active (record is in use)
/// - bit 1: paired (registration completed)
/// - bit 2: verified (local NFC tag ID was present/compared; not reader proof)
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
pub struct BondedDevice {
    /// Bluetooth MAC address of the bonded device (6 bytes, little-endian).
    pub mac: [u8; 6],
    /// Application-supplied Unix timestamp in seconds; firmware leaves it zero.
    pub timestamp: u32,
    /// Status flags (bit 0 = active, bit 1 = paired, bit 2 = verified).
    pub flags: u8,
}

impl BondedDevice {
    /// Create a new bonded device record with the active flag set.
    pub fn new(mac: [u8; 6]) -> Self {
        BondedDevice {
            mac,
            timestamp: 0,
            flags: 0b001, // active only
        }
    }

    /// Deserialise from an 11-byte flash record.
    pub fn from_bytes(bytes: &[u8; BONDED_DEVICE_STRUCT_SIZE]) -> Self {
        let mac = [bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5]];
        let timestamp = u32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]);
        let flags = bytes[10];
        BondedDevice {
            mac,
            timestamp,
            flags,
        }
    }

    /// Serialise to an 11-byte flash record.
    pub fn to_bytes(&self) -> [u8; BONDED_DEVICE_STRUCT_SIZE] {
        let mut buf = [0u8; BONDED_DEVICE_STRUCT_SIZE];
        buf[0..6].copy_from_slice(&self.mac);
        buf[6..10].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[10] = self.flags;
        buf
    }

    /// The record is in use (active flag set).
    pub fn is_active(&self) -> bool {
        self.flags & 0b001 != 0
    }

    /// The software record is marked registered.
    pub fn is_paired(&self) -> bool {
        self.flags & 0b010 != 0
    }

    /// A local tag-ID check was recorded; this is not reader authentication.
    pub fn is_verified(&self) -> bool {
        self.flags & 0b100 != 0
    }

    /// Set or clear the active flag (bit 0).
    pub fn set_active(&mut self, active: bool) {
        if active {
            self.flags |= 0b001;
        } else {
            self.flags &= !0b001;
        }
    }

    /// Set or clear the paired flag (bit 1).
    pub fn set_paired(&mut self, paired: bool) {
        if paired {
            self.flags |= 0b010;
        } else {
            self.flags &= !0b010;
        }
    }

    /// Set or clear the verified flag (bit 2).
    pub fn set_verified(&mut self, verified: bool) {
        if verified {
            self.flags |= 0b100;
        } else {
            self.flags &= !0b100;
        }
    }

    /// Return the stored timestamp.
    pub fn timestamp(&self) -> u32 {
        self.timestamp
    }

    /// Update the stored timestamp.
    pub fn set_timestamp(&mut self, timestamp: u32) {
        self.timestamp = timestamp;
    }
}

/// Encode a primary bonded-device record as magic + device bytes (12 bytes).
pub fn encode_primary(device: &BondedDevice) -> [u8; FLASH_RECORD_SIZE] {
    let mut record = [0; FLASH_RECORD_SIZE];
    record[0] = FLASH_RECORD_MAGIC;
    record[1..].copy_from_slice(&device.to_bytes());
    record
}

/// Decode a primary record, rejecting a missing magic byte or inactive record.
pub fn decode_primary(record: &[u8; FLASH_RECORD_SIZE]) -> Option<BondedDevice> {
    if record[0] != FLASH_RECORD_MAGIC {
        return None;
    }
    let bytes: &[u8; BONDED_DEVICE_STRUCT_SIZE] = record[1..].try_into().ok()?;
    let device = BondedDevice::from_bytes(bytes);
    device.is_active().then_some(device)
}

/// Encode the primary record followed by its 10-byte NFC UID (22 bytes).
pub fn encode_full(device: &BondedDevice, uid: &[u8; 10]) -> [u8; FULL_RECORD_SIZE] {
    let mut record = [0; FULL_RECORD_SIZE];
    record[..FLASH_RECORD_SIZE].copy_from_slice(&encode_primary(device));
    record[FLASH_RECORD_SIZE..].copy_from_slice(uid);
    record
}

/// Decode the 22-byte extended record, rejecting invalid/inactive primary data.
pub fn decode_full(record: &[u8; FULL_RECORD_SIZE]) -> Option<(BondedDevice, [u8; 10])> {
    let primary: &[u8; FLASH_RECORD_SIZE] = record[..FLASH_RECORD_SIZE].try_into().ok()?;
    let device = decode_primary(primary)?;
    let uid = record[FLASH_RECORD_SIZE..].try_into().ok()?;
    Some((device, uid))
}

/// Encode the extended record to the 24-byte, four-byte-aligned flash envelope.
///
/// If `uid` is `None`, the ten UID bytes are left erased (`0xFF`).
pub fn encode_full_padded(
    device: &BondedDevice,
    uid: Option<&[u8; 10]>,
) -> [u8; FULL_RECORD_PADDED_SIZE] {
    let mut record = [0xFF; FULL_RECORD_PADDED_SIZE];
    record[..FLASH_RECORD_SIZE].copy_from_slice(&encode_primary(device));
    if let Some(uid) = uid {
        record[FLASH_RECORD_SIZE..FULL_RECORD_SIZE].copy_from_slice(uid);
    }
    record
}

/// Decode the meaningful 22 bytes of a 24-byte flash envelope.
///
/// Trailing alignment bytes are ignored because they are not part of the record.
pub fn decode_full_padded(
    record: &[u8; FULL_RECORD_PADDED_SIZE],
) -> Option<(BondedDevice, [u8; 10])> {
    let full: &[u8; FULL_RECORD_SIZE] = record[..FULL_RECORD_SIZE].try_into().ok()?;
    decode_full(full)
}

/// Load the primary record from an explicit backend.
pub fn load_primary(
    backend: &impl BondedDeviceBackend,
) -> Result<Option<BondedDevice>, StorageError> {
    let mut record = [0xFF; FLASH_RECORD_SIZE];
    backend.read(FLASH_BONDED_DEVICE_START, &mut record)?;
    Ok(decode_primary(&record))
}

/// Load the extended record, interpreting erased UID bytes as absent (`0xFF`).
pub fn load_full(
    backend: &impl BondedDeviceBackend,
) -> Result<Option<(BondedDevice, [u8; 10])>, StorageError> {
    let mut record = [0xFF; FULL_RECORD_PADDED_SIZE];
    backend.read(FLASH_BONDED_DEVICE_START, &mut record)?;
    Ok(decode_full_padded(&record))
}

/// Replace the single stored device record after a completed erase and write.
///
/// The returned error distinguishes erase from write failure; an erase followed
/// by a failed write leaves the page erased and is not reported as atomic.
pub fn store_device(
    backend: &mut impl BondedDeviceBackend,
    device: &BondedDevice,
    uid: Option<&[u8; 10]>,
) -> Result<(), StorageError> {
    let record = AlignedFullRecord(encode_full_padded(device, uid));
    backend.erase_page(FLASH_BONDED_DEVICE_START)?;
    backend.write(FLASH_BONDED_DEVICE_START, &record.0)
}

/// Erase the stored device only when its MAC matches the requested address.
pub fn remove_device(
    backend: &mut impl BondedDeviceBackend,
    mac: &[u8; 6],
) -> Result<(), StorageError> {
    match load_primary(backend)? {
        Some(device) if &device.mac == mac => backend.erase_page(FLASH_BONDED_DEVICE_START),
        _ => Err(StorageError::NotFound),
    }
}

/// Erase the single bonded-device page.
pub fn clear_devices(backend: &mut impl BondedDeviceBackend) -> Result<(), StorageError> {
    backend.erase_page(FLASH_BONDED_DEVICE_START)
}

/// Persist a record using the same codec and completion rules as [`store_device`].
///
/// The buffer is four-byte aligned for S140. Errors after erase leave an erased
/// page; this operation is not power-loss atomic.
pub async fn store_device_async(
    backend: &mut impl AsyncBondedDeviceBackend,
    device: &BondedDevice,
    uid: Option<&[u8; 10]>,
) -> Result<(), StorageError> {
    let record = AlignedFullRecord(encode_full_padded(device, uid));
    backend.erase_page(FLASH_BONDED_DEVICE_START).await?;
    backend.write(FLASH_BONDED_DEVICE_START, &record.0).await
}

/// Await completed erasure of the single bond page, propagating failure.
pub async fn clear_devices_async(
    backend: &mut impl AsyncBondedDeviceBackend,
) -> Result<(), StorageError> {
    backend.erase_page(FLASH_BONDED_DEVICE_START).await
}
