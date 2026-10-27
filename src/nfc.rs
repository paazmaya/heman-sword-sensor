//! Host-testable NFC UID parsing and a RAM-only Type 2 page shadow.

/// Last known local tag-selection state, not a remote reader identity.
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NfcFieldState {
    /// No NFC field detected.
    Idle,
    /// Actively scanning for a field.
    Scanning,
    /// An external reader selected this device's tag.
    Detected,
    /// Hardware or protocol error.
    Error,
}

/// Return the current NFC field state.
///
/// On the embedded target this tracks the last value stored by
/// the embedded UID store. On the host it always returns `Idle`.
pub fn get_field_state() -> NfcFieldState {
    #[cfg(feature = "embedded")]
    {
        if peek_activated_uid().is_some() {
            NfcFieldState::Detected
        } else {
            NfcFieldState::Idle
        }
    }
    #[cfg(not(feature = "embedded"))]
    NfcFieldState::Idle
}

// ------------------------------------------------------------------
// UID validation helpers (no hardware dependency)
// ------------------------------------------------------------------

/// Check that a 10-byte UID buffer is not the all-zero placeholder.
///
/// This deliberately does not claim that a byte pattern proves tag
/// authenticity or identifies a remote BLE central.
pub fn is_valid_uid(uid: &[u8; 10]) -> bool {
    uid.iter().any(|&byte| byte != 0)
}

/// Check that a 7-byte UID buffer is not the all-zero placeholder.
pub fn is_valid_uid_7(uid: &[u8; 7]) -> bool {
    uid.iter().any(|&byte| byte != 0)
}

// ------------------------------------------------------------------
// UID extraction helpers (pure logic — testable on host)
// ------------------------------------------------------------------

/// Canonicalize a raw local-tag UID byte slice to ten bytes.
///
/// Hardware identifiers use 4, 7, or 10 bytes, according to the configured
/// `NfcId` size. This compatibility helper also accepts other nonempty lengths:
/// it zero-pads short inputs and truncates inputs longer than ten bytes.
///
/// | `NfcId` size   | Raw bytes | Resulting UID                         |
/// |----------------|-----------|---------------------------------------|
/// | `SingleSize`   | 4         | Padded to 10 bytes (bytes 4–9 = 0x00) |
/// | `DoubleSize`   | 7         | Padded to 10 bytes (bytes 7–9 = 0x00) |
/// | `TripleSize`   | 10        | Used as-is                            |
///
/// Returns `None` when the raw slice is empty or all-zero.
pub fn extract_uid_from_nfct(raw: &[u8]) -> Option<[u8; 10]> {
    if raw.is_empty() {
        return None;
    }
    let mut uid = [0u8; 10];
    let copy_len = raw.len().min(10);
    uid[..copy_len].copy_from_slice(&raw[..copy_len]);
    if uid.iter().all(|&b| b == 0) {
        None
    } else {
        Some(uid)
    }
}

/// Parse raw UID bytes and validate that the result is nonzero.
///
/// This is a thin convenience wrapper around [`extract_uid_from_nfct`]
/// that additionally rejects an all-zero placeholder. This is format
/// checking only, not authentication.
///
/// Returns `None` when the UID is missing or its first ten bytes are all zero.
/// No manufacturer-byte or input-length restriction is imposed.
pub fn parse_nfct_uid(raw: &[u8]) -> Option<[u8; 10]> {
    let uid = extract_uid_from_nfct(raw)?;

    // For 7-byte source data, validate via the 7-byte helper
    if raw.len() == 7 {
        let mut short = [0u8; 7];
        short.copy_from_slice(&raw[..7]);
        if !is_valid_uid_7(&short) {
            return None;
        }
    } else if !is_valid_uid(&uid) {
        return None;
    }

    Some(uid)
}

// ------------------------------------------------------------------
// NFC UID shared state
//
// After `NfcT::activate()` completes in main.rs, the firmware calls
// `store_activated_uid()` so that the library-level pairing logic
// (pairing_mode(), authenticate_with_uid(), …) can access the real
// tag UID without receiving it as a function argument.
//
// On the host (desktop tests) the static is never written, so
// peek_activated_uid() always returns None which is fine.
// ------------------------------------------------------------------

/// Store the UID of the most-recently activated NFC tag.
///
/// Call this after the local NFCT tag is activated, passing the local
/// tag UID bytes. The NFCT activation API does not provide a BLE central's
/// identity or authenticate that central.
/// Internally calls [`parse_nfct_uid`] to validate and canonicalise
/// to 10 bytes before storing.
///
/// Passing an empty or all-zero slice clears the stored UID.
#[cfg(feature = "embedded")]
pub fn store_activated_uid(raw: &[u8]) {
    let uid = parse_nfct_uid(raw);
    ACTIVATED_UID.lock(|cell| cell.set(uid));
    match uid {
        Some(u) => defmt::info!("🏷️  NFC UID stored: {:02x}", u),
        None => defmt::warn!("⚠️  NFC UID invalid or empty – cleared"),
    }
}

/// Return the UID stored by the most recent successful NFC activation,
/// or `None` if no valid tag has been activated since boot.
#[cfg(feature = "embedded")]
pub fn peek_activated_uid() -> Option<[u8; 10]> {
    ACTIVATED_UID.lock(|cell| cell.get())
}

/// Clear the stored NFC UID (e.g. after pairing completes).
#[cfg(feature = "embedded")]
pub fn clear_activated_uid() {
    ACTIVATED_UID.lock(|cell| cell.set(None));
}

// Safety: only accessed through the CriticalSectionMutex lock.
#[cfg(feature = "embedded")]
static ACTIVATED_UID: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::Cell<Option<[u8; 10]>>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new(None));

// ------------------------------------------------------------------
// Async helpers — only compiled when the embassy runtime is present
// ------------------------------------------------------------------

/// Detect NFC field presence with a custom timeout.
///
/// Polls [`peek_activated_uid`] every 100 ms.  The actual NFCT
/// hardware activation (`NfcT::activate()`) is initiated in
/// `main.rs`; once it completes the result is deposited via
/// [`store_activated_uid`] and this function returns `true`.
///
/// Returns `false` when `timeout_ms` elapses without a UID being stored.
#[cfg(feature = "embedded")]
pub async fn detect_field_with_timeout(timeout_ms: u64) -> bool {
    use embassy_time::{Duration, Instant, Timer};
    defmt::info!("📡 NFC field detection – timeout {} ms", timeout_ms);
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    loop {
        if peek_activated_uid().is_some() {
            defmt::info!("✅ NFC field detected (UID present)");
            return true;
        }
        if Instant::now() >= deadline {
            defmt::info!("⏱️  NFC detect_field timeout");
            return false;
        }
        Timer::after_millis(100).await;
    }
}

/// Detect NFC field with the default 500 ms poll timeout.
#[cfg(feature = "embedded")]
pub async fn detect_field() -> bool {
    detect_field_with_timeout(500).await
}

/// Minimum writable user-memory page on an ISO/IEC 14443 Type 2 tag.
pub const NFC_MIN_USER_PAGE: u8 = 4;
/// Maximum writable user-memory page supported by our tag shadow buffer (NTAG213 range).
pub const NFC_MAX_USER_PAGE: u8 = 39;
/// Total number of 4-byte user pages tracked in the shadow buffer.
pub const NFC_USER_PAGE_COUNT: usize = (NFC_MAX_USER_PAGE - NFC_MIN_USER_PAGE + 1) as usize;

/// Mutable RAM shadow of the supported Type 2 user pages.
///
/// A fresh value provides isolated state for independent callers and tests;
/// these writes do not reach physical NFC tag memory.
pub struct NfcPageShadow {
    pages: [core::sync::atomic::AtomicU32; NFC_USER_PAGE_COUNT],
}

impl NfcPageShadow {
    /// Create an empty, independently owned page shadow.
    pub const fn new() -> Self {
        Self {
            pages: [const { core::sync::atomic::AtomicU32::new(0) }; NFC_USER_PAGE_COUNT],
        }
    }

    /// Store four bytes in a supported page, returning false for reserved pages.
    pub fn write(&self, page: u8, data: &[u8; 4]) -> bool {
        if !is_valid_nfc_user_page(page) {
            return false;
        }
        let idx = (page - NFC_MIN_USER_PAGE) as usize;
        self.pages[idx].store(
            u32::from_le_bytes(*data),
            core::sync::atomic::Ordering::SeqCst,
        );
        true
    }

    /// Read four bytes from a supported page, or None for reserved pages.
    pub fn read(&self, page: u8) -> Option<[u8; 4]> {
        if !is_valid_nfc_user_page(page) {
            return None;
        }
        let idx = (page - NFC_MIN_USER_PAGE) as usize;
        Some(
            self.pages[idx]
                .load(core::sync::atomic::Ordering::SeqCst)
                .to_le_bytes(),
        )
    }
}

impl Default for NfcPageShadow {
    fn default() -> Self {
        Self::new()
    }
}

static NFC_PAGE_SHADOW: NfcPageShadow = NfcPageShadow::new();

/// Check whether `page` is within the writable Type 2 Tag user memory region (pages 4–39).
pub const fn is_valid_nfc_user_page(page: u8) -> bool {
    page >= NFC_MIN_USER_PAGE && page <= NFC_MAX_USER_PAGE
}

/// Synchronously store four bytes in the RAM shadow for a Type 2 user page.
///
/// This does not write physical NFC tag memory. Returns `false` when `page` is
/// outside `[NFC_MIN_USER_PAGE..=NFC_MAX_USER_PAGE]`.
pub fn write_nfc_page_sync(page: u8, data: &[u8; 4]) -> bool {
    NFC_PAGE_SHADOW.write(page, data)
}

/// Read back 4 bytes from a Type 2 tag user page shadow buffer (pages 4–39).
pub fn read_nfc_page(page: u8) -> Option<[u8; 4]> {
    NFC_PAGE_SHADOW.read(page)
}

/// Update the RAM shadow for a Type 2 tag page (pages 4–39).
///
/// Type 2 tags are organised in 4-byte pages.  User memory starts at
/// page 4 (byte address 16) and extends to page 15 (bytes 60–63) on
/// a standard 64-byte MIFARE Ultralight tag, or up to page 39 on NTAG213.
/// This helper does not communicate with or write an actual tag.
#[cfg(feature = "embedded")]
pub async fn write_nfc_page(page: u8, data: &[u8; 4]) -> bool {
    if !write_nfc_page_sync(page, data) {
        defmt::warn!("⚠️  NFC WRITE rejected: invalid user page {}", page);
        return false;
    }
    defmt::info!("📡 NFC WRITE page {} data {:02x}", page, data);
    true
}
