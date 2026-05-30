//! Pure logic module for He-Man Sword Sensor
//! 
//! This module contains testable functions that don't depend on embedded hardware.
//! All embedded-specific code remains in main.rs.

#![cfg_attr(not(test), no_std)]

// Conditional imports for embedded features
#[cfg(feature = "embedded")]
use defmt::{info, warn};

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

pub const THRUST_THRESHOLD: i16 = 50;  // Threshold for upward thrust detection (~0.5g)
pub const NFC_PAIRING_TIMEOUT_SECS: u64 = 15;
pub const SENSOR_SAMPLING_INTERVAL_MS: u64 = 50;  // 20 Hz

// ============================================================================
// SENSOR DATA STRUCTURE
// ============================================================================

/// Sensor data packet for BLE transmission (12 bytes)
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorData {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
}

impl SensorData {
    /// Create a new SensorData instance
    pub fn new(
        accel_x: i16,
        accel_y: i16,
        accel_z: i16,
        gyro_x: i16,
        gyro_y: i16,
        gyro_z: i16,
    ) -> Self {
        SensorData {
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
        }
    }

    /// Get the size of this struct (should be exactly 12 bytes)
    pub fn size() -> usize {
        core::mem::size_of::<SensorData>()
    }
}

// ============================================================================
// MOTION DETECTION
// ============================================================================

/// Detect upward thrust based on Z-axis acceleration
/// Returns true if acceleration exceeds THRUST_THRESHOLD
pub fn detect_upward_thrust(accel_z: i16) -> bool {
    accel_z > THRUST_THRESHOLD
}

/// Classify motion based on acceleration magnitude
pub fn classify_motion(accel_x: i16, accel_y: i16, accel_z: i16) -> MotionType {
    let magnitude_sq = (accel_x as i32).pow(2)
        + (accel_y as i32).pow(2)
        + (accel_z as i32).pow(2);

    if detect_upward_thrust(accel_z) {
        MotionType::UpwardThrust
    } else if magnitude_sq >= 10000 {
        MotionType::Intense
    } else if magnitude_sq >= 900 {
        MotionType::Moderate
    } else {
        MotionType::Idle
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionType {
    Idle,
    Moderate,
    Intense,
    UpwardThrust,
}

// ============================================================================
// VALIDATION FUNCTIONS
// ============================================================================

/// Validate sensor data for reasonable values
/// Returns true if data is within expected ranges
pub fn validate_sensor_data(data: &SensorData) -> bool {
    // Accelerometer: typically ±16g, ±2g = ±20480 in raw values, so ±5000 is safe
    let accel_valid = data.accel_x.abs() < 10000
        && data.accel_y.abs() < 10000
        && data.accel_z.abs() < 10000;

    // Gyroscope: typically ±2000 dps, raw value limit is ±32767
    let gyro_valid = data.gyro_x.abs() < 20000
        && data.gyro_y.abs() < 20000
        && data.gyro_z.abs() < 20000;

    accel_valid && gyro_valid
}

// ============================================================================
// NFC PAIRING MODULE
// ============================================================================

/// NFC Pairing Status
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NfcPairingStatus {
    /// NFC pairing is idle, waiting for field
    Idle,
    /// NFC field is being scanned
    Scanning,
    /// NFC device is being authenticated
    Authenticating,
    /// NFC pairing completed successfully
    Success,
    /// NFC pairing failed
    Failed,
    /// NFC pairing timed out
    Timeout,
}

/// NFC Bonded Device Storage (6 bytes for MAC address)
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BondedDevice {
    /// MAC address of the bonded device (6 bytes)
    pub mac: [u8; 6],
    /// Timestamp of the last pairing (Unix timestamp)
    pub timestamp: u32,
    /// Flags: bit 0 = active, bit 1 = paired, bit 2 = verified
    pub flags: u8,
}

impl BondedDevice {
    /// Create a new bonded device with default values
    pub fn new(mac: [u8; 6]) -> Self {
        BondedDevice {
            mac,
            timestamp: 0,
            flags: 0b001, // active only
        }
    }

    /// Check if the device is active
    pub fn is_active(&self) -> bool {
        self.flags & 0b001 != 0
    }

    /// Check if the device is paired
    pub fn is_paired(&self) -> bool {
        self.flags & 0b010 != 0
    }

    /// Check if the device is verified
    pub fn is_verified(&self) -> bool {
        self.flags & 0b100 != 0
    }

    /// Set the active flag
    pub fn set_active(&mut self, active: bool) {
        self.flags |= active as u8;
    }

    /// Set the paired flag
    pub fn set_paired(&mut self, paired: bool) {
        self.flags |= (paired as u8) << 1;
    }

    /// Set the verified flag
    pub fn set_verified(&mut self, verified: bool) {
        self.flags |= (verified as u8) << 2;
    }

    /// Get the current timestamp
    pub fn timestamp(&self) -> u32 {
        self.timestamp
    }

    /// Set the timestamp
    pub fn set_timestamp(&mut self, timestamp: u32) {
        self.timestamp = timestamp;
    }
}

/// NFC Field Detection
/// 
/// This module provides NFC field detection functionality for the nRF52840.
/// In production, this would use the actual NFCT (NFC Type 2 Tag) peripheral.
#[cfg(feature = "embedded")]
pub mod nfc {
    use super::*;
    use embassy_time::Timer;

    /// NFC Field Detection State
    #[cfg_attr(feature = "embedded", derive(defmt::Format))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum NfcFieldState {
        /// No NFC field detected
        Idle,
        /// Scanning for NFC field
        Scanning,
        /// NFC field detected
        Detected,
        /// Error occurred
        Error,
    }

    /// Detect NFC field presence
    /// 
    /// In a real implementation, this would:
    /// 1. Initialize the NFCT peripheral
    /// 2. Configure it to listen for passive tags
    /// 3. Read the NFC UID from the tag
    /// 4. Return true if a valid tag is detected
    pub async fn detect_field() -> bool {
        info!("📡 NFC Field Detection - Scanning for NFC tags...");
        
        // Simulate NFC field detection
        // In production, this would use the actual NFCT peripheral
        Timer::after_millis(100).await;
        true
    }

    /// Read NFC field UID
    /// 
    /// Returns the UID of the detected NFC tag
    pub async fn read_nfc_uid() -> Option<[u8; 10]> {
        info!("📡 Reading NFC UID...");
        
        // Simulated NFC UID (10 bytes for Type 2 Tag)
        let nfc_uid = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        
        info!("✅ NFC UID: {:?}", nfc_uid);
        Some(nfc_uid)
    }

    /// Get current NFC field state
    pub fn get_field_state() -> NfcFieldState {
        NfcFieldState::Idle
    }

    /// Set NFC field state
    pub fn set_field_state(state: NfcFieldState) {
        info!("NFC Field State: {:?}", state);
    }
}

/// NFC Pairing Functions
/// 
/// These functions handle the complete NFC pairing flow:
/// 1. Detect NFC field
/// 2. Read bonded device MAC from flash
/// 3. Authenticate the device
/// 4. Establish secure connection
#[cfg(feature = "embedded")]
pub mod pairing {
    use super::*;
    use embassy_time::{Duration, Instant, Timer};

    /// NFC Pairing Mode: wait for NFC field or timeout
    /// 
    /// This function implements the primary pairing gate:
    /// 1. Wait for NFC field presence (up to 15 seconds)
    /// 2. If NFC detected, read bonded MAC and authenticate
    /// 3. If timeout, fall back to Bluetooth mode
    /// 
    /// Returns true if NFC pairing successful, false if timeout
    pub async fn pairing_mode() -> bool {
        info!("═══════════════════════════════════════════");
        info!("🔌 NFC Pairing Mode - Primary Authentication Gate");
        info!("═══════════════════════════════════════════");
        info!("   Timeout in {} seconds...", NFC_PAIRING_TIMEOUT_SECS);
        info!("");

        let pairing_start = Instant::now();
        let pairing_timeout = Duration::from_secs(NFC_PAIRING_TIMEOUT_SECS);

        loop {
            // Check for NFC field presence
            if nfc::detect_field().await {
                info!("✅ NFC field detected - initiating pairing sequence...");
                
                // Read bonded device MAC from flash
                if let Some(bonded_mac) = read_bonded_device_mac() {
                    info!("🔑 Bonded device MAC: {:?}", bonded_mac);
                    
                    // In a real implementation, we would:
                    // 1. Send the MAC to the paired mobile app
                    // 2. Verify the MAC against stored credentials
                    // 3. Establish secure BLE connection
                    // 4. Store the pairing timestamp and flags
                    
                    info!("✅ Bonded device authentication successful");
                    return true;
                } else {
                    warn!("⚠️  No bonded device MAC found in flash");
                    info!("   Pairing failed - no bonded device registered");
                    return false;
                }
            }

            // Check for timeout
            if pairing_start.elapsed() > pairing_timeout {
                info!("⏱️  NFC pairing timeout - switching to Bluetooth mode");
                info!("   Fallback to legacy BLE advertising");
                return false;
            }

            // Wait before next check
            Timer::after_millis(100).await;
        }
    }

    /// Read bonded device MAC from flash memory
    /// 
    /// The nRF52840 has 512 KB of flash memory. We'll store the bonded
    /// device MAC address in a dedicated region of flash.
    pub fn read_bonded_device_mac() -> Option<[u8; 6]> {
        info!("🔑 Reading bonded device MAC from flash...");
        
        // In a real implementation, we would:
        // 1. Use the nRF52840's flash controller to read from flash memory
        // 2. Read the MAC address from the bonded device storage region
        // 3. Validate the data (check timestamp, flags, etc.)
        // 4. Return the MAC address if valid
        
        // For now, we'll simulate reading a bonded MAC from flash
        // In production, this would use the actual flash controller
        
        // Simulated bonded MAC (would be read from flash in real hardware)
        let bonded_mac = [0x00, 11, 22, 33, 44, 55];
        
        info!("✅ Bonded MAC read: {:?}", bonded_mac);
        Some(bonded_mac)
    }

    /// Authenticate bonded device
    /// 
    /// Verifies the MAC address against stored credentials
    pub fn authenticate_bonded_device(_mac: &[u8; 6]) -> bool {
        info!("🔐 Authenticating bonded device...");
        
        // In a real implementation, we would:
        // 1. Compare the provided MAC with the stored MAC
        // 2. Verify the timestamp is recent
        // 3. Check the device flags
        // 4. Return true if authentication succeeds
        
        // For now, we'll simulate successful authentication
        info!("✅ Authentication successful");
        true
    }

    /// Get current pairing status
    pub fn get_pairing_status() -> NfcPairingStatus {
        NfcPairingStatus::Idle
    }

    /// Set pairing status
    pub fn set_pairing_status(status: NfcPairingStatus) {
        info!("NFC Status: {:?}", status);
    }

    /// Write bonded device MAC to flash memory
    /// 
    /// Stores the MAC address in the bonded device storage region
    /// Flash offset: 0x2000 (8 KB region)
    pub fn write_bonded_device_to_flash(_mac: &[u8; 6]) -> bool {
        info!("💾 Writing bonded device MAC to flash...");
        
        // In a real implementation, we would:
        // 1. Use the nRF52840's flash controller to write to flash memory
        // 2. Write the MAC address at offset 0x2000
        // 3. Write timestamp at offset 0x2006
        // 4. Write flags at offset 0x200C
        // 5. Verify the write was successful
        
        // For now, we'll simulate successful flash write
        info!("✅ Bonded device MAC written to flash");
        true
    }

    /// Register new bonded device
    /// 
    /// Adds a new MAC address to the bonded device list
    pub fn register_bonded_device(mac: &[u8; 6]) -> bool {
        info!("📝 Registering new bonded device...");
        
        // In a real implementation, we would:
        // 1. Read existing bonded devices from flash
        // 2. Check if MAC already exists
        // 3. If new, write to flash storage
        // 4. Return success
        
        // For now, we'll simulate successful registration
        info!("✅ New bonded device registered: {:?}", mac);
        true
    }

    /// Unregister bonded device
    /// 
    /// Removes a MAC address from the bonded device list
    pub fn unregister_bonded_device(mac: &[u8; 6]) -> bool {
        info!("🗑️  Unregistering bonded device...");
        
        // In a real implementation, we would:
        // 1. Read existing bonded devices from flash
        // 2. Remove the specified MAC
        // 3. Write updated list to flash
        // 4. Return success
        
        // For now, we'll simulate successful unregistration
        info!("✅ Bonded device unregistered: {:?}", mac);
        true
    }

    /// Get all bonded devices
    /// 
    /// Returns a list of all registered bonded devices
    pub fn get_bonded_devices() -> [BondedDevice; 2] {
        info!("📋 Reading bonded devices from flash...");
        
        // In a real implementation, we would:
        // 1. Read all bonded devices from flash
        // 2. Return the list
        
        // For now, we'll return a simulated list
        [
            BondedDevice::new([0x00, 11, 22, 33, 44, 55]),
            BondedDevice::new([0x00, 12, 23, 34, 45, 56]),
        ]
    }

    /// Check if MAC is in bonded device list
    pub fn is_bonded_device(mac: &[u8; 6]) -> bool {
        let devices = get_bonded_devices();
        devices.iter().any(|d| d.mac == *mac)
    }
}


