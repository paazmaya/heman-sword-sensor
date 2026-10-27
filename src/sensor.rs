//! Raw sensor sample types, wire codec, and input-domain validation.

// ============================================================================
// SENSOR DATA STRUCTURE
// ============================================================================

/// Sensor-frame accelerometer and gyroscope packet for BLE transmission (12 bytes).
///
/// Each field is a native-endian `i16` raw count; its scale depends on the IMU
/// configuration. Use [`SensorData::to_bytes`] for the little-endian BLE format,
/// rather than relying on in-memory layout or target byte order.
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorData {
    /// X-axis accelerometer sample in uncalibrated hardware raw counts.
    pub accel_x: i16,
    /// Y-axis accelerometer sample in uncalibrated hardware raw counts.
    pub accel_y: i16,
    /// Z-axis accelerometer sample in uncalibrated hardware raw counts.
    pub accel_z: i16,
    /// X-axis gyroscope sample in uncalibrated hardware raw counts.
    pub gyro_x: i16,
    /// Y-axis gyroscope sample in uncalibrated hardware raw counts.
    pub gyro_y: i16,
    /// Z-axis gyroscope sample in uncalibrated hardware raw counts.
    pub gyro_z: i16,
}

impl SensorData {
    /// Create a new SensorData instance.
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

    /// Get the byte size of this struct (always 12).
    pub fn size() -> usize {
        core::mem::size_of::<SensorData>()
    }

    /// Serialize to a 12-byte array suitable for a BLE characteristic value.
    ///
    /// ```
    /// use xiao_nrf52840_sword::SensorData;
    /// let sample = SensorData::new(0x1234, -2, 0, 0, 0, 0);
    /// let bytes = sample.to_bytes();
    /// assert_eq!(&bytes[..4], &[0x34, 0x12, 0xFE, 0xFF]);
    /// assert_eq!(SensorData::from_bytes(&bytes), sample);
    /// ```
    pub fn to_bytes(&self) -> [u8; 12] {
        let mut buf = [0u8; 12];
        buf[0..2].copy_from_slice(&self.accel_x.to_le_bytes());
        buf[2..4].copy_from_slice(&self.accel_y.to_le_bytes());
        buf[4..6].copy_from_slice(&self.accel_z.to_le_bytes());
        buf[6..8].copy_from_slice(&self.gyro_x.to_le_bytes());
        buf[8..10].copy_from_slice(&self.gyro_y.to_le_bytes());
        buf[10..12].copy_from_slice(&self.gyro_z.to_le_bytes());
        buf
    }

    /// Deserialize from a 12-byte array.
    pub fn from_bytes(buf: &[u8; 12]) -> Self {
        SensorData {
            accel_x: i16::from_le_bytes([buf[0], buf[1]]),
            accel_y: i16::from_le_bytes([buf[2], buf[3]]),
            accel_z: i16::from_le_bytes([buf[4], buf[5]]),
            gyro_x: i16::from_le_bytes([buf[6], buf[7]]),
            gyro_y: i16::from_le_bytes([buf[8], buf[9]]),
            gyro_z: i16::from_le_bytes([buf[10], buf[11]]),
        }
    }
}

/// Validate a sensor sample against the input representation domain.
///
/// Every combination of `i16` values is a representable hardware sample, so
/// this currently accepts all inputs. Scale-specific plausibility checks
/// belong in the adapter that knows the configured IMU range.
pub const fn validate_sensor_data(_data: &SensorData) -> bool {
    true
}
