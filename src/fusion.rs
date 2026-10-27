//! Six-axis Madgwick orientation filter and raw-unit conversions.

use crate::SensorData;

/// Q14 fixed-point scale factor (`16384 = 2^14`) for compact 8-byte BLE quaternion packets.
pub const QUAT_Q14_SCALE: f32 = 16384.0;
/// Default Madgwick filter gain `beta` (rad/s).
pub const DEFAULT_MADGWICK_BETA: f32 = 0.1;
/// Default sensor sampling frequency (20 Hz).
pub const DEFAULT_SAMPLE_FREQ_HZ: f32 = 20.0;
/// Gyroscope stationary deadband in rad/s; this is not a position estimator.
pub const GYRO_DEADBAND_RAD_S: f32 = 0.015;

/// Fast `no_std` square root using Newton-Raphson iterations on IEEE-754 initial guess.
#[inline]
pub fn fast_sqrt(x: f32) -> f32 {
    if x.is_nan() {
        return f32::NAN;
    }
    if x == f32::INFINITY {
        return f32::INFINITY;
    }
    if x <= 0.0 {
        return 0.0;
    }
    let inv = fast_inv_sqrt(x);
    x * inv
}

/// Fast `no_std` inverse square root (`1.0 / sqrt(x)`) with two Newton-Raphson refinement steps.
#[inline]
pub fn fast_inv_sqrt(x: f32) -> f32 {
    if !x.is_finite() || x <= 0.0 {
        return 0.0;
    }
    let xhalf = 0.5 * x;
    let i = x.to_bits();
    let i = 0x5f37_5a86 - (i >> 1);
    let mut y = f32::from_bits(i);
    y = y * (1.5 - xhalf * y * y);
    y = y * (1.5 - xhalf * y * y);
    y
}

/// 3D orientation quaternion `(w, x, y, z)`.
///
/// The filter uses the existing sensor-frame convention; six-axis inputs do
/// not observe absolute yaw, so yaw may drift over time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    /// Scalar component in `(w, x, y, z)` ordering.
    pub w: f32,
    /// X component in `(w, x, y, z)` ordering.
    pub x: f32,
    /// Y component in `(w, x, y, z)` ordering.
    pub y: f32,
    /// Z component in `(w, x, y, z)` ordering.
    pub z: f32,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quaternion {
    /// Identity quaternion `(1, 0, 0, 0)`.
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Construct a quaternion.
    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// Compute the Euclidean norm `sqrt(w^2 + x^2 + y^2 + z^2)`.
    pub fn norm(&self) -> f32 {
        fast_sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Return a normalized unit quaternion.
    pub fn normalized(&self) -> Self {
        let n_sq = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z;
        if !n_sq.is_finite() || n_sq <= 1.0e-12 {
            return Self::IDENTITY;
        }
        let inv = fast_inv_sqrt(n_sq);
        Self {
            w: self.w * inv,
            x: self.x * inv,
            y: self.y * inv,
            z: self.z * inv,
        }
    }

    /// Normalize and serialize as eight little-endian Q14 bytes (`w, x, y, z`).
    ///
    /// Values are rounded to the nearest integer (ties away from zero) and
    /// clamped to the representable normalized range `[-1.0, 1.0]`.
    pub fn to_q14_bytes(&self) -> [u8; 8] {
        let q = self.normalized();
        let clamp_q14 = |v: f32| -> i16 {
            let scaled = v * QUAT_Q14_SCALE;
            if scaled >= 16384.0 {
                16384
            } else if scaled <= -16384.0 {
                -16384
            } else if scaled >= 0.0 {
                (scaled + 0.5) as i16
            } else {
                (scaled - 0.5) as i16
            }
        };
        let w = clamp_q14(q.w).to_le_bytes();
        let x = clamp_q14(q.x).to_le_bytes();
        let y = clamp_q14(q.y).to_le_bytes();
        let z = clamp_q14(q.z).to_le_bytes();
        [w[0], w[1], x[0], x[1], y[0], y[1], z[0], z[1]]
    }

    /// Deserialize from an 8-byte little-endian Q14 fixed-point array.
    pub fn from_q14_bytes(bytes: &[u8; 8]) -> Self {
        let w = i16::from_le_bytes([bytes[0], bytes[1]]) as f32 / QUAT_Q14_SCALE;
        let x = i16::from_le_bytes([bytes[2], bytes[3]]) as f32 / QUAT_Q14_SCALE;
        let y = i16::from_le_bytes([bytes[4], bytes[5]]) as f32 / QUAT_Q14_SCALE;
        let z = i16::from_le_bytes([bytes[6], bytes[7]]) as f32 / QUAT_Q14_SCALE;
        Self { w, x, y, z }.normalized()
    }

    /// Compute gravity direction vector `(gx, gy, gz)` in the sensor frame predicted by this quaternion.
    pub fn gravity_vector(&self) -> (f32, f32, f32) {
        let gx = 2.0 * (self.x * self.z - self.w * self.y);
        let gy = 2.0 * (self.w * self.x + self.y * self.z);
        let gz = self.w * self.w - self.x * self.x - self.y * self.y + self.z * self.z;
        (gx, gy, gz)
    }
}

/// Convert an LSM6DS3TR accelerometer sample to `g` assuming ±2 g scale.
///
/// The conversion factor is 0.061 mg/LSB; it is only correct when that scale
/// is configured and is not inferred from [`SensorData`].
#[inline]
pub fn accel_lsb_to_g(lsb: i16) -> f32 {
    lsb as f32 * 0.000_061
}

/// Convert an LSM6DS3TR gyroscope sample to rad/s assuming ±245 dps scale.
///
/// The conversion factor is 8.75 mdps/LSB; it is only correct when that scale
/// is configured and is not inferred from [`SensorData`].
#[inline]
pub fn gyro_lsb_to_rad_s(lsb: i16) -> f32 {
    const DPS_TO_RAD: f32 = core::f32::consts::PI / 180.0;
    lsb as f32 * 0.008_75 * DPS_TO_RAD
}

/// Madgwick 6-DoF IMU AHRS orientation estimator.
#[derive(Clone, Copy, Debug)]
pub struct MadgwickAhrs {
    q: Quaternion,
    beta: f32,
    inv_sample_freq: f32,
}

impl Default for MadgwickAhrs {
    fn default() -> Self {
        Self::new(DEFAULT_SAMPLE_FREQ_HZ, DEFAULT_MADGWICK_BETA)
    }
}

impl MadgwickAhrs {
    /// Create a new Madgwick AHRS filter for `sample_freq_hz` and gradient gain `beta`.
    pub fn new(sample_freq_hz: f32, beta: f32) -> Self {
        let freq = if sample_freq_hz.is_finite() && sample_freq_hz > 0.0 {
            sample_freq_hz
        } else {
            DEFAULT_SAMPLE_FREQ_HZ
        };
        let beta = if beta.is_finite() && beta >= 0.0 {
            beta
        } else {
            DEFAULT_MADGWICK_BETA
        };
        Self {
            q: Quaternion::IDENTITY,
            beta,
            inv_sample_freq: 1.0 / freq,
        }
    }

    /// Return the current orientation quaternion.
    pub const fn quaternion(&self) -> Quaternion {
        self.q
    }

    /// Reset orientation to the identity quaternion.
    pub fn reset(&mut self) {
        self.q = Quaternion::IDENTITY;
    }

    /// Step the filter using a raw [`SensorData`] sample.
    pub fn update_from_sensor_data(&mut self, data: &SensorData) -> Quaternion {
        self.update_from_sensor_data_with_dt(data, self.inv_sample_freq)
    }

    /// Step the filter using a raw [`SensorData`] sample and actual elapsed time.
    ///
    /// `dt_seconds` must be positive and finite. Invalid intervals leave the
    /// filter unchanged and return the current quaternion.
    pub fn update_from_sensor_data_with_dt(
        &mut self,
        data: &SensorData,
        dt_seconds: f32,
    ) -> Quaternion {
        if !dt_seconds.is_finite() || dt_seconds <= 0.0 {
            return self.q;
        }
        let gx = gyro_lsb_to_rad_s(data.gyro_x);
        let gy = gyro_lsb_to_rad_s(data.gyro_y);
        let gz = gyro_lsb_to_rad_s(data.gyro_z);
        let ax = accel_lsb_to_g(data.accel_x);
        let ay = accel_lsb_to_g(data.accel_y);
        let az = accel_lsb_to_g(data.accel_z);
        let configured_inverse = self.inv_sample_freq;
        self.inv_sample_freq = dt_seconds;
        let result = self.update_imu(gx, gy, gz, ax, ay, az);
        self.inv_sample_freq = configured_inverse;
        result
    }

    /// Step the 6-DoF Madgwick IMU filter (`gx, gy, gz` in rad/s, `ax, ay, az` in any consistent unit).
    pub fn update_imu(
        &mut self,
        mut gx: f32,
        mut gy: f32,
        mut gz: f32,
        ax: f32,
        ay: f32,
        az: f32,
    ) -> Quaternion {
        if !gx.is_finite()
            || !gy.is_finite()
            || !gz.is_finite()
            || !ax.is_finite()
            || !ay.is_finite()
            || !az.is_finite()
        {
            return self.q;
        }

        // Apply a gyro deadband to suppress tiny rates.
        if gx.abs() < GYRO_DEADBAND_RAD_S {
            gx = 0.0;
        }
        if gy.abs() < GYRO_DEADBAND_RAD_S {
            gy = 0.0;
        }
        if gz.abs() < GYRO_DEADBAND_RAD_S {
            gz = 0.0;
        }

        let mut q0 = self.q.w;
        let mut q1 = self.q.x;
        let mut q2 = self.q.y;
        let mut q3 = self.q.z;

        // Rate of change of quaternion from gyroscope
        let mut q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        let mut q_dot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        let mut q_dot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        let mut q_dot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement is non-zero
        let accel_norm_sq = ax * ax + ay * ay + az * az;
        if accel_norm_sq > 1.0e-12 {
            let recip_norm = fast_inv_sqrt(accel_norm_sq);
            let ax = ax * recip_norm;
            let ay = ay * recip_norm;
            let az = az * recip_norm;

            // Auxiliary variables to avoid repeated arithmetic
            let _2q0 = 2.0 * q0;
            let _2q1 = 2.0 * q1;
            let _2q2 = 2.0 * q2;
            let _2q3 = 2.0 * q3;
            let _4q0 = 4.0 * q0;
            let _4q1 = 4.0 * q1;
            let _4q2 = 4.0 * q2;
            let _8q1 = 8.0 * q1;
            let _8q2 = 8.0 * q2;
            let q0q0 = q0 * q0;
            let q1q1 = q1 * q1;
            let q2q2 = q2 * q2;
            let q3q3 = q3 * q3;

            // Gradient descent corrective step
            let s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            let s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1
                + _8q1 * q1q1
                + _8q1 * q2q2
                + _4q1 * az;
            let s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
                + _8q2 * q1q1
                + _8q2 * q2q2
                + _4q2 * az;
            let s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

            let s_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
            if s_norm_sq > 1.0e-12 {
                let recip_s = fast_inv_sqrt(s_norm_sq);
                q_dot1 -= self.beta * s0 * recip_s;
                q_dot2 -= self.beta * s1 * recip_s;
                q_dot3 -= self.beta * s2 * recip_s;
                q_dot4 -= self.beta * s3 * recip_s;
            }
        }

        // Integrate rate of change of quaternion
        q0 += q_dot1 * self.inv_sample_freq;
        q1 += q_dot2 * self.inv_sample_freq;
        q2 += q_dot3 * self.inv_sample_freq;
        q3 += q_dot4 * self.inv_sample_freq;

        self.q = Quaternion::new(q0, q1, q2, q3).normalized();
        self.q
    }
}
