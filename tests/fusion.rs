mod madgwick_fusion_tests {
    use xiao_nrf52840_sword::fusion::{fast_inv_sqrt, fast_sqrt, MadgwickAhrs, Quaternion};
    use xiao_nrf52840_sword::SensorData;

    #[test]
    fn test_quaternion_identity_and_normalization() {
        let q = Quaternion::IDENTITY;
        assert!((q.norm() - 1.0).abs() < 1.0e-3);

        let unnorm = Quaternion::new(2.0, 2.0, 2.0, 2.0).normalized();
        assert!((unnorm.norm() - 1.0).abs() < 1.0e-3);
        assert!((unnorm.w - 0.5).abs() < 1.0e-2);
    }

    #[test]
    fn test_quaternion_q14_bytes_round_trip() {
        let half_rotation = core::f32::consts::FRAC_1_SQRT_2;
        let orig = Quaternion::new(half_rotation, 0.0, half_rotation, 0.0).normalized();
        let bytes = orig.to_q14_bytes();
        assert_eq!(bytes.len(), 8);
        let restored = Quaternion::from_q14_bytes(&bytes);
        assert!((orig.w - restored.w).abs() < 1.0e-3);
        assert!((orig.x - restored.x).abs() < 1.0e-3);
        assert!((orig.y - restored.y).abs() < 1.0e-3);
        assert!((orig.z - restored.z).abs() < 1.0e-3);
    }

    #[test]
    fn test_madgwick_stationary_z_gravity_preserves_upright_orientation() {
        let mut ahrs = MadgwickAhrs::new(20.0, 0.1);
        for _ in 0..100 {
            ahrs.update_imu(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        }
        let q = ahrs.quaternion();
        assert!((q.norm() - 1.0).abs() < 1.0e-2);
        let (gx, gy, gz) = q.gravity_vector();
        assert!(gx.abs() < 0.05);
        assert!(gy.abs() < 0.05);
        assert!(gz > 0.98);
    }

    #[test]
    fn test_madgwick_converges_toward_tilted_x_gravity() {
        let mut ahrs = MadgwickAhrs::new(50.0, 0.5);
        for _ in 0..250 {
            ahrs.update_imu(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        }
        let (gx, gy, gz) = ahrs.quaternion().gravity_vector();
        assert!(gx > 0.85, "expected gx > 0.85, got {}", gx);
        assert!(gy.abs() < 0.1);
        assert!(gz.abs() < 0.5);
    }

    #[test]
    fn test_madgwick_yaw_gyro_integration_and_sensor_data_helper() {
        let mut ahrs = MadgwickAhrs::default();
        // ~90 deg/s yaw rotation (1.57 rad/s) for 10 steps at 20 Hz
        for _ in 0..10 {
            ahrs.update_imu(0.0, 0.0, 1.57, 0.0, 0.0, 1.0);
        }
        let q = ahrs.quaternion();
        assert!(q.z > 0.25);
        assert!((q.norm() - 1.0).abs() < 1.0e-2);

        // Verify raw SensorData helper runs cleanly
        let raw = SensorData::new(0, 0, 16384, 0, 0, 1000);
        let q2 = ahrs.update_from_sensor_data(&raw);
        assert!((q2.norm() - 1.0).abs() < 1.0e-2);
    }

    #[test]
    fn fast_math_handles_nonpositive_and_nonfinite_values() {
        assert_eq!(fast_sqrt(0.0), 0.0);
        assert_eq!(fast_sqrt(-1.0), 0.0);
        assert!(fast_sqrt(f32::NAN).is_nan());
        assert_eq!(fast_sqrt(f32::INFINITY), f32::INFINITY);
        assert_eq!(fast_inv_sqrt(f32::INFINITY), 0.0);
        assert_eq!(fast_inv_sqrt(-1.0), 0.0);
        assert!((fast_sqrt(4.0) - 2.0).abs() < 0.01);
    }

    #[test]
    fn q14_codec_has_literal_identity_and_negative_one_fixtures() {
        assert_eq!(
            Quaternion::IDENTITY.to_q14_bytes(),
            [0x00, 0x40, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            Quaternion::new(-1.0, 0.0, 0.0, 0.0).to_q14_bytes(),
            [0x00, 0xC0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            Quaternion::new(0.0, 0.0, 0.0, 0.0).normalized(),
            Quaternion::IDENTITY
        );
        assert_eq!(
            Quaternion::new(f32::INFINITY, 0.0, 0.0, 0.0).normalized(),
            Quaternion::IDENTITY
        );
    }

    #[test]
    fn filter_handles_invalid_parameters_inputs_and_elapsed_time() {
        let mut fallback = MadgwickAhrs::new(f32::NAN, f32::NAN);
        let mut default_filter = MadgwickAhrs::default();
        let sample = SensorData::new(0, 0, 0, 0, 0, i16::MAX);
        let default_q = default_filter.update_from_sensor_data(&sample);
        let fallback_q = fallback.update_from_sensor_data(&sample);
        assert_eq!(fallback_q, default_q);

        let mut longer_dt = MadgwickAhrs::default();
        let long_q = longer_dt.update_from_sensor_data_with_dt(&sample, 0.5);
        assert_ne!(long_q, default_q);

        let before_invalid_dt = longer_dt.quaternion();
        assert_eq!(
            longer_dt.update_from_sensor_data_with_dt(&sample, f32::NAN),
            before_invalid_dt
        );
        assert_eq!(
            longer_dt.update_from_sensor_data_with_dt(&sample, 0.0),
            before_invalid_dt
        );
        assert_eq!(
            longer_dt.update_imu(f32::INFINITY, 0.0, 0.0, 0.0, 0.0, 1.0),
            before_invalid_dt
        );

        longer_dt.reset();
        assert_eq!(longer_dt.quaternion(), Quaternion::IDENTITY);
    }
}
