mod tlv_api_tests {
    use xiao_nrf52840_sword::api::{
        ApiCommand, ApiEvent, ApiStatus, EVT_COMMAND_ACK, EVT_DEVICE_STATUS, EVT_GESTURE,
        OP_CLEAR_BONDED_DEVICES, OP_GET_STATUS, OP_SET_BRIGHTNESS, OP_SET_LED_MODE,
        OP_SET_THRESHOLDS, OP_TRIGGER_ANIMATION, TLV_PACKET_SIZE,
    };
    use xiao_nrf52840_sword::led::{LedAnimation, LedControlMode, RgbColor};

    #[test]
    fn status_codes_match_literal_wire_mapping() {
        let statuses = [
            ApiStatus::Ok,
            ApiStatus::InvalidOpcode,
            ApiStatus::InvalidLength,
            ApiStatus::InvalidParameter,
            ApiStatus::StorageError,
        ];
        for (byte, status) in statuses.into_iter().enumerate() {
            assert_eq!(status.to_u8(), byte as u8);
            assert_eq!(ApiStatus::from_u8(byte as u8), Some(status));
        }
        assert_eq!(ApiStatus::from_u8(5), None);
    }

    #[test]
    fn command_parser_covers_declared_lengths_parameters_and_padding() {
        assert_eq!(ApiCommand::parse(&[]), Err((0, ApiStatus::InvalidLength)));
        assert_eq!(
            ApiCommand::parse(&[OP_SET_BRIGHTNESS]),
            Err((OP_SET_BRIGHTNESS, ApiStatus::InvalidLength))
        );
        assert_eq!(
            ApiCommand::parse(&[OP_SET_LED_MODE, 5, 0, 0, 0, 0, 0]),
            Err((OP_SET_LED_MODE, ApiStatus::InvalidLength))
        );
        assert_eq!(
            ApiCommand::parse(&[OP_SET_LED_MODE, 6, 2, 0, 0, 0, 0, 0]),
            Err((OP_SET_LED_MODE, ApiStatus::InvalidParameter))
        );
        assert_eq!(
            ApiCommand::parse(&[OP_TRIGGER_ANIMATION, 2, 1, 0]),
            Err((OP_TRIGGER_ANIMATION, ApiStatus::InvalidLength))
        );
        assert_eq!(
            ApiCommand::parse(&[OP_TRIGGER_ANIMATION, 3, 0xFE, 0, 0]),
            Err((OP_TRIGGER_ANIMATION, ApiStatus::InvalidParameter))
        );
        assert_eq!(
            ApiCommand::parse(&[OP_SET_THRESHOLDS, 3, 1, 0, 2]),
            Err((OP_SET_THRESHOLDS, ApiStatus::InvalidLength))
        );
        assert_eq!(
            ApiCommand::parse(&[OP_SET_THRESHOLDS, 4, 0, 0, 1, 0]),
            Err((OP_SET_THRESHOLDS, ApiStatus::InvalidParameter))
        );
        assert_eq!(
            ApiCommand::parse(&[OP_GET_STATUS, 1, 0]),
            Err((OP_GET_STATUS, ApiStatus::InvalidLength))
        );
        assert_eq!(
            ApiCommand::parse(&[OP_CLEAR_BONDED_DEVICES, 1, 0]),
            Err((OP_CLEAR_BONDED_DEVICES, ApiStatus::InvalidLength))
        );
        assert_eq!(
            ApiCommand::parse(&[OP_SET_BRIGHTNESS, 15, 0]),
            Err((OP_SET_BRIGHTNESS, ApiStatus::InvalidLength))
        );

        assert_eq!(
            ApiCommand::parse(&[OP_SET_BRIGHTNESS, 1, 0x80, 0xAA, 0xBB]),
            Ok(ApiCommand::SetBrightness { brightness: 0x80 })
        );
        assert_eq!(
            ApiCommand::parse(&[OP_TRIGGER_ANIMATION, 3, 2, 0x34, 0x12]),
            Ok(ApiCommand::TriggerAnimation {
                animation: LedAnimation::UpwardThrust,
                duration_ms: 0x1234
            })
        );
        assert_eq!(
            ApiCommand::parse(&[OP_TRIGGER_ANIMATION, 6, 2, 1, 0, 9, 8, 7]),
            Ok(ApiCommand::TriggerAnimation {
                animation: LedAnimation::UpwardThrust,
                duration_ms: 1
            })
        );
    }

    #[test]
    fn test_tlv_set_led_mode_round_trip() {
        let cmd = ApiCommand::SetLedMode {
            mode: LedControlMode::ManualOverride,
            color: RgbColor::new(10, 200, 255),
            duration_ms: 1500,
        };
        let encoded = cmd.encode();
        assert_eq!(encoded.len(), TLV_PACKET_SIZE);
        assert_eq!(encoded[0], OP_SET_LED_MODE);
        assert_eq!(encoded[1], 6);
        let parsed = ApiCommand::parse(&encoded).expect("valid SetLedMode");
        assert_eq!(cmd, parsed);
    }

    #[test]
    fn test_tlv_set_brightness_round_trip() {
        let cmd = ApiCommand::SetBrightness { brightness: 128 };
        let encoded = cmd.encode();
        assert_eq!(encoded[0], OP_SET_BRIGHTNESS);
        assert_eq!(encoded[1], 1);
        assert_eq!(ApiCommand::parse(&encoded), Ok(cmd));
    }

    #[test]
    fn test_tlv_trigger_animation_round_trip() {
        let cmd = ApiCommand::TriggerAnimation {
            animation: LedAnimation::SolidColor(RgbColor::new(255, 0, 128)),
            duration_ms: 500,
        };
        let encoded = cmd.encode();
        assert_eq!(encoded[0], OP_TRIGGER_ANIMATION);
        assert_eq!(ApiCommand::parse(&encoded), Ok(cmd));
    }

    #[test]
    fn test_tlv_set_thresholds_round_trip_and_validation() {
        let cmd = ApiCommand::SetThresholds {
            thrust_threshold: 75,
            impact_threshold: 450,
        };
        let encoded = cmd.encode();
        assert_eq!(encoded[0], OP_SET_THRESHOLDS);
        assert_eq!(ApiCommand::parse(&encoded), Ok(cmd));

        // Invalid thresholds (impact <= thrust or non-positive thrust)
        let bad = [OP_SET_THRESHOLDS, 4, 100, 0, 50, 0];
        assert_eq!(
            ApiCommand::parse(&bad),
            Err((OP_SET_THRESHOLDS, ApiStatus::InvalidParameter))
        );
    }

    #[test]
    fn test_tlv_get_status_and_clear_bonds_round_trip() {
        let get_status = ApiCommand::GetStatus;
        assert_eq!(
            ApiCommand::parse(&get_status.encode()),
            Ok(ApiCommand::GetStatus)
        );
        assert_eq!(get_status.encode()[0], OP_GET_STATUS);

        let clear = ApiCommand::ClearBondedDevices;
        assert_eq!(
            ApiCommand::parse(&clear.encode()),
            Ok(ApiCommand::ClearBondedDevices)
        );
        assert_eq!(clear.encode()[0], OP_CLEAR_BONDED_DEVICES);
    }

    #[test]
    fn test_tlv_invalid_opcode_and_length_errors() {
        // Unknown opcode
        assert_eq!(
            ApiCommand::parse(&[0xFE, 0]),
            Err((0xFE, ApiStatus::InvalidOpcode))
        );
        // Declared length exceeds slice
        assert_eq!(
            ApiCommand::parse(&[OP_SET_BRIGHTNESS, 5, 100]),
            Err((OP_SET_BRIGHTNESS, ApiStatus::InvalidLength))
        );
        // Wrong payload length for SetBrightness
        assert_eq!(
            ApiCommand::parse(&[OP_SET_BRIGHTNESS, 2, 100, 0]),
            Err((OP_SET_BRIGHTNESS, ApiStatus::InvalidLength))
        );
    }

    #[test]
    fn test_tlv_events_round_trip() {
        let ack = ApiEvent::CommandAck {
            ref_opcode: OP_SET_LED_MODE,
            status: ApiStatus::Ok,
        };
        let ack_bytes = ack.encode();
        assert_eq!(ack_bytes[0], EVT_COMMAND_ACK);
        assert_eq!(ApiEvent::parse(&ack_bytes), Some(ack));

        let gesture = ApiEvent::GestureEvent {
            animation_id: LedAnimation::UpwardThrust.id(),
            peak_accel: 420,
        };
        let gesture_bytes = gesture.encode();
        assert_eq!(gesture_bytes[0], EVT_GESTURE);
        assert_eq!(ApiEvent::parse(&gesture_bytes), Some(gesture));

        let status = ApiEvent::DeviceStatus {
            pairing_status: 3,
            led_mode: 0,
            active_animation: 1,
            brightness: 200,
            thrust_threshold: 50,
            impact_threshold: 300,
        };
        let status_bytes = status.encode();
        assert_eq!(status_bytes[0], EVT_DEVICE_STATUS);
        assert_eq!(ApiEvent::parse(&status_bytes), Some(status));
    }

    #[test]
    fn event_parser_rejects_malformed_and_unknown_frames() {
        assert_eq!(ApiEvent::parse(&[]), None);
        assert_eq!(ApiEvent::parse(&[EVT_GESTURE]), None);
        assert_eq!(ApiEvent::parse(&[EVT_GESTURE, 3, 1]), None);
        assert_eq!(ApiEvent::parse(&[EVT_GESTURE, 15]), None);
        assert_eq!(ApiEvent::parse(&[0xFE, 0]), None);
        assert_eq!(ApiEvent::parse(&[EVT_COMMAND_ACK, 2, 3, 0xFF]), None);
        assert_eq!(ApiEvent::parse(&[EVT_COMMAND_ACK, 1, 3]), None);
        assert_eq!(
            ApiEvent::parse(&[EVT_DEVICE_STATUS, 7, 0, 0, 0, 0, 0, 0, 0]),
            None
        );
    }

    #[test]
    fn event_encoders_match_literal_wire_frames() {
        assert_eq!(
            ApiEvent::CommandAck {
                ref_opcode: OP_SET_BRIGHTNESS,
                status: ApiStatus::StorageError,
            }
            .encode()[..4],
            [0x80, 2, 0x02, 0x04]
        );
        assert_eq!(
            ApiEvent::GestureEvent {
                animation_id: 3,
                peak_accel: -2,
            }
            .encode()[..5],
            [0x81, 3, 3, 0xFE, 0xFF]
        );
        assert_eq!(
            ApiEvent::DeviceStatus {
                pairing_status: 5,
                led_mode: 1,
                active_animation: 6,
                brightness: 0x80,
                thrust_threshold: 0x1234,
                impact_threshold: -1,
            }
            .encode()[..10],
            [0x82, 8, 5, 1, 6, 0x80, 0x34, 0x12, 0xFF, 0xFF]
        );
    }

    #[test]
    fn command_encoder_emits_autonomous_mode_and_non_solid_animation_fields() {
        let mode = ApiCommand::SetLedMode {
            mode: LedControlMode::AutoMotion,
            color: RgbColor::new(0x11, 0x22, 0x33),
            duration_ms: 0x1234,
        }
        .encode();
        assert_eq!(&mode[..8], &[1, 6, 0, 0x11, 0x22, 0x33, 0x34, 0x12]);

        let animation = ApiCommand::TriggerAnimation {
            animation: LedAnimation::ImpactHit,
            duration_ms: 0xABCD,
        }
        .encode();
        assert_eq!(&animation[..5], &[3, 6, 4, 0xCD, 0xAB]);
        assert_eq!(&animation[5..8], &[0, 0, 0]);
    }
}
