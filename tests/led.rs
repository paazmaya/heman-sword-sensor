mod led_engine_tests {
    use xiao_nrf52840_sword::led::{
        encode_ws2812_pwm, LedAnimation, LedControlMode, LedEngine, RgbColor, BITS_PER_LED,
        BLADE_SIDE_LEDS, DURATION_IMPACT_MS, DURATION_MOTION_STOP_MS, DURATION_SLASH_MS,
        DURATION_THRUST_MS, LED_FRAME_INTERVAL_MS, NUM_LEDS, PWM_BUFFER_WORDS, PWM_POLARITY_BIT,
        PWM_RESET_WORD, PWM_T0H_WORD, PWM_T1H_WORD, RESET_SLOTS, TIP_LEDS, TIP_START, WHITE_CAP,
    };
    use xiao_nrf52840_sword::{
        detect_sword_gesture, detect_sword_gesture_with_thresholds, SensorData, IMPACT_THRESHOLD,
        THRUST_THRESHOLD,
    };

    fn tip_pixels(frame: &[RgbColor; NUM_LEDS]) -> &[RgbColor] {
        &frame[TIP_START..TIP_START + TIP_LEDS]
    }

    #[test]
    fn test_led_strip_constants_for_folded_strip_and_tip_module() {
        assert_eq!(BLADE_SIDE_LEDS, 63);
        assert_eq!(TIP_LEDS, 3);
        assert_eq!(NUM_LEDS, 63 + 3 + 63);
        assert_eq!(TIP_START, 63);
        assert_eq!(LED_FRAME_INTERVAL_MS, 20); // 50 Hz
        assert_eq!(PWM_BUFFER_WORDS, 129 * 24 + 40);
        assert_eq!(PWM_BUFFER_WORDS, 3136);
    }

    #[test]
    fn test_white_cap_limits_white_but_keeps_saturated_colors() {
        assert_eq!(
            RgbColor::WHITE.limit_white(WHITE_CAP),
            RgbColor::new(WHITE_CAP, WHITE_CAP, WHITE_CAP)
        );
        // White component is capped, the colored part is kept: (80, 180, 255) has min 80 <= cap.
        assert_eq!(
            RgbColor::GRAYSKULL_BLUE.limit_white(WHITE_CAP),
            RgbColor::GRAYSKULL_BLUE
        );
        assert_eq!(
            RgbColor::new(200, 150, 100).limit_white(WHITE_CAP),
            RgbColor::new(185, 135, 85)
        );
        assert_eq!(
            RgbColor::SLASH_ORANGE.limit_white(WHITE_CAP),
            RgbColor::SLASH_ORANGE
        );
        assert_eq!(RgbColor::BLACK.limit_white(WHITE_CAP), RgbColor::BLACK);
    }

    #[test]
    fn test_encode_caps_white_pixels() {
        let mut leds = [RgbColor::BLACK; NUM_LEDS];
        leds[0] = RgbColor::WHITE;
        let mut capped = [0u16; PWM_BUFFER_WORDS];
        encode_ws2812_pwm(&leds, 255, &mut capped);

        let mut expected_leds = [RgbColor::BLACK; NUM_LEDS];
        expected_leds[0] = RgbColor::new(WHITE_CAP, WHITE_CAP, WHITE_CAP);
        let mut expected = [0u16; PWM_BUFFER_WORDS];
        encode_ws2812_pwm(&expected_leds, 255, &mut expected);
        assert_eq!(capped, expected);
        assert_ne!(capped[0..24], [PWM_T1H_WORD; 24]);
    }

    #[test]
    fn test_rgb_color_scaling() {
        let color = RgbColor::new(200, 100, 50);
        assert_eq!(color.scale(0), RgbColor::BLACK);
        assert_eq!(color.scale(255), color);
        let half = color.scale(128);
        assert!(half.r >= 99 && half.r <= 101);
        assert!(half.g >= 49 && half.g <= 51);
    }

    #[test]
    fn test_led_animation_id_round_trip() {
        let custom = RgbColor::new(12, 34, 56);
        let variants = [
            LedAnimation::Off,
            LedAnimation::IdleBreathing,
            LedAnimation::UpwardThrust,
            LedAnimation::DownwardSlash,
            LedAnimation::ImpactHit,
            LedAnimation::MotionStop,
            LedAnimation::SolidColor(custom),
        ];
        for anim in variants {
            let id = anim.id();
            let restored = LedAnimation::from_id(id, custom).expect("valid id");
            assert_eq!(anim, restored);
        }
        assert!(LedAnimation::from_id(99, RgbColor::WHITE).is_none());
    }

    #[test]
    fn test_led_engine_default_starts_in_idle_breathing() {
        let mut engine = LedEngine::new();
        assert_eq!(engine.current_animation(), LedAnimation::IdleBreathing);
        assert_eq!(engine.control_mode(), LedControlMode::AutoMotion);
        assert_eq!(engine.brightness(), 255);
        let frame = engine.tick(20);
        assert_eq!(frame.len(), NUM_LEDS);
        // Idle breathing produces a non-black dim blue/cyan glow on the blade, tip module off
        assert!(frame[0].b > 0);
        assert!(frame[NUM_LEDS - 1].b > 0);
        assert!(tip_pixels(&frame).iter().all(|p| *p == RgbColor::BLACK));
    }

    #[test]
    fn test_upward_thrust_propagates_guard_to_tip_and_returns_to_idle() {
        let mut engine = LedEngine::new();
        engine.trigger_motion(LedAnimation::UpwardThrust);
        assert_eq!(engine.current_animation(), LedAnimation::UpwardThrust);
        let tip_end_up = BLADE_SIDE_LEDS - 1;
        let tip_end_down = NUM_LEDS - BLADE_SIDE_LEDS;

        // Frame 0 (t = 0 ms): guard pixels are lit on both sides, tip end and tip module are dark
        let frame_start = engine.tick(50);
        assert_ne!(frame_start[0], RgbColor::BLACK);
        assert_ne!(frame_start[NUM_LEDS - 1], RgbColor::BLACK);
        assert_eq!(frame_start[tip_end_up], RgbColor::BLACK);
        assert_eq!(frame_start[tip_end_down], RgbColor::BLACK);
        assert!(tip_pixels(&frame_start)
            .iter()
            .all(|p| *p == RgbColor::BLACK));

        // Advance to end of thrust (t = 200 ms): blade tip and tip module are fully lit
        let _ = engine.tick(50); // t=50
        let _ = engine.tick(50); // t=100
        let _ = engine.tick(50); // t=150
        let frame_end = engine.tick(20); // t=200 (duration limit)
        assert_ne!(frame_end[tip_end_up], RgbColor::BLACK);
        assert_ne!(frame_end[tip_end_down], RgbColor::BLACK);
        assert!(tip_pixels(&frame_end).iter().all(|p| *p == RgbColor::WHITE));

        // Next tick after duration elapses transitions back to IdleBreathing
        assert_eq!(DURATION_THRUST_MS, 200);
        let _ = engine.tick(20);
        assert_eq!(engine.current_animation(), LedAnimation::IdleBreathing);
    }

    #[test]
    fn test_upward_thrust_tip_module_ramps_up_during_beam() {
        let mut engine = LedEngine::new();
        engine.trigger_motion(LedAnimation::UpwardThrust);
        let _ = engine.tick(100);
        let mid = engine.tick(100);
        let mid_tip = tip_pixels(&mid)[0];
        assert!(mid_tip != RgbColor::BLACK && mid_tip != RgbColor::WHITE);
    }

    #[test]
    fn test_downward_slash_propagates_tip_to_guard() {
        let mut engine = LedEngine::new();
        engine.trigger_motion(LedAnimation::DownwardSlash);
        assert_eq!(engine.current_animation(), LedAnimation::DownwardSlash);

        // Frame 0 (t = 0 ms): tip end is lit on both sides, guard and tip module are dark
        let frame0 = engine.tick(50);
        assert_ne!(frame0[BLADE_SIDE_LEDS - 1], RgbColor::BLACK);
        assert_ne!(frame0[NUM_LEDS - BLADE_SIDE_LEDS], RgbColor::BLACK);
        assert_eq!(frame0[0], RgbColor::BLACK);
        assert_eq!(frame0[NUM_LEDS - 1], RgbColor::BLACK);
        assert!(tip_pixels(&frame0).iter().all(|p| *p == RgbColor::BLACK));

        // Advance to end of 150 ms slash: guard pixels are lit
        let _ = engine.tick(50);
        let _ = engine.tick(50);
        let frame_end = engine.tick(20); // t=150 ms
        assert_ne!(frame_end[0], RgbColor::BLACK);
        assert_ne!(frame_end[NUM_LEDS - 1], RgbColor::BLACK);
        assert_eq!(DURATION_SLASH_MS, 150);
    }

    #[test]
    fn test_impact_hit_flashes_bright_then_decays() {
        let mut engine = LedEngine::new();
        engine.trigger_motion(LedAnimation::ImpactHit);
        let f0 = engine.tick(50);
        let f1 = engine.tick(40);
        // Starts full white (255) and decays at t=50 ms
        assert_eq!(f0[0], RgbColor::WHITE);
        assert!(f1[0].r < f0[0].r);
        assert_eq!(DURATION_IMPACT_MS, 100);
    }

    #[test]
    fn test_motion_stop_fades_over_1000_ms() {
        let mut engine = LedEngine::new();
        engine.start_animation(LedAnimation::SolidColor(RgbColor::WHITE), None);
        let _ = engine.tick(20);
        engine.start_animation(LedAnimation::MotionStop, None);
        let start_frame = engine.tick(500);
        let mid_frame = engine.tick(400);
        assert!(mid_frame[0].r < start_frame[0].r);
        assert_eq!(DURATION_MOTION_STOP_MS, 1000);

        let near_terminal = engine.tick(100);
        assert!(near_terminal[0].r > 0);
        let terminal = engine.tick(0);
        assert_eq!(terminal, [RgbColor::BLACK; NUM_LEDS]);
        assert_eq!(engine.current_animation(), LedAnimation::Off);
        assert_eq!(engine.tick(20), [RgbColor::BLACK; NUM_LEDS]);
    }

    #[test]
    fn maximum_duration_arithmetic_does_not_overflow() {
        let mut engine = LedEngine::new();
        engine.start_animation(LedAnimation::ImpactHit, Some(u32::MAX));
        let _ = engine.tick(1);
        let near_terminal = engine.tick(u32::MAX);
        assert!(near_terminal[0].r > 0);
        let terminal = engine.tick(0);
        assert_eq!(terminal, [RgbColor::BLACK; NUM_LEDS]);

        engine.start_animation(LedAnimation::UpwardThrust, Some(u32::MAX));
        let _ = engine.tick(1);
        let _ = engine.tick(u32::MAX);
        let endpoint = engine.tick(0);
        assert_ne!(endpoint[BLADE_SIDE_LEDS - 1], RgbColor::BLACK);
        assert_ne!(endpoint[TIP_START], RgbColor::BLACK);
        assert_eq!(engine.current_animation(), LedAnimation::IdleBreathing);
    }

    #[test]
    fn repeated_same_gesture_does_not_restart_animation() {
        let mut engine = LedEngine::new();
        engine.trigger_motion(LedAnimation::UpwardThrust);
        let _ = engine.tick(100);
        engine.trigger_motion(LedAnimation::UpwardThrust);
        let frame = engine.tick(100);
        assert_ne!(frame[29], RgbColor::BLACK);
        let terminal = engine.tick(0);
        assert_ne!(terminal[BLADE_SIDE_LEDS - 1], RgbColor::BLACK);
        assert_ne!(terminal[TIP_START], RgbColor::BLACK);
        assert_eq!(engine.current_animation(), LedAnimation::IdleBreathing);
    }

    #[test]
    fn test_manual_override_ignores_motion_triggers() {
        let mut engine = LedEngine::new();
        let purple = RgbColor::new(180, 0, 255);
        engine.set_control_mode(LedControlMode::ManualOverride);
        engine.start_animation(LedAnimation::SolidColor(purple), None);

        // Motion gesture should be ignored while in ManualOverride
        engine.trigger_motion(LedAnimation::UpwardThrust);
        assert_eq!(engine.current_animation(), LedAnimation::SolidColor(purple));
        let frame = engine.tick(20);
        assert_eq!(frame[0], purple);
        assert_eq!(frame[NUM_LEDS - 1], purple);
        assert!(tip_pixels(&frame).iter().all(|p| *p == purple));
    }

    #[test]
    fn test_encode_ws2812_pwm_grb_bit_order_and_reset_latch() {
        let mut leds = [RgbColor::BLACK; NUM_LEDS];
        // LED 0: G = 0xFF, R = 0x00, B = 0xAA (10101010)
        leds[0] = RgbColor::new(0x00, 0xFF, 0xAA);

        let mut pwm = [0u16; PWM_BUFFER_WORDS];
        encode_ws2812_pwm(&leds, 255, &mut pwm);

        // First 8 words = Green (0xFF -> all PWM_T1H_WORD)
        for &w in &pwm[0..8] {
            assert_eq!(w, PWM_T1H_WORD);
            assert_ne!(w & PWM_POLARITY_BIT, 0);
        }
        // Next 8 words = Red (0x00 -> all PWM_T0H_WORD)
        for &w in &pwm[8..16] {
            assert_eq!(w, PWM_T0H_WORD);
        }
        // Next 8 words = Blue (0xAA = 0b1010_1010 -> alternating T1H, T0H)
        for i in 0..8 {
            let expected = if i % 2 == 0 {
                PWM_T1H_WORD
            } else {
                PWM_T0H_WORD
            };
            assert_eq!(pwm[16 + i], expected);
        }
        // Final RESET_SLOTS words must all be PWM_RESET_WORD (0x8000)
        for &w in &pwm[NUM_LEDS * 24..PWM_BUFFER_WORDS] {
            assert_eq!(w, PWM_RESET_WORD);
        }
        assert_eq!(pwm[NUM_LEDS * 24..].len(), RESET_SLOTS);
    }

    #[test]
    fn pwm_encodes_later_led_and_brightness_extremes() {
        let mut leds = [RgbColor::BLACK; NUM_LEDS];
        leds[1] = RgbColor::new(0x10, 0x20, 0x40);
        let mut pwm = [0u16; PWM_BUFFER_WORDS];

        encode_ws2812_pwm(&leds, 255, &mut pwm);
        let second_led = 24;
        // G=0x20 starts with zero then zeros; R=0x10 starts with zero;
        // B=0x40 has its only one at the second MSB-first bit.
        assert_eq!(pwm[second_led], PWM_T0H_WORD);
        assert_eq!(pwm[second_led + 8], PWM_T0H_WORD);
        assert_eq!(pwm[second_led + 16], PWM_T0H_WORD);
        assert_eq!(pwm[second_led + 17], PWM_T1H_WORD);

        encode_ws2812_pwm(&leds, 0, &mut pwm);
        assert!(pwm[..NUM_LEDS * BITS_PER_LED]
            .iter()
            .all(|word| *word == PWM_T0H_WORD));
        encode_ws2812_pwm(&leds, 255, &mut pwm);
        assert_eq!(pwm[NUM_LEDS * BITS_PER_LED], PWM_RESET_WORD);
    }

    #[test]
    fn test_detect_sword_gesture_priority_and_thresholds() {
        // Impact takes top priority
        let impact = SensorData::new(IMPACT_THRESHOLD + 10, 0, THRUST_THRESHOLD + 20, 0, 0, 0);
        assert_eq!(detect_sword_gesture(&impact), Some(LedAnimation::ImpactHit));

        // Upward thrust
        let thrust = SensorData::new(0, 0, THRUST_THRESHOLD + 15, 0, 0, 0);
        assert_eq!(
            detect_sword_gesture(&thrust),
            Some(LedAnimation::UpwardThrust)
        );

        // Downward slash (negative Z accel + pitch/roll gyro)
        let slash = SensorData::new(0, 0, -(THRUST_THRESHOLD + 20), 60, 0, 0);
        assert_eq!(
            detect_sword_gesture(&slash),
            Some(LedAnimation::DownwardSlash)
        );

        // Idle sample triggers no gesture
        let idle = SensorData::new(0, 0, 10, 0, 0, 0);
        assert_eq!(detect_sword_gesture(&idle), None);

        // Custom thresholds
        assert_eq!(
            detect_sword_gesture_with_thresholds(&thrust, 200, 500),
            None
        );
    }
}
