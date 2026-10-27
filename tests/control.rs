use xiao_nrf52840_sword::{
    api::{ApiCommand, ApiEvent, ApiStatus, OP_CLEAR_BONDED_DEVICES},
    control::{dispatch, led_action, CommandTarget, DispatchError, LedAction},
    led::{LedAnimation, LedControlMode, LedEngine, RgbColor},
};

struct Runtime {
    capacity: usize,
    led_capacity: usize,
    clear_capacity: usize,
    leds: Vec<LedAction>,
    clears: usize,
    thresholds: (i16, i16),
    events: Vec<ApiEvent>,
}

impl Runtime {
    fn new() -> Self {
        Self {
            capacity: 8,
            led_capacity: 1,
            clear_capacity: 1,
            leds: Vec::new(),
            clears: 0,
            thresholds: (50, 300),
            events: Vec::new(),
        }
    }
}

impl CommandTarget for Runtime {
    fn response_capacity(&self) -> usize {
        self.capacity - self.events.len()
    }
    fn enqueue_led(&mut self, action: LedAction) -> bool {
        if self.leds.len() == self.led_capacity {
            return false;
        }
        self.leds.push(action);
        true
    }
    fn enqueue_clear(&mut self) -> bool {
        if self.clears == self.clear_capacity {
            return false;
        }
        self.clears += 1;
        true
    }
    fn set_thresholds(&mut self, thrust: i16, impact: i16) {
        self.thresholds = (thrust, impact);
    }
    fn status(&self) -> ApiEvent {
        ApiEvent::DeviceStatus {
            pairing_status: 0,
            led_mode: 0,
            active_animation: 1,
            brightness: 255,
            thrust_threshold: self.thresholds.0,
            impact_threshold: self.thresholds.1,
        }
    }
    fn respond(&mut self, event: ApiEvent) {
        self.events.push(event);
    }
}

#[test]
fn rejected_queues_never_acknowledge_success_or_mutate_applied_state() {
    let mut runtime = Runtime::new();
    let mut engine = LedEngine::new();
    let command = ApiCommand::SetBrightness { brightness: 20 }.encode();
    assert_eq!(dispatch(&command, &mut runtime), Ok(()));
    assert_eq!(engine.brightness(), 255, "acceptance is not application");
    runtime.leds[0].apply(&mut engine);
    assert_eq!(engine.brightness(), 20);
    assert_eq!(dispatch(&command, &mut runtime), Ok(()));
    assert_eq!(runtime.leds.len(), 1);
    assert_eq!(
        runtime.events,
        [
            ApiEvent::CommandAck {
                ref_opcode: 2,
                status: ApiStatus::Ok
            },
            ApiEvent::CommandAck {
                ref_opcode: 2,
                status: ApiStatus::StorageError
            },
        ]
    );

    assert_eq!(
        dispatch(&ApiCommand::ClearBondedDevices.encode(), &mut runtime),
        Ok(())
    );
    assert_eq!(runtime.events.len(), 2, "erase ACK must await persistence");
    assert_eq!(
        dispatch(&ApiCommand::ClearBondedDevices.encode(), &mut runtime),
        Ok(())
    );
    assert_eq!(runtime.clears, 1);
    assert_eq!(
        runtime.events[2],
        ApiEvent::CommandAck {
            ref_opcode: OP_CLEAR_BONDED_DEVICES,
            status: ApiStatus::StorageError,
        }
    );
}

#[test]
fn response_saturation_refuses_side_effects_and_get_status_requires_two_slots() {
    let mut runtime = Runtime::new();
    runtime.capacity = 1;
    for command in [
        ApiCommand::SetThresholds {
            thrust_threshold: 100,
            impact_threshold: 500,
        },
        ApiCommand::SetBrightness { brightness: 0 },
        ApiCommand::ClearBondedDevices,
        ApiCommand::GetStatus,
    ] {
        assert_eq!(
            dispatch(&command.encode(), &mut runtime),
            Err(DispatchError::ResponseQueueFull)
        );
    }
    assert!(runtime.events.is_empty());
    assert!(runtime.leds.is_empty());
    assert_eq!(runtime.clears, 0);
    assert_eq!(runtime.thresholds, (50, 300));
}

#[test]
fn validated_thresholds_status_and_parse_errors_follow_production_path() {
    let mut runtime = Runtime::new();
    dispatch(&[4, 4, 100, 0, 0xF4, 1], &mut runtime).unwrap();
    assert_eq!(runtime.thresholds, (100, 500));
    dispatch(&[5, 0], &mut runtime).unwrap();
    assert_eq!(runtime.events[2], runtime.status());
    dispatch(&[4, 4, 0, 0, 10, 0], &mut runtime).unwrap();
    assert_eq!(runtime.thresholds, (100, 500));
    assert_eq!(
        runtime.events[3],
        ApiEvent::CommandAck {
            ref_opcode: 4,
            status: ApiStatus::InvalidParameter,
        }
    );
    dispatch(&[], &mut runtime).unwrap();
    assert_eq!(
        runtime.events[4],
        ApiEvent::CommandAck {
            ref_opcode: 0,
            status: ApiStatus::InvalidLength,
        }
    );
}

#[test]
fn api_commands_map_to_typed_led_actions_or_no_led_action() {
    let color = RgbColor::new(1, 2, 3);
    assert_eq!(
        led_action(&ApiCommand::SetLedMode {
            mode: LedControlMode::ManualOverride,
            color,
            duration_ms: 10,
        }),
        Some(LedAction::SetMode {
            mode: LedControlMode::ManualOverride,
            color,
            duration_ms: 10,
        })
    );
    assert_eq!(
        led_action(&ApiCommand::SetBrightness { brightness: 128 }),
        Some(LedAction::SetBrightness(128))
    );
    assert_eq!(
        led_action(&ApiCommand::TriggerAnimation {
            animation: LedAnimation::ImpactHit,
            duration_ms: 25,
        }),
        Some(LedAction::Trigger {
            animation: LedAnimation::ImpactHit,
            duration_ms: 25,
        })
    );
    assert_eq!(
        led_action(&ApiCommand::SetThresholds {
            thrust_threshold: 50,
            impact_threshold: 300,
        }),
        None
    );
    assert_eq!(led_action(&ApiCommand::GetStatus), None);
    assert_eq!(led_action(&ApiCommand::ClearBondedDevices), None);
}

#[test]
fn manual_mode_action_changes_engine_and_holds_requested_color() {
    let mut engine = LedEngine::new();
    LedAction::SetMode {
        mode: LedControlMode::ManualOverride,
        color: RgbColor::new(10, 20, 30),
        duration_ms: 0,
    }
    .apply(&mut engine);
    assert_eq!(engine.control_mode(), LedControlMode::ManualOverride);
    assert_eq!(
        engine.tick(20),
        [RgbColor::new(10, 20, 30); xiao_nrf52840_sword::led::NUM_LEDS]
    );

    LedAction::MotionGesture(LedAnimation::UpwardThrust).apply(&mut engine);
    assert_eq!(
        engine.current_animation(),
        LedAnimation::SolidColor(RgbColor::new(10, 20, 30))
    );
}

#[test]
fn auto_mode_brightness_and_trigger_actions_apply_to_production_engine() {
    let mut engine = LedEngine::new();
    LedAction::SetMode {
        mode: LedControlMode::ManualOverride,
        color: RgbColor::WHITE,
        duration_ms: 1,
    }
    .apply(&mut engine);
    LedAction::SetMode {
        mode: LedControlMode::AutoMotion,
        color: RgbColor::BLACK,
        duration_ms: 50,
    }
    .apply(&mut engine);
    assert_eq!(engine.control_mode(), LedControlMode::AutoMotion);
    assert_eq!(engine.current_animation(), LedAnimation::IdleBreathing);

    LedAction::SetBrightness(0).apply(&mut engine);
    assert_eq!(engine.brightness(), 0);

    LedAction::Trigger {
        animation: LedAnimation::ImpactHit,
        duration_ms: 10,
    }
    .apply(&mut engine);
    assert_eq!(engine.current_animation(), LedAnimation::ImpactHit);
    let _ = engine.tick(10);
    let terminal = engine.tick(1);
    assert_eq!(terminal[0], RgbColor::BLACK);
    assert_eq!(engine.current_animation(), LedAnimation::IdleBreathing);
}

#[test]
fn motion_action_uses_auto_mode_and_retrigger_does_not_restart_clock() {
    let mut engine = LedEngine::new();
    LedAction::MotionGesture(LedAnimation::UpwardThrust).apply(&mut engine);
    let first = engine.tick(20);
    LedAction::MotionGesture(LedAnimation::UpwardThrust).apply(&mut engine);
    let second = engine.tick(20);
    assert_ne!(first, second);
    assert_eq!(engine.current_animation(), LedAnimation::UpwardThrust);
}
