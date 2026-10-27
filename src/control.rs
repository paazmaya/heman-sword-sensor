//! Small, deterministic command-to-action seam shared by BLE and host tests.
//!
//! ```
//! use xiao_nrf52840_sword::{api::ApiCommand, control::led_action, led::LedEngine};
//! let command = ApiCommand::parse(&[2, 1, 128]).expect("valid brightness command");
//! let mut engine = LedEngine::new();
//! led_action(&command).expect("LED action").apply(&mut engine);
//! assert_eq!(engine.brightness(), 128);
//! ```

use crate::{
    api::{ApiCommand, ApiEvent, ApiStatus},
    led::{LedAnimation, LedControlMode, LedEngine, RgbColor},
};

/// Effects needed to execute a BLE command; hardware adapters own the queues.
pub trait CommandTarget {
    /// Number of response slots available before starting an operation.
    fn response_capacity(&self) -> usize;
    /// Queue a LED operation, returning false without mutation when full.
    fn enqueue_led(&mut self, action: LedAction) -> bool;
    /// Queue a clear request; its completion response is deferred to storage.
    fn enqueue_clear(&mut self) -> bool;
    /// Apply both validated thresholds.
    fn set_thresholds(&mut self, thrust: i16, impact: i16);
    /// Snapshot applied runtime state for a GetStatus response.
    fn status(&self) -> ApiEvent;
    /// Publish an immediate response to the current connection.
    fn respond(&mut self, event: ApiEvent);
}

/// Dispatch could not start because replies would have been lost.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DispatchError {
    /// Fewer than two response slots remain; no command effect was performed.
    ResponseQueueFull,
}

/// Parse and execute a command using production queue/completion semantics.
///
/// LED acknowledgements mean queue acceptance, not completion. Clear requests
/// produce no immediate success response: storage must reply after erasure.
/// Queue rejection uses the existing wire `StorageError` code.
pub fn dispatch(raw: &[u8], target: &mut impl CommandTarget) -> Result<(), DispatchError> {
    if target.response_capacity() < 2 {
        return Err(DispatchError::ResponseQueueFull);
    }
    let command = match ApiCommand::parse(raw) {
        Ok(command) => command,
        Err((opcode, status)) => {
            target.respond(ApiEvent::CommandAck {
                ref_opcode: opcode,
                status,
            });
            return Ok(());
        }
    };
    let opcode = command.opcode();
    let accepted = if let Some(action) = led_action(&command) {
        target.enqueue_led(action)
    } else {
        match command {
            ApiCommand::SetThresholds {
                thrust_threshold,
                impact_threshold,
            } => {
                target.set_thresholds(thrust_threshold, impact_threshold);
                true
            }
            ApiCommand::GetStatus => {
                target.respond(ApiEvent::CommandAck {
                    ref_opcode: opcode,
                    status: ApiStatus::Ok,
                });
                target.respond(target.status());
                return Ok(());
            }
            ApiCommand::ClearBondedDevices => {
                if target.enqueue_clear() {
                    return Ok(());
                }
                false
            }
            ApiCommand::SetLedMode { .. }
            | ApiCommand::SetBrightness { .. }
            | ApiCommand::TriggerAnimation { .. } => unreachable!("LED action handled above"),
        }
    };
    target.respond(ApiEvent::CommandAck {
        ref_opcode: opcode,
        status: if accepted {
            ApiStatus::Ok
        } else {
            ApiStatus::StorageError
        },
    });
    Ok(())
}

/// LED operation decoded from a command or generated from a motion gesture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LedAction {
    /// Apply manual/automatic mode and its requested color/duration.
    SetMode {
        /// New engine control mode.
        mode: LedControlMode,
        /// Solid color used in manual mode.
        color: RgbColor,
        /// Transient duration in milliseconds; zero selects the engine default.
        duration_ms: u16,
    },
    /// Start an animation immediately.
    Trigger {
        /// Animation to start.
        animation: LedAnimation,
        /// Transient duration in milliseconds; zero selects the animation default.
        duration_ms: u16,
    },
    /// Change master brightness.
    SetBrightness(u8),
    /// Trigger a gesture animation subject to the engine's manual-mode suppression.
    MotionGesture(LedAnimation),
}

impl LedAction {
    /// Apply the action to the production animation engine.
    pub fn apply(self, engine: &mut LedEngine) {
        match self {
            Self::SetMode {
                mode,
                color,
                duration_ms,
            } => {
                engine.set_control_mode(mode);
                match mode {
                    LedControlMode::ManualOverride => engine
                        .start_animation(LedAnimation::SolidColor(color), duration(duration_ms)),
                    LedControlMode::AutoMotion => {
                        engine.start_animation(LedAnimation::IdleBreathing, None)
                    }
                }
            }
            Self::Trigger {
                animation,
                duration_ms,
            } => engine.start_animation(animation, duration(duration_ms)),
            Self::SetBrightness(brightness) => engine.set_brightness(brightness),
            Self::MotionGesture(animation) => engine.trigger_motion(animation),
        }
    }
}

/// Convert an API command into the corresponding LED action, if any.
pub const fn led_action(command: &ApiCommand) -> Option<LedAction> {
    match *command {
        ApiCommand::SetLedMode {
            mode,
            color,
            duration_ms,
        } => Some(LedAction::SetMode {
            mode,
            color,
            duration_ms,
        }),
        ApiCommand::SetBrightness { brightness } => Some(LedAction::SetBrightness(brightness)),
        ApiCommand::TriggerAnimation {
            animation,
            duration_ms,
        } => Some(LedAction::Trigger {
            animation,
            duration_ms,
        }),
        ApiCommand::SetThresholds { .. }
        | ApiCommand::GetStatus
        | ApiCommand::ClearBondedDevices => None,
    }
}

const fn duration(duration_ms: u16) -> Option<u32> {
    if duration_ms == 0 {
        None
    } else {
        Some(duration_ms as u32)
    }
}
