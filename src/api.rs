//! Fixed-size TLV command and event protocol.

use crate::led::{LedAnimation, LedControlMode, RgbColor};

/// Fixed GATT characteristic buffer size for `command_control`.
pub const TLV_PACKET_SIZE: usize = 16;
/// Maximum payload length inside a 16-byte TLV packet.
pub const TLV_MAX_PAYLOAD_LEN: usize = TLV_PACKET_SIZE - 2;

/// `0x01`: set LED mode.
pub const OP_SET_LED_MODE: u8 = 0x01;
/// `0x02`: set master brightness.
pub const OP_SET_BRIGHTNESS: u8 = 0x02;
/// `0x03`: trigger an animation.
pub const OP_TRIGGER_ANIMATION: u8 = 0x03;
/// `0x04`: update motion thresholds.
pub const OP_SET_THRESHOLDS: u8 = 0x04;
/// `0x05`: request a status snapshot.
pub const OP_GET_STATUS: u8 = 0x05;
/// `0x06`: clear stored bonded-device data.
pub const OP_CLEAR_BONDED_DEVICES: u8 = 0x06;

/// `0x80`: command acknowledgement event.
pub const EVT_COMMAND_ACK: u8 = 0x80;
/// `0x81`: gesture event.
pub const EVT_GESTURE: u8 = 0x81;
/// `0x82`: device-status event.
pub const EVT_DEVICE_STATUS: u8 = 0x82;

/// Status/error codes returned in [`ApiEvent::CommandAck`].
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApiStatus {
    /// Command completed successfully at the execution stage acknowledged by the caller.
    Ok = 0x00,
    /// Opcode is not recognized.
    InvalidOpcode = 0x01,
    /// The declared payload length is invalid or truncated.
    InvalidLength = 0x02,
    /// A payload value is outside the accepted domain.
    InvalidParameter = 0x03,
    /// A storage operation failed.
    StorageError = 0x04,
}

impl ApiStatus {
    /// Return the stable status wire byte.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parse a status wire byte, returning `None` for unknown values.
    pub const fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x00 => Some(ApiStatus::Ok),
            0x01 => Some(ApiStatus::InvalidOpcode),
            0x02 => Some(ApiStatus::InvalidLength),
            0x03 => Some(ApiStatus::InvalidParameter),
            0x04 => Some(ApiStatus::StorageError),
            _ => None,
        }
    }
}

/// Commands sent from the mobile/web client to the sword over `command_control` (`…0006`).
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApiCommand {
    /// `0x01`: Set LED control mode (`0` = AutoMotion, `1` = ManualOverride), RGB color, and optional duration (ms).
    SetLedMode {
        /// Automatic or manual engine mode.
        mode: LedControlMode,
        /// Requested solid RGB color for manual mode.
        color: RgbColor,
        /// Requested transient duration in milliseconds; zero selects default.
        duration_ms: u16,
    },
    /// `0x02`: Set global LED brightness (`0..=255`).
    SetBrightness {
        /// Master brightness.
        brightness: u8,
    },
    /// `0x03`: Trigger an animation; payload is 3 bytes, or 6 with optional RGB.
    ///
    /// With 3 bytes the color defaults to white; RGB is used only for the
    /// solid-color animation ID.
    TriggerAnimation {
        /// Animation selected by ID; solid color may be included in the payload.
        animation: LedAnimation,
        /// Requested transient duration in milliseconds; zero selects default.
        duration_ms: u16,
    },
    /// `0x04`: Update positive raw-count thresholds; impact must exceed thrust.
    SetThresholds {
        /// Positive upward-thrust raw-count threshold.
        thrust_threshold: i16,
        /// Positive impact raw-count threshold, greater than thrust.
        impact_threshold: i16,
    },
    /// `0x05`: Request a [`ApiEvent::DeviceStatus`] snapshot.
    GetStatus,
    /// `0x06`: Clear bonded device records and reopen BLE advertising.
    ClearBondedDevices,
}

impl ApiCommand {
    /// Return the TLV opcode byte for this command.
    pub const fn opcode(&self) -> u8 {
        match self {
            ApiCommand::SetLedMode { .. } => OP_SET_LED_MODE,
            ApiCommand::SetBrightness { .. } => OP_SET_BRIGHTNESS,
            ApiCommand::TriggerAnimation { .. } => OP_TRIGGER_ANIMATION,
            ApiCommand::SetThresholds { .. } => OP_SET_THRESHOLDS,
            ApiCommand::GetStatus => OP_GET_STATUS,
            ApiCommand::ClearBondedDevices => OP_CLEAR_BONDED_DEVICES,
        }
    }

    /// Encode the command into a 16-byte TLV buffer.
    pub fn encode(&self) -> [u8; TLV_PACKET_SIZE] {
        let mut buf = [0u8; TLV_PACKET_SIZE];
        buf[0] = self.opcode();
        match *self {
            ApiCommand::SetLedMode {
                mode,
                color,
                duration_ms,
            } => {
                buf[1] = 6;
                buf[2] = match mode {
                    LedControlMode::AutoMotion => 0,
                    LedControlMode::ManualOverride => 1,
                };
                buf[3] = color.r;
                buf[4] = color.g;
                buf[5] = color.b;
                let dur = duration_ms.to_le_bytes();
                buf[6] = dur[0];
                buf[7] = dur[1];
            }
            ApiCommand::SetBrightness { brightness } => {
                buf[1] = 1;
                buf[2] = brightness;
            }
            ApiCommand::TriggerAnimation {
                animation,
                duration_ms,
            } => {
                buf[1] = 6;
                buf[2] = animation.id();
                let dur = duration_ms.to_le_bytes();
                buf[3] = dur[0];
                buf[4] = dur[1];
                if let LedAnimation::SolidColor(c) = animation {
                    buf[5] = c.r;
                    buf[6] = c.g;
                    buf[7] = c.b;
                }
            }
            ApiCommand::SetThresholds {
                thrust_threshold,
                impact_threshold,
            } => {
                buf[1] = 4;
                buf[2..4].copy_from_slice(&thrust_threshold.to_le_bytes());
                buf[4..6].copy_from_slice(&impact_threshold.to_le_bytes());
            }
            ApiCommand::GetStatus | ApiCommand::ClearBondedDevices => {
                buf[1] = 0;
            }
        }
        buf
    }

    /// Parse a TLV command from a raw byte slice (or 16-byte GATT write).
    ///
    /// On error, returns `(opcode, ApiStatus)` so the firmware can immediately
    /// emit a matching [`ApiEvent::CommandAck`].
    /// The declared payload length must exactly match the command. Bytes after
    /// the declared payload are ignored, allowing fixed-size padded GATT writes.
    pub fn parse(raw: &[u8]) -> Result<Self, (u8, ApiStatus)> {
        if raw.len() < 2 {
            let op = raw.first().copied().unwrap_or(0);
            return Err((op, ApiStatus::InvalidLength));
        }
        let opcode = raw[0];
        let len = raw[1] as usize;
        if len > TLV_MAX_PAYLOAD_LEN || raw.len() < 2 + len {
            return Err((opcode, ApiStatus::InvalidLength));
        }
        let payload = &raw[2..2 + len];

        match opcode {
            OP_SET_LED_MODE => {
                if len != 6 {
                    return Err((opcode, ApiStatus::InvalidLength));
                }
                let mode = match payload[0] {
                    0 => LedControlMode::AutoMotion,
                    1 => LedControlMode::ManualOverride,
                    _ => return Err((opcode, ApiStatus::InvalidParameter)),
                };
                let color = RgbColor::new(payload[1], payload[2], payload[3]);
                let duration_ms = u16::from_le_bytes([payload[4], payload[5]]);
                Ok(ApiCommand::SetLedMode {
                    mode,
                    color,
                    duration_ms,
                })
            }
            OP_SET_BRIGHTNESS => {
                if len != 1 {
                    return Err((opcode, ApiStatus::InvalidLength));
                }
                Ok(ApiCommand::SetBrightness {
                    brightness: payload[0],
                })
            }
            OP_TRIGGER_ANIMATION => {
                if len != 3 && len != 6 {
                    return Err((opcode, ApiStatus::InvalidLength));
                }
                let color = if len == 6 {
                    RgbColor::new(payload[3], payload[4], payload[5])
                } else {
                    RgbColor::WHITE
                };
                let animation = LedAnimation::from_id(payload[0], color)
                    .ok_or((opcode, ApiStatus::InvalidParameter))?;
                let duration_ms = u16::from_le_bytes([payload[1], payload[2]]);
                Ok(ApiCommand::TriggerAnimation {
                    animation,
                    duration_ms,
                })
            }
            OP_SET_THRESHOLDS => {
                if len != 4 {
                    return Err((opcode, ApiStatus::InvalidLength));
                }
                let thrust_threshold = i16::from_le_bytes([payload[0], payload[1]]);
                let impact_threshold = i16::from_le_bytes([payload[2], payload[3]]);
                if thrust_threshold <= 0 || impact_threshold <= thrust_threshold {
                    return Err((opcode, ApiStatus::InvalidParameter));
                }
                Ok(ApiCommand::SetThresholds {
                    thrust_threshold,
                    impact_threshold,
                })
            }
            OP_GET_STATUS => {
                if len != 0 {
                    return Err((opcode, ApiStatus::InvalidLength));
                }
                Ok(ApiCommand::GetStatus)
            }
            OP_CLEAR_BONDED_DEVICES => {
                if len != 0 {
                    return Err((opcode, ApiStatus::InvalidLength));
                }
                Ok(ApiCommand::ClearBondedDevices)
            }
            _ => Err((opcode, ApiStatus::InvalidOpcode)),
        }
    }
}

/// Events and responses sent from the sword to the client via `command_control` (`…0006`) notifications.
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApiEvent {
    /// `0x80`: Command acknowledgement with original opcode and status code.
    CommandAck {
        /// Opcode of the command being acknowledged.
        ref_opcode: u8,
        /// Result code from the stable status table.
        status: ApiStatus,
    },
    /// `0x81`: Real-time sword gesture notification with peak acceleration.
    GestureEvent {
        /// Animation identifier from [`LedAnimation::id`].
        animation_id: u8,
        /// Peak absolute acceleration raw count reported for the gesture.
        peak_accel: i16,
    },
    /// `0x82`: Full device state snapshot.
    DeviceStatus {
        /// NFC pairing lifecycle byte.
        pairing_status: u8,
        /// LED mode (`0` automatic, `1` manual).
        led_mode: u8,
        /// Current animation identifier.
        active_animation: u8,
        /// Master brightness (`0..=255`).
        brightness: u8,
        /// Current upward-thrust raw-count threshold.
        thrust_threshold: i16,
        /// Current impact raw-count threshold.
        impact_threshold: i16,
    },
}

impl ApiEvent {
    /// Encode the event into a 16-byte TLV packet.
    pub fn encode(&self) -> [u8; TLV_PACKET_SIZE] {
        let mut buf = [0u8; TLV_PACKET_SIZE];
        match *self {
            ApiEvent::CommandAck { ref_opcode, status } => {
                buf[0] = EVT_COMMAND_ACK;
                buf[1] = 2;
                buf[2] = ref_opcode;
                buf[3] = status.to_u8();
            }
            ApiEvent::GestureEvent {
                animation_id,
                peak_accel,
            } => {
                buf[0] = EVT_GESTURE;
                buf[1] = 3;
                buf[2] = animation_id;
                buf[3..5].copy_from_slice(&peak_accel.to_le_bytes());
            }
            ApiEvent::DeviceStatus {
                pairing_status,
                led_mode,
                active_animation,
                brightness,
                thrust_threshold,
                impact_threshold,
            } => {
                buf[0] = EVT_DEVICE_STATUS;
                buf[1] = 8;
                buf[2] = pairing_status;
                buf[3] = led_mode;
                buf[4] = active_animation;
                buf[5] = brightness;
                buf[6..8].copy_from_slice(&thrust_threshold.to_le_bytes());
                buf[8..10].copy_from_slice(&impact_threshold.to_le_bytes());
            }
        }
        buf
    }

    /// Parse an `ApiEvent` from a raw byte slice.
    pub fn parse(raw: &[u8]) -> Option<Self> {
        if raw.len() < 2 {
            return None;
        }
        let opcode = raw[0];
        let len = raw[1] as usize;
        if len > TLV_MAX_PAYLOAD_LEN || raw.len() < 2 + len {
            return None;
        }
        let payload = &raw[2..2 + len];
        match opcode {
            EVT_COMMAND_ACK if len == 2 => Some(ApiEvent::CommandAck {
                ref_opcode: payload[0],
                status: ApiStatus::from_u8(payload[1])?,
            }),
            EVT_GESTURE if len == 3 => Some(ApiEvent::GestureEvent {
                animation_id: payload[0],
                peak_accel: i16::from_le_bytes([payload[1], payload[2]]),
            }),
            EVT_DEVICE_STATUS if len == 8 => Some(ApiEvent::DeviceStatus {
                pairing_status: payload[0],
                led_mode: payload[1],
                active_animation: payload[2],
                brightness: payload[3],
                thrust_threshold: i16::from_le_bytes([payload[4], payload[5]]),
                impact_threshold: i16::from_le_bytes([payload[6], payload[7]]),
            }),
            _ => None,
        }
    }
}
