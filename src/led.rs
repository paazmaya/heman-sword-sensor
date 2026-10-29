//! Deterministic LED animation engine and WS2812 PWM encoding.

/// WS2812B pixels on each side of the blade: 2.1 m of 60 LEDs/m folded at the tip.
pub const BLADE_SIDE_LEDS: usize = 63;
/// Pixels in the separate tip module spliced into the data chain at the fold.
pub const TIP_LEDS: usize = 3;
/// Total pixels in the data chain.
///
/// Chain order: `0..63` guard→tip (up side), `63..66` tip module,
/// `66..129` tip→guard (down side). Blade position `p` (0 = guard) is lit
/// through the mirrored pixel pair `p` and `NUM_LEDS - 1 - p`.
pub const NUM_LEDS: usize = 2 * BLADE_SIDE_LEDS + TIP_LEDS;
/// Index of the first tip-module pixel in the data chain.
pub const TIP_START: usize = BLADE_SIDE_LEDS;
/// Frame update period in milliseconds (50 Hz).
pub const LED_FRAME_INTERVAL_MS: u32 = 20;
/// Number of PWM words per LED (24 bits: 8 G + 8 R + 8 B).
pub const BITS_PER_LED: usize = 24;
/// Number of trailing zero-duty PWM words for the ≥50 µs WS2812B reset latch (40 × 1.25 µs = 50 µs).
pub const RESET_SLOTS: usize = 40;
/// Total length of the `SequencePwm` DMA buffer in 16-bit half-words.
pub const PWM_BUFFER_WORDS: usize = NUM_LEDS * BITS_PER_LED + RESET_SLOTS; // 3136

/// nRF52 PWM countertop for 800 kHz at 16 MHz (`16 MHz / 800 kHz = 20` ticks = 1.25 µs).
pub const PWM_MAX_DUTY: u16 = 20;
/// Bit 15 inverted polarity flag required by nRF52 PWM peripheral so duty value = high time.
pub const PWM_POLARITY_BIT: u16 = 0x8000;
/// WS2812B `'0'` bit duty word: 6 ticks high (0.375 µs), 14 ticks low (0.875 µs).
pub const PWM_T0H_WORD: u16 = 6 | PWM_POLARITY_BIT;
/// WS2812B `'1'` bit duty word: 13 ticks high (0.8125 µs), 7 ticks low (0.4375 µs).
pub const PWM_T1H_WORD: u16 = 13 | PWM_POLARITY_BIT;
/// WS2812B reset latch word: 0 ticks high (continuous low).
pub const PWM_RESET_WORD: u16 = PWM_POLARITY_BIT;

/// Output cap for the white component of every pixel (`0..=255` per channel).
///
/// Pure white lights all three dies and draws 0.2 W (40 mA at 5 V) per pixel at the rated
/// strip spec; capping it at a third limits an all-white frame to about 13 mA per pixel
/// (about 1.7 A for 129 pixels).
pub const WHITE_CAP: u8 = 85;

/// Default upward thrust animation duration (200 ms).
pub const DURATION_THRUST_MS: u32 = 200;
/// Default downward slash animation duration (150 ms).
pub const DURATION_SLASH_MS: u32 = 150;
/// Default impact flash animation duration (100 ms).
pub const DURATION_IMPACT_MS: u32 = 100;
/// Default motion-stop fade duration (1000 ms).
pub const DURATION_MOTION_STOP_MS: u32 = 1000;
/// Full cycle period for continuous idle breathing (2000 ms).
pub const BREATHING_PERIOD_MS: u32 = 2000;

/// 24-bit RGB color value.
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RgbColor {
    /// Red intensity (`0..=255`).
    pub r: u8,
    /// Green intensity (`0..=255`).
    pub g: u8,
    /// Blue intensity (`0..=255`).
    pub b: u8,
}

impl RgbColor {
    /// Black.
    pub const BLACK: Self = Self::new(0, 0, 0);
    /// Full-intensity white.
    pub const WHITE: Self = Self::new(255, 255, 255);
    /// Default cyan/blue idle color.
    pub const GRAYSKULL_BLUE: Self = Self::new(80, 180, 255);
    /// Default orange slash color.
    pub const SLASH_ORANGE: Self = Self::new(255, 70, 0);

    /// Construct an RGB color.
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    /// Scale each channel by `scale` (`0..=255`).
    pub const fn scale(self, scale: u8) -> Self {
        let s = scale as u16;
        Self {
            r: ((self.r as u16 * s) / 255) as u8,
            g: ((self.g as u16 * s) / 255) as u8,
            b: ((self.b as u16 * s) / 255) as u8,
        }
    }

    /// Cap the white component (the part shared by all channels) at `cap`.
    ///
    /// Pure white becomes `(cap, cap, cap)`; saturated colors whose weakest
    /// channel is already `<= cap` (for example orange) are unchanged.
    pub const fn limit_white(self, cap: u8) -> Self {
        let min = if self.r < self.g { self.r } else { self.g };
        let min = if min < self.b { min } else { self.b };
        if min <= cap {
            return self;
        }
        let excess = min - cap;
        Self {
            r: self.r - excess,
            g: self.g - excess,
            b: self.b - excess,
        }
    }
}

/// Sword blade LED animation patterns.
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LedAnimation {
    /// Slow continuous cyan/blue breathing glow when idle.
    IdleBreathing,
    /// Guard-to-tip white/blue energy wave on upward thrust (default 200 ms).
    UpwardThrust,
    /// Tip-to-guard red/orange wave on downward slash (default 150 ms).
    DownwardSlash,
    /// Full-blade bright white impact flash with decay (default 100 ms).
    ImpactHit,
    /// Smooth 1000 ms fade to black, then remain Off.
    MotionStop,
    /// Static user-selected RGB color (e.g. set via BLE API).
    SolidColor(RgbColor),
    /// All LEDs off.
    Off,
}

impl LedAnimation {
    /// Numeric animation identifier used by the BLE TLV protocol.
    pub const fn id(&self) -> u8 {
        match self {
            LedAnimation::Off => 0,
            LedAnimation::IdleBreathing => 1,
            LedAnimation::UpwardThrust => 2,
            LedAnimation::DownwardSlash => 3,
            LedAnimation::ImpactHit => 4,
            LedAnimation::MotionStop => 5,
            LedAnimation::SolidColor(_) => 6,
        }
    }

    /// Construct an animation from a numeric ID and optional color.
    pub const fn from_id(id: u8, color: RgbColor) -> Option<Self> {
        match id {
            0 => Some(LedAnimation::Off),
            1 => Some(LedAnimation::IdleBreathing),
            2 => Some(LedAnimation::UpwardThrust),
            3 => Some(LedAnimation::DownwardSlash),
            4 => Some(LedAnimation::ImpactHit),
            5 => Some(LedAnimation::MotionStop),
            6 => Some(LedAnimation::SolidColor(color)),
            _ => None,
        }
    }

    /// Default duration in milliseconds (`None` for continuous states).
    pub const fn default_duration_ms(&self) -> Option<u32> {
        match self {
            LedAnimation::UpwardThrust => Some(DURATION_THRUST_MS),
            LedAnimation::DownwardSlash => Some(DURATION_SLASH_MS),
            LedAnimation::ImpactHit => Some(DURATION_IMPACT_MS),
            LedAnimation::MotionStop => Some(DURATION_MOTION_STOP_MS),
            LedAnimation::IdleBreathing | LedAnimation::SolidColor(_) | LedAnimation::Off => None,
        }
    }
}

/// Operating mode for the LED engine.
#[cfg_attr(feature = "embedded", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LedControlMode {
    /// Automatically trigger animations from IMU gestures.
    AutoMotion,
    /// Hold manual BLE-configured animation/color and ignore IMU gestures.
    ManualOverride,
}

/// Deterministic animation engine for the folded blade strip and tip module.
#[derive(Clone, Debug)]
pub struct LedEngine {
    active: LedAnimation,
    mode: LedControlMode,
    elapsed_ms: u32,
    duration_ms: Option<u32>,
    brightness: u8,
    last_frame: [RgbColor; NUM_LEDS],
}

impl Default for LedEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl LedEngine {
    /// Create a new `LedEngine` starting in `IdleBreathing` at full brightness (255).
    pub const fn new() -> Self {
        Self {
            active: LedAnimation::IdleBreathing,
            mode: LedControlMode::AutoMotion,
            elapsed_ms: 0,
            duration_ms: None,
            brightness: 255,
            last_frame: [RgbColor::BLACK; NUM_LEDS],
        }
    }

    /// Return the currently active animation.
    pub const fn current_animation(&self) -> LedAnimation {
        self.active
    }

    /// Return the current control mode (`AutoMotion` vs `ManualOverride`).
    pub const fn control_mode(&self) -> LedControlMode {
        self.mode
    }

    /// Set the control mode.
    pub fn set_control_mode(&mut self, mode: LedControlMode) {
        self.mode = mode;
    }

    /// Return the global brightness (`0..=255`).
    pub const fn brightness(&self) -> u8 {
        self.brightness
    }

    /// Set global master brightness (`0..=255`).
    pub fn set_brightness(&mut self, brightness: u8) {
        self.brightness = brightness;
    }

    /// Trigger a motion animation using its default duration.
    ///
    /// Ignored in [`LedControlMode::ManualOverride`]. Repeated triggers of
    /// the currently active timed animation do not restart its clock.
    pub fn trigger_motion(&mut self, anim: LedAnimation) {
        if self.mode == LedControlMode::ManualOverride {
            return;
        }
        if self.active == anim && self.duration_ms.is_some() {
            return;
        }
        self.start_animation(anim, anim.default_duration_ms());
    }

    /// Force-start an animation with an optional custom duration (in ms).
    ///
    /// If `duration_ms` is `Some(0)`, the animation's default duration is used.
    pub fn start_animation(&mut self, anim: LedAnimation, duration_ms: Option<u32>) {
        self.active = anim;
        self.elapsed_ms = 0;
        self.duration_ms = match duration_ms {
            Some(0) => anim.default_duration_ms(),
            Some(d) => Some(d),
            None => anim.default_duration_ms(),
        };
    }

    /// Render the current frame, then advance the animation clock by `dt_ms`.
    ///
    /// A transient animation renders its terminal frame at its duration
    /// before returning to breathing (or Off for MotionStop) on that tick.
    ///
    /// ```
    /// use xiao_nrf52840_sword::led::{LedAnimation, LedEngine, RgbColor, TIP_START};
    /// let mut engine = LedEngine::new();
    /// engine.start_animation(LedAnimation::UpwardThrust, Some(200));
    /// assert_eq!(engine.tick(200)[TIP_START], RgbColor::BLACK);
    /// assert_eq!(engine.tick(20)[TIP_START], RgbColor::WHITE);
    /// assert_eq!(engine.current_animation(), LedAnimation::IdleBreathing);
    /// ```
    pub fn tick(&mut self, dt_ms: u32) -> [RgbColor; NUM_LEDS] {
        let terminal_frame = self
            .duration_ms
            .is_some_and(|limit| self.elapsed_ms >= limit);

        let frame = match self.active {
            LedAnimation::Off => [RgbColor::BLACK; NUM_LEDS],
            LedAnimation::SolidColor(color) => [color; NUM_LEDS],
            LedAnimation::IdleBreathing => self.render_idle_breathing(),
            LedAnimation::UpwardThrust => {
                let dur = self.duration_ms.unwrap_or(DURATION_THRUST_MS).max(1);
                self.render_upward_thrust(dur)
            }
            LedAnimation::DownwardSlash => {
                let dur = self.duration_ms.unwrap_or(DURATION_SLASH_MS).max(1);
                self.render_downward_slash(dur)
            }
            LedAnimation::ImpactHit => {
                let dur = self.duration_ms.unwrap_or(DURATION_IMPACT_MS).max(1);
                self.render_impact_hit(dur)
            }
            LedAnimation::MotionStop => {
                let dur = self.duration_ms.unwrap_or(DURATION_MOTION_STOP_MS).max(1);
                self.render_motion_stop(dur)
            }
        };

        if terminal_frame {
            self.active = if self.active == LedAnimation::MotionStop {
                LedAnimation::Off
            } else {
                LedAnimation::IdleBreathing
            };
            self.elapsed_ms = 0;
            self.duration_ms = None;
        } else {
            self.elapsed_ms = self.elapsed_ms.saturating_add(dt_ms);
            if let Some(limit) = self.duration_ms {
                self.elapsed_ms = self.elapsed_ms.min(limit);
            }
        }
        if terminal_frame || self.active != LedAnimation::MotionStop {
            self.last_frame = frame;
        }
        frame
    }

    fn render_idle_breathing(&self) -> [RgbColor; NUM_LEDS] {
        let phase = self.elapsed_ms % BREATHING_PERIOD_MS;
        let half = BREATHING_PERIOD_MS / 2;
        // Triangle wave oscillating smoothly between intensity 20 and 120
        let triangle = if phase < half {
            phase
        } else {
            BREATHING_PERIOD_MS - phase
        };
        let level = (20 + (triangle * 100) / half) as u8;
        let color = RgbColor::GRAYSKULL_BLUE.scale(level);
        let mut leds = [color; NUM_LEDS];
        clear_tip(&mut leds);
        leds
    }

    fn render_upward_thrust(&self, duration_ms: u32) -> [RgbColor; NUM_LEDS] {
        let mut leds = [RgbColor::BLACK; NUM_LEDS];
        let clamped = self.elapsed_ms.min(duration_ms);
        // Wavefront advances from guard (0) to tip (BLADE_SIDE_LEDS - 1)
        let head = ((clamped as u64) * (BLADE_SIDE_LEDS as u64 - 1)) / (duration_ms as u64);
        for pos in 0..BLADE_SIDE_LEDS {
            if (pos as u64) <= head {
                let dist = head - pos as u64;
                // Bright white tip at wavefront, decaying cyan/blue plasma trail toward guard
                let trail_scale = 255u16.saturating_sub((dist as u16) * 4) as u8;
                let color = if dist <= 3 {
                    RgbColor::WHITE.scale(trail_scale)
                } else {
                    RgbColor::GRAYSKULL_BLUE.scale(trail_scale)
                };
                set_blade_pair(&mut leds, pos, color);
            }
        }
        // Tip module ramps up to full white as the beam reaches the end of the blade
        let tip_scale = ((clamped as u64) * 255 / (duration_ms as u64)) as u8;
        leds[TIP_START..TIP_START + TIP_LEDS].fill(RgbColor::WHITE.scale(tip_scale));
        leds
    }

    fn render_downward_slash(&self, duration_ms: u32) -> [RgbColor; NUM_LEDS] {
        let mut leds = [RgbColor::BLACK; NUM_LEDS];
        let clamped = self.elapsed_ms.min(duration_ms);
        // Reverse wave: starts at tip (BLADE_SIDE_LEDS - 1) and sweeps down to guard (0)
        let progress = ((clamped as u64) * (BLADE_SIDE_LEDS as u64 - 1)) / (duration_ms as u64);
        let head = (BLADE_SIDE_LEDS as u64 - 1).saturating_sub(progress);
        for pos in 0..BLADE_SIDE_LEDS {
            if (pos as u64) >= head {
                let dist = pos as u64 - head;
                let trail_scale = 255u16.saturating_sub((dist as u16) * 5) as u8;
                set_blade_pair(&mut leds, pos, RgbColor::SLASH_ORANGE.scale(trail_scale));
            }
        }
        leds
    }

    fn render_impact_hit(&self, duration_ms: u32) -> [RgbColor; NUM_LEDS] {
        let clamped = self.elapsed_ms.min(duration_ms);
        let remaining = duration_ms.saturating_sub(clamped);
        // Quadratic decay from full white flash (255) to 0
        let linear = (remaining as u64 * 255 / duration_ms as u64) as u16;
        let scale = ((linear * linear) / 255) as u8;
        let color = RgbColor::WHITE.scale(scale);
        [color; NUM_LEDS]
    }

    fn render_motion_stop(&self, duration_ms: u32) -> [RgbColor; NUM_LEDS] {
        let clamped = self.elapsed_ms.min(duration_ms);
        let remaining = duration_ms.saturating_sub(clamped);
        let scale = ((remaining as u64 * 255) / duration_ms as u64) as u8;
        let mut leds = [RgbColor::BLACK; NUM_LEDS];
        for (i, pixel) in leds.iter_mut().enumerate() {
            let is_tip = (TIP_START..TIP_START + TIP_LEDS).contains(&i);
            let base = if self.last_frame[i] == RgbColor::BLACK && !is_tip {
                RgbColor::GRAYSKULL_BLUE.scale(80)
            } else {
                self.last_frame[i]
            };
            *pixel = base.scale(scale);
        }
        leds
    }
}

/// Light blade position `pos` (0 = guard) on both sides of the folded strip.
fn set_blade_pair(leds: &mut [RgbColor; NUM_LEDS], pos: usize, color: RgbColor) {
    leds[pos] = color;
    leds[NUM_LEDS - 1 - pos] = color;
}

/// Turn the tip module off.
fn clear_tip(leds: &mut [RgbColor; NUM_LEDS]) {
    leds[TIP_START..TIP_START + TIP_LEDS].fill(RgbColor::BLACK);
}

/// Encode an `NUM_LEDS` RGB frame into nRF52840 `SequencePwm` 16-bit duty words.
///
/// - WS2812B expects **GRB** byte order, MSB first (24 bits per pixel).
/// - Caps each pixel's white component at [`WHITE_CAP`], then applies global `brightness` (`0..=255`) scaling before encoding.
/// - Appends [`RESET_SLOTS`] words of [`PWM_RESET_WORD`] (`0x8000`) for the ≥50 µs latch.
pub fn encode_ws2812_pwm(
    leds: &[RgbColor; NUM_LEDS],
    brightness: u8,
    out: &mut [u16; PWM_BUFFER_WORDS],
) {
    let mut word_idx = 0usize;
    for pixel in leds.iter() {
        let scaled = pixel.limit_white(WHITE_CAP).scale(brightness);
        let grb = [scaled.g, scaled.r, scaled.b];
        for byte in grb {
            for bit in (0..8).rev() {
                let is_one = (byte >> bit) & 1 != 0;
                out[word_idx] = if is_one { PWM_T1H_WORD } else { PWM_T0H_WORD };
                word_idx += 1;
            }
        }
    }
    while word_idx < PWM_BUFFER_WORDS {
        out[word_idx] = PWM_RESET_WORD;
        word_idx += 1;
    }
}
