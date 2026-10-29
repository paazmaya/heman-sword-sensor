//! Embedded led adapter. Hardware-independent contracts live in the library.

use crate::{ACTIVE_LED_ANIMATION, ACTIVE_LED_BRIGHTNESS, ACTIVE_LED_MODE, LED_CMD_CHANNEL};
use core::sync::atomic::Ordering;
use defmt::{info, warn};
use embassy_nrf::{
    peripherals,
    pwm::{Config as PwmConfig, Prescaler, SequenceConfig, SequencePwm, SingleSequencer},
};
use embassy_time::Timer;
use xiao_nrf52840_sword::led::{
    encode_ws2812_pwm, LedControlMode, LedEngine, LED_FRAME_INTERVAL_MS, PWM_BUFFER_WORDS,
    PWM_MAX_DUTY,
};

/// 50 Hz WS2812B output on exposed D0/P0.02, not the onboard IMU interrupt.
#[embassy_executor::task]
pub(crate) async fn led_task(
    pwm0: embassy_nrf::Peri<'static, peripherals::PWM0>,
    led_pin: embassy_nrf::Peri<'static, peripherals::P0_02>,
) -> ! {
    let mut pwm_config = PwmConfig::default();
    pwm_config.prescaler = Prescaler::Div1; // 16 MHz base clock
    pwm_config.max_duty = PWM_MAX_DUTY; // 20 ticks = 1.25 µs (800 kHz WS2812B bit rate)

    let mut seq_pwm = match SequencePwm::new_1ch(pwm0, led_pin, pwm_config) {
        Ok(pwm) => pwm,
        Err(_) => defmt::panic!("Failed to initialise SequencePwm on PWM0/P0.02"),
    };
    let mut engine = LedEngine::new();
    let mut pwm_words = [0u16; PWM_BUFFER_WORDS];

    info!("💡 WS2812B LED engine initialised on P0.02 (129 pixels @ 50 Hz)");

    loop {
        // Drain any pending LED commands before rendering the next frame
        while let Ok(cmd) = LED_CMD_CHANNEL.try_receive() {
            cmd.apply(&mut engine);
        }

        let frame = engine.tick(LED_FRAME_INTERVAL_MS);
        ACTIVE_LED_MODE.store(
            match engine.control_mode() {
                LedControlMode::AutoMotion => 0,
                LedControlMode::ManualOverride => 1,
            },
            Ordering::Relaxed,
        );
        ACTIVE_LED_BRIGHTNESS.store(engine.brightness(), Ordering::Relaxed);
        ACTIVE_LED_ANIMATION.store(engine.current_animation().id(), Ordering::Relaxed);
        encode_ws2812_pwm(&frame, engine.brightness(), &mut pwm_words);

        {
            let mut seq_cfg = SequenceConfig::default();
            seq_cfg.end_delay = 0;
            let sequencer = SingleSequencer::new(&mut seq_pwm, &pwm_words, seq_cfg);
            if sequencer
                .start(embassy_nrf::pwm::SingleSequenceMode::Times(1))
                .is_err()
            {
                warn!("PWM sequence start failed");
            }
            // Wait for the 20 ms frame interval (50 Hz update rate).
            // The 3136-word DMA transfer completes in ~3.9 ms, well within 20 ms.
            Timer::after_millis(LED_FRAME_INTERVAL_MS as u64).await;
        }
    }
}
