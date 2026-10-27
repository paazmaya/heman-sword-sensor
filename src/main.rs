//! He-Man Sword Sensor – embedded entry point.
//!
//! Hardware: Seeed XIAO nRF52840 Sense
//! - IMU: LSM6DS3TR on TWIM0 (SDA P0.07, SCL P0.27), power P1.08
//! - NFC:  nRF52840 built-in NFCT peripheral (NFC Type 2 Tag)
//! - BLE:  nRF52840 Bluetooth 5.0 via nrf-softdevice S140
//!
//! # Boot sequence
//! 1. Heap + Embassy init
//! 2. I2C + IMU init
//! 3. Softdevice enable + spawn softdevice task
//! 4. NFCT init → 15-second NFC pairing gate
//!    - On selection: read/register a record via SoftDevice flash
//!    - On timeout: open or whitelist-only advertising fallback
//! 5. Spawn BLE advertising + sensor streaming tasks
//! 6. Main loop samples the IMU and processes storage requests

#![no_std]
#![no_main]

mod firmware;

use firmware::{
    ble::{ble_task, softdevice_task, update_pairing_characteristic, Server, SERVER},
    board::{init_i2c, init_imu, read_sensor_data},
    flash::process_storage_request,
    led::led_task,
    nfc::{pairing_gate, LOCAL_NFC_ID},
};

use core::mem;
use core::sync::atomic::{AtomicI16, AtomicU32, AtomicU8, Ordering};

use defmt::{info, unwrap, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::interrupt::InterruptExt;
use embassy_nrf::{
    bind_interrupts,
    config::Config as NrfConfig,
    nfct::{Config as NfcConfig, NfcId, NfcT, SddPat, SelResProtocol},
    peripherals, twim,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use linked_list_allocator::LockedHeap;
use nrf_softdevice::{raw, Flash as SdFlash, Softdevice};
use panic_probe as _;

// Lib re-exports
use xiao_nrf52840_sword::{
    api::{ApiEvent, TLV_PACKET_SIZE},
    control::LedAction as LedCommand,
    detect_sword_gesture_with_thresholds,
    fusion::MadgwickAhrs,
    led::LedAnimation,
    pairing::is_valid_ble_mac,
    validate_sensor_data, SensorData, IMPACT_THRESHOLD, NFC_PAIRING_TIMEOUT_SECS,
    SENSOR_SAMPLING_INTERVAL_MS, THRUST_THRESHOLD,
};

// ============================================================================
// INTERRUPT BINDINGS
// ============================================================================

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    NFCT    => embassy_nrf::nfct::InterruptHandler;
});

// ============================================================================
// GLOBAL ALLOCATOR (4 KB heap)
// ============================================================================

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 4096;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    // Called once at boot, before any task can allocate; HEAP lives forever.
    unsafe {
        ALLOCATOR
            .lock()
            .init(core::ptr::addr_of_mut!(HEAP) as *mut u8, HEAP_SIZE);
    }
}

// Inter-task channel: sensor producer → BLE streaming consumer
static SENSOR_CHANNEL: Signal<CriticalSectionRawMutex, SensorData> = Signal::new();

// Inter-task channel: Madgwick AHRS quaternion Q14 bytes (8 bytes) → BLE streaming consumer
static ORIENTATION_CHANNEL: Signal<CriticalSectionRawMutex, [u8; 8]> = Signal::new();

// Inter-task channel: TLV events/responses → BLE streaming consumer
static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, (u32, [u8; TLV_PACKET_SIZE]), 8> =
    Channel::new();
static GESTURE_CHANNEL: Signal<CriticalSectionRawMutex, [u8; TLV_PACKET_SIZE]> = Signal::new();
static CONNECTION_EPOCH: AtomicU32 = AtomicU32::new(0);

// Inter-task channel: sensor loop & BLE GATT commands → 50 Hz WS2812B `led_task`
static LED_CMD_CHANNEL: Channel<CriticalSectionRawMutex, LedCommand, 8> = Channel::new();

// Shared runtime state for `DeviceStatus` reporting and configurable gesture thresholds
static ACTIVE_THRUST_THRESHOLD: AtomicI16 = AtomicI16::new(THRUST_THRESHOLD);
static ACTIVE_IMPACT_THRESHOLD: AtomicI16 = AtomicI16::new(IMPACT_THRESHOLD);
static ACTIVE_LED_MODE: AtomicU8 = AtomicU8::new(0); // 0 = AutoMotion, 1 = ManualOverride
static ACTIVE_LED_ANIMATION: AtomicU8 = AtomicU8::new(1); // 1 = IdleBreathing
static ACTIVE_LED_BRIGHTNESS: AtomicU8 = AtomicU8::new(255);

/// Enqueue a sword blade LED animation on [`LED_CMD_CHANNEL`].
fn trigger_led_animation(anim: LedAnimation) {
    if LED_CMD_CHANNEL
        .try_send(LedCommand::MotionGesture(anim))
        .is_err()
    {
        warn!("LED command queue full; motion frame dropped");
    }
}

// ============================================================================
// MAIN
// ============================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    init_heap();

    info!("═══════════════════════════════════════════");
    info!("He-Man Sword Sensor – nRF52840 + embassy");
    info!("═══════════════════════════════════════════");

    // --- Embassy-nrf init (must happen before Softdevice::enable) ---
    let mut nrf_config = NrfConfig::default();
    // Softdevice requires these interrupt priorities to be lower than P0
    nrf_config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    nrf_config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    let p = embassy_nrf::init(nrf_config);
    embassy_nrf::interrupt::TWISPI0.set_priority(embassy_nrf::interrupt::Priority::P2);
    embassy_nrf::interrupt::NFCT.set_priority(embassy_nrf::interrupt::Priority::P2);

    spawner.spawn(unwrap!(led_task(p.PWM0, p.P0_02)));

    // --- I2C + IMU ---
    let _imu_power = embassy_nrf::gpio::Output::new(
        p.P1_08,
        embassy_nrf::gpio::Level::High,
        embassy_nrf::gpio::OutputDrive::Standard,
    );
    Timer::after_millis(20).await;
    let i2c = init_i2c(p.TWISPI0, p.P0_07, p.P0_27);
    let mut imu = init_imu(i2c);

    // --- NFCT peripheral ---
    let nfct_config = NfcConfig {
        nfcid1: NfcId::SingleSize(LOCAL_NFC_ID),
        sdd_pat: SddPat::Sdd00000,
        plat_conf: 0x00,
        protocol: SelResProtocol::Type2,
    };
    let mut nfct = NfcT::new(p.NFCT, Irqs, &nfct_config);
    info!("NFCT initialised (NFC Type 2 Tag)");

    // =========================================================================
    // STEP 1 – SOFTDEVICE ENABLE (enabled early so Flash::take is available)
    // =========================================================================
    let sd_config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 1,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 64 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: raw::BLE_GAP_ADV_SET_COUNT_DEFAULT as u8,
            periph_role_count: 1,
            central_role_count: 0,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"He-Man Sword" as *const u8 as _,
            current_len: 12,
            max_len: 12,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    };

    let sd: &'static mut Softdevice = Softdevice::enable(&sd_config);
    let server: &'static mut Server = SERVER.init(unwrap!(Server::new(sd)));

    // Softdevice event loop must run on its own task before any sd_flash operations
    spawner.spawn(unwrap!(softdevice_task(sd)));
    let mut sd_flash = SdFlash::take(sd);

    // =========================================================================
    // STEP 2 – NFC PAIRING GATE (15-second window)
    // =========================================================================
    info!("");
    info!(
        "🔌 NFC pairing gate – {} s window",
        NFC_PAIRING_TIMEOUT_SECS
    );

    let bonded_mac = pairing_gate(&mut nfct, &mut sd_flash).await;

    update_pairing_characteristic(server);

    info!(
        "   BLE mode: {}",
        if bonded_mac.filter(is_valid_ble_mac).is_some() {
            "whitelist"
        } else {
            "open"
        }
    );

    // =========================================================================
    // STEP 3 – SPAWN BLE TASK
    // =========================================================================
    spawner.spawn(unwrap!(ble_task(sd, server, bonded_mac)));

    // =========================================================================
    // STEP 4 – MAIN LOOP: 20 Hz IMU + Madgwick AHRS + runtime flash/whitelist updates
    // =========================================================================
    let mut ahrs = MadgwickAhrs::default();
    let mut ticker = Ticker::every(Duration::from_millis(SENSOR_SAMPLING_INTERVAL_MS));
    let mut previous_sample: Option<Instant> = None;
    info!("📊 Sensor + Madgwick AHRS loop running at 20 Hz");

    loop {
        process_storage_request(&mut sd_flash).await;
        update_pairing_characteristic(server);

        // Run one iteration of 20 Hz sensor sampling + AHRS fusion + LED gesture dispatch
        if let Some(data) = read_sensor_data(&mut imu) {
            if !validate_sensor_data(&data) {
                warn!("⚠️  sensor out of range");
            } else {
                let thrust_thresh = ACTIVE_THRUST_THRESHOLD.load(Ordering::Relaxed);
                let impact_thresh = ACTIVE_IMPACT_THRESHOLD.load(Ordering::Relaxed);

                if let Some(gesture_anim) =
                    detect_sword_gesture_with_thresholds(&data, thrust_thresh, impact_thresh)
                {
                    trigger_led_animation(gesture_anim);
                    let peak = data
                        .accel_x
                        .saturating_abs()
                        .max(data.accel_y.saturating_abs())
                        .max(data.accel_z.saturating_abs());
                    let evt = ApiEvent::GestureEvent {
                        animation_id: gesture_anim.id(),
                        peak_accel: peak,
                    };
                    GESTURE_CHANNEL.signal(evt.encode());
                }

                // Step Madgwick AHRS filter and stream 8-byte Q14 quaternion
                let now = Instant::now();
                let dt_seconds = previous_sample
                    .map_or(SENSOR_SAMPLING_INTERVAL_MS as f32 / 1000.0, |previous| {
                        now.duration_since(previous).as_micros() as f32 / 1_000_000.0
                    });
                previous_sample = Some(now);
                let quat = ahrs.update_from_sensor_data_with_dt(&data, dt_seconds);
                ORIENTATION_CHANNEL.signal(quat.to_q14_bytes());

                SENSOR_CHANNEL.signal(data);
                info!(
                    "A:({},{},{}) G:({},{},{})",
                    data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z
                );
            }
        }

        ticker.next().await;
    }
}
