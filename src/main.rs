//! He-Man Sword Sensor – embedded entry point.
//!
//! Hardware: Seeed XIAO nRF52840 Sense
//! - IMU:  LSM6DS3TR on TWIM0 (SDA P0.26, SCL P0.27)
//! - NFC:  nRF52840 built-in NFCT peripheral (NFC Type 2 Tag)
//! - BLE:  nRF52840 Bluetooth 5.0 via nrf-softdevice S140
//!
//! # Boot sequence
//! 1. Heap + Embassy init
//! 2. I2C + IMU init
//! 3. Softdevice enable + spawn softdevice task
//! 4. NFCT init → 15-second NFC pairing gate
//!    - On success: read/register bonded device in NVMC flash
//!    - On timeout: open or whitelist-only advertising fallback
//! 5. Spawn BLE advertising + sensor streaming tasks
//! 6. Main loop monitors for new NFC taps (re-pairing)

#![no_std]
#![no_main]

use core::mem;
use core::sync::atomic::{AtomicI16, AtomicU8, Ordering};

use defmt::{info, unwrap, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::{
    bind_interrupts,
    config::Config as NrfConfig,
    nfct::{Config as NfcConfig, NfcId, NfcT, SddPat, SelResProtocol},
    peripherals,
    pwm::{Config as PwmConfig, Prescaler, SequenceConfig, SequencePwm, SingleSequencer},
    twim::{self, Twim},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_storage_async::nor_flash::NorFlash as AsyncNorFlash;
use linked_list_allocator::LockedHeap;
use lsm6ds3tr::interface::I2cInterface;
use lsm6ds3tr::LSM6DS3TR;
use nrf_softdevice::ble::advertisement_builder::{
    Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList, ServiceUuid16,
};
use nrf_softdevice::ble::{gatt_server, peripheral, Address, AddressType, Connection};
use nrf_softdevice::{raw, Flash as SdFlash, Softdevice};
use panic_probe as _;
use static_cell::{ConstStaticCell, StaticCell};

// Lib re-exports
use xiao_nrf52840_sword::{
    api::{ApiCommand, ApiEvent, ApiStatus, TLV_PACKET_SIZE},
    detect_sword_gesture_with_thresholds,
    fusion::MadgwickAhrs,
    get_nfc_pairing_status,
    led::{
        encode_ws2812_pwm, LedAnimation, LedControlMode, LedEngine, RgbColor,
        LED_FRAME_INTERVAL_MS, PWM_BUFFER_WORDS, PWM_MAX_DUTY,
    },
    nfc::{parse_nfct_uid, store_activated_uid},
    pairing::{
        is_valid_ble_mac, make_flash_record, parse_flash_record, read_bonded_device,
        FLASH_BONDED_DEVICE_START, FLASH_RECORD_SIZE, PLACEHOLDER_MAC,
    },
    set_nfc_pairing_status, validate_sensor_data, BondedDevice, NfcPairingStatus,
    SensorData as LibSensorData, IMPACT_THRESHOLD, NFC_PAIRING_TIMEOUT_SECS,
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
    unsafe {
        ALLOCATOR
            .lock()
            .init(core::ptr::addr_of_mut!(HEAP) as *mut u8, HEAP_SIZE);
    }
}

// ============================================================================
// SENSOR DATA  (embedded version — adds defmt::Format)
// ============================================================================

/// Sensor data packet – 12 bytes, identical layout to lib::SensorData.
#[repr(C)]
#[derive(Clone, Copy, defmt::Format)]
struct SensorData {
    accel_x: i16,
    accel_y: i16,
    accel_z: i16,
    gyro_x: i16,
    gyro_y: i16,
    gyro_z: i16,
}

impl SensorData {
    /// Serialise to little-endian bytes for the BLE characteristic value.
    fn to_bytes(self) -> [u8; 12] {
        let mut b = [0u8; 12];
        b[0..2].copy_from_slice(&self.accel_x.to_le_bytes());
        b[2..4].copy_from_slice(&self.accel_y.to_le_bytes());
        b[4..6].copy_from_slice(&self.accel_z.to_le_bytes());
        b[6..8].copy_from_slice(&self.gyro_x.to_le_bytes());
        b[8..10].copy_from_slice(&self.gyro_y.to_le_bytes());
        b[10..12].copy_from_slice(&self.gyro_z.to_le_bytes());
        b
    }
}

// Inter-task channel: sensor producer → BLE streaming consumer
static SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorData, 16> = Channel::new();

// Inter-task channel: Madgwick AHRS quaternion Q14 bytes (8 bytes) → BLE streaming consumer
static ORIENTATION_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 8], 16> = Channel::new();

// Inter-task channel: TLV events/responses → BLE streaming consumer
static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, [u8; TLV_PACKET_SIZE], 8> = Channel::new();

/// Internal command message sent from the sensor loop or BLE API to `led_task`.
#[derive(Clone, Copy)]
enum LedCommand {
    /// Trigger a motion animation (ignored if `LedEngine` is in `ManualOverride`).
    MotionGesture(LedAnimation),
    /// Set control mode, color, and optional duration (`0` = default/continuous).
    SetMode {
        mode: LedControlMode,
        color: RgbColor,
        duration_ms: u16,
    },
    /// Force-start a specific animation with an optional duration (`0` = default).
    Trigger {
        animation: LedAnimation,
        duration_ms: u16,
    },
    /// Update master brightness (`0..=255`).
    SetBrightness(u8),
}

// Inter-task channel: sensor loop & BLE GATT commands → 50 Hz WS2812B `led_task`
static LED_CMD_CHANNEL: Channel<CriticalSectionRawMutex, LedCommand, 8> = Channel::new();

// Shared runtime state for `DeviceStatus` reporting and configurable gesture thresholds
static ACTIVE_THRUST_THRESHOLD: AtomicI16 = AtomicI16::new(THRUST_THRESHOLD);
static ACTIVE_IMPACT_THRESHOLD: AtomicI16 = AtomicI16::new(IMPACT_THRESHOLD);
static ACTIVE_LED_MODE: AtomicU8 = AtomicU8::new(0); // 0 = AutoMotion, 1 = ManualOverride
static ACTIVE_LED_ANIMATION: AtomicU8 = AtomicU8::new(1); // 1 = IdleBreathing
static ACTIVE_LED_BRIGHTNESS: AtomicU8 = AtomicU8::new(255);

// ============================================================================
// GATT SERVER DEFINITION
// ============================================================================

// Custom 128-bit service UUID: he-man-sword-sensor
//   Service UUID:          6e400001-b5a3-f393-e0a9-e50e24dcca9e
//   Sensor data char:      6e400003-b5a3-f393-e0a9-e50e24dcca9e  (read/notify, 12B)
//   Pairing status char:   6e400004-b5a3-f393-e0a9-e50e24dcca9e  (read/notify, 1B)
//   Pairing MAC char:      6e400005-b5a3-f393-e0a9-e50e24dcca9e  (write, 6B)
//   Command/Control char:  6e400006-b5a3-f393-e0a9-e50e24dcca9e  (write/notify, 16B TLV)
//   Orientation data char: 6e400007-b5a3-f393-e0a9-e50e24dcca9e  (read/notify, 8B Q14 quat)

/// GATT service for He-Man Sword Sensor.
#[nrf_softdevice::gatt_service(uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e")]
struct SwordSensorService {
    /// 12-byte sensor data (accel XYZ + gyro XYZ, i16 LE each), read + notify at 20 Hz.
    #[characteristic(uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e", read, notify)]
    sensor_data: [u8; 12],

    /// 1-byte pairing status code (0=Idle, 1=Scanning, 2=Auth, 3=Success, 4=Failed, 5=Timeout).
    #[characteristic(uuid = "6e400004-b5a3-f393-e0a9-e50e24dcca9e", read, notify)]
    pairing_status: u8,

    /// 6-byte BLE MAC written by the mobile app during first-time pairing.
    #[characteristic(uuid = "6e400005-b5a3-f393-e0a9-e50e24dcca9e", write)]
    pairing_mac: [u8; 6],

    /// 16-byte TLV Command/Event endpoint (`[opcode, len, payload...]`).
    #[characteristic(uuid = "6e400006-b5a3-f393-e0a9-e50e24dcca9e", write, notify)]
    command_control: [u8; 16],

    /// 8-byte Q14 quaternion orientation (`[w, x, y, z]` as `i16` LE, scaled by 16384), read + notify at 20 Hz.
    #[characteristic(uuid = "6e400007-b5a3-f393-e0a9-e50e24dcca9e", read, notify)]
    orientation_data: [u8; 8],
}

/// Top-level GATT server.
#[nrf_softdevice::gatt_server]
struct Server {
    swd: SwordSensorService,
}

// ============================================================================
// STATIC CELL FOR GATT SERVER
// ============================================================================

static SERVER: StaticCell<Server> = StaticCell::new();

// ============================================================================
// EMBASSY TASKS
// ============================================================================

/// Drive the nrf-softdevice event loop.  Must run on its own task.
#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

// Channel for the mobile app to update the bonded MAC from ble_task → main.
// Capacity 1 is enough: only one MAC update is expected per pairing session.
static MAC_UPDATE_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 6], 1> = Channel::new();

// Channel for clearing bonded devices from ble_task → main.
static CLEAR_BONDS_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

// Signal from main → ble_task to hot-reload the BLE GAP whitelist without rebooting.
static WHITELIST_UPDATE_SIGNAL: Signal<CriticalSectionRawMutex, Option<[u8; 6]>> = Signal::new();

/// Construct the current `ApiEvent::DeviceStatus` snapshot from shared atomics.
fn build_device_status_event() -> ApiEvent {
    ApiEvent::DeviceStatus {
        pairing_status: get_nfc_pairing_status().to_u8(),
        led_mode: ACTIVE_LED_MODE.load(Ordering::Relaxed),
        active_animation: ACTIVE_LED_ANIMATION.load(Ordering::Relaxed),
        brightness: ACTIVE_LED_BRIGHTNESS.load(Ordering::Relaxed),
        thrust_threshold: ACTIVE_THRUST_THRESHOLD.load(Ordering::Relaxed),
        impact_threshold: ACTIVE_IMPACT_THRESHOLD.load(Ordering::Relaxed),
    }
}

/// Process an incoming TLV command write on `command_control` (`…0006`).
fn handle_tlv_command(raw_packet: &[u8; TLV_PACKET_SIZE]) {
    match ApiCommand::parse(raw_packet) {
        Ok(cmd) => {
            let opcode = cmd.opcode();
            match cmd {
                ApiCommand::SetLedMode {
                    mode,
                    color,
                    duration_ms,
                } => {
                    let mode_u8 = match mode {
                        LedControlMode::AutoMotion => 0,
                        LedControlMode::ManualOverride => 1,
                    };
                    ACTIVE_LED_MODE.store(mode_u8, Ordering::Relaxed);
                    let _ = LED_CMD_CHANNEL.try_send(LedCommand::SetMode {
                        mode,
                        color,
                        duration_ms,
                    });
                    let ack = ApiEvent::CommandAck {
                        ref_opcode: opcode,
                        status: ApiStatus::Ok,
                    };
                    let _ = EVENT_CHANNEL.try_send(ack.encode());
                }
                ApiCommand::SetBrightness { brightness } => {
                    ACTIVE_LED_BRIGHTNESS.store(brightness, Ordering::Relaxed);
                    let _ = LED_CMD_CHANNEL.try_send(LedCommand::SetBrightness(brightness));
                    let ack = ApiEvent::CommandAck {
                        ref_opcode: opcode,
                        status: ApiStatus::Ok,
                    };
                    let _ = EVENT_CHANNEL.try_send(ack.encode());
                }
                ApiCommand::TriggerAnimation {
                    animation,
                    duration_ms,
                } => {
                    ACTIVE_LED_ANIMATION.store(animation.id(), Ordering::Relaxed);
                    let _ = LED_CMD_CHANNEL.try_send(LedCommand::Trigger {
                        animation,
                        duration_ms,
                    });
                    let ack = ApiEvent::CommandAck {
                        ref_opcode: opcode,
                        status: ApiStatus::Ok,
                    };
                    let _ = EVENT_CHANNEL.try_send(ack.encode());
                }
                ApiCommand::SetThresholds {
                    thrust_threshold,
                    impact_threshold,
                } => {
                    ACTIVE_THRUST_THRESHOLD.store(thrust_threshold, Ordering::Relaxed);
                    ACTIVE_IMPACT_THRESHOLD.store(impact_threshold, Ordering::Relaxed);
                    let ack = ApiEvent::CommandAck {
                        ref_opcode: opcode,
                        status: ApiStatus::Ok,
                    };
                    let _ = EVENT_CHANNEL.try_send(ack.encode());
                }
                ApiCommand::GetStatus => {
                    let ack = ApiEvent::CommandAck {
                        ref_opcode: opcode,
                        status: ApiStatus::Ok,
                    };
                    let _ = EVENT_CHANNEL.try_send(ack.encode());
                    let status_evt = build_device_status_event();
                    let _ = EVENT_CHANNEL.try_send(status_evt.encode());
                }
                ApiCommand::ClearBondedDevices => {
                    let status = if CLEAR_BONDS_CHANNEL.try_send(()).is_ok() {
                        ApiStatus::Ok
                    } else {
                        ApiStatus::StorageError
                    };
                    let ack = ApiEvent::CommandAck {
                        ref_opcode: opcode,
                        status,
                    };
                    let _ = EVENT_CHANNEL.try_send(ack.encode());
                }
            }
        }
        Err((opcode, err_status)) => {
            warn!(
                "⚠️  Invalid TLV command (opcode=0x{:02x}, err={:?})",
                opcode, err_status
            );
            let ack = ApiEvent::CommandAck {
                ref_opcode: opcode,
                status: err_status,
            };
            let _ = EVENT_CHANNEL.try_send(ack.encode());
        }
    }
}

/// BLE peripheral task: advertise → accept connection → stream sensor/orientation/events.
///
/// Supports hot-reloading `bonded_mac` via [`WHITELIST_UPDATE_SIGNAL`] without rebooting.
#[embassy_executor::task]
async fn ble_task(
    sd: &'static Softdevice,
    server: &'static Server,
    mut bonded_mac: Option<[u8; 6]>,
) {
    // Build advertising payload
    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_16(ServiceList::Incomplete, &[ServiceUuid16::BATTERY])
        .full_name("He-Man Sword")
        .build();

    static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .services_128(
            ServiceList::Complete,
            &[0x6e400001_b5a3_f393_e0a9_e50e24dcca9e_u128.to_le_bytes()],
        )
        .build();

    info!("📡 BLE advertising – device: \"He-Man Sword\"");

    loop {
        // Apply any pending hot-reloaded whitelist update before advertising
        if let Some(updated_mac) = WHITELIST_UPDATE_SIGNAL.try_take() {
            info!(
                "🔄 BLE task applying updated whitelist MAC: {:?}",
                updated_mac
            );
            bonded_mac = updated_mac;
        }

        // Only enable GAP whitelist filtering when the MAC is a real central MAC
        // (not the first-time pairing placeholder).
        let whitelist_mac = bonded_mac.filter(is_valid_ble_mac);

        let mut config = peripheral::Config::default();
        if whitelist_mac.is_some() {
            config.interval = 32; // 32 × 0.625 ms = 20 ms fast reconnect
        }

        // Sync initial pairing status characteristic with shared state
        let _ = server
            .swd
            .pairing_status_set(&get_nfc_pairing_status().to_u8());

        let conn: Connection = match whitelist_mac {
            Some(mac) => {
                info!("   Whitelist advertising for MAC: {:?}", mac);
                let addr = Address::new(AddressType::RandomStatic, mac);
                if let Err(e) = nrf_softdevice::ble::set_whitelist(sd, &[addr]) {
                    warn!("Failed to set BLE whitelist: {:?}", e);
                }
                config.filter_policy = peripheral::FilterPolicy::Both;
                let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
                    adv_data: &ADV_DATA,
                    scan_data: &SCAN_DATA,
                };
                match select(
                    peripheral::advertise_connectable(sd, adv, &config),
                    WHITELIST_UPDATE_SIGNAL.wait(),
                )
                .await
                {
                    Either::First(Ok(c)) => c,
                    Either::First(Err(e)) => {
                        warn!("BLE advertise error (whitelist): {:?}", e);
                        Timer::after_millis(500).await;
                        continue;
                    }
                    Either::Second(new_mac) => {
                        info!(
                            "🔄 Whitelist hot-reloaded during advertising: {:?}",
                            new_mac
                        );
                        bonded_mac = new_mac;
                        continue;
                    }
                }
            }
            None => {
                info!("   Open advertising (no whitelist restriction)");
                let _ = nrf_softdevice::ble::set_whitelist(sd, &[]);
                config.filter_policy = peripheral::FilterPolicy::Any;
                let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
                    adv_data: &ADV_DATA,
                    scan_data: &SCAN_DATA,
                };
                match select(
                    peripheral::advertise_connectable(sd, adv, &config),
                    WHITELIST_UPDATE_SIGNAL.wait(),
                )
                .await
                {
                    Either::First(Ok(c)) => c,
                    Either::First(Err(e)) => {
                        warn!("BLE advertise error (open): {:?}", e);
                        Timer::after_millis(500).await;
                        continue;
                    }
                    Either::Second(new_mac) => {
                        info!(
                            "🔄 Whitelist hot-reloaded during open advertising: {:?}",
                            new_mac
                        );
                        bonded_mac = new_mac;
                        continue;
                    }
                }
            }
        };

        info!("✅ BLE central connected");
        set_nfc_pairing_status(NfcPairingStatus::Success);
        let _ = server
            .swd
            .pairing_status_set(&NfcPairingStatus::Success.to_u8());

        // Run GATT server and sensor/orientation/event streaming concurrently until disconnect
        let stream_fut = stream_sensor_data_ble(server, &conn);
        let gatt_fut = gatt_server::run(&conn, server, |e| match e {
            ServerEvent::Swd(e) => match e {
                SwordSensorServiceEvent::SensorDataCccdWrite { notifications } => {
                    info!("sensor_data notifications: {}", notifications);
                }
                SwordSensorServiceEvent::PairingStatusCccdWrite { notifications } => {
                    info!("pairing_status notifications: {}", notifications);
                }
                SwordSensorServiceEvent::CommandControlCccdWrite { notifications } => {
                    info!("command_control notifications: {}", notifications);
                }
                SwordSensorServiceEvent::OrientationDataCccdWrite { notifications } => {
                    info!("orientation_data notifications: {}", notifications);
                }
                SwordSensorServiceEvent::PairingMacWrite(value) => {
                    info!("📲 pairing_mac written by central: {:?}", value);
                    if is_valid_ble_mac(&value) {
                        if MAC_UPDATE_CHANNEL.try_send(value).is_err() {
                            warn!("MAC_UPDATE_CHANNEL full – MAC update dropped");
                        }
                    } else {
                        warn!("⚠️  Rejected invalid pairing_mac write: {:?}", value);
                    }
                }
                SwordSensorServiceEvent::CommandControlWrite(raw_cmd) => {
                    info!("🎛️  command_control TLV write: {:02x}", raw_cmd);
                    handle_tlv_command(&raw_cmd);
                }
            },
        });
        select(stream_fut, gatt_fut).await;

        info!("🔌 BLE central disconnected – restarting advertising");
        set_nfc_pairing_status(NfcPairingStatus::Idle);
        let _ = server
            .swd
            .pairing_status_set(&NfcPairingStatus::Idle.to_u8());
    }
}

/// Drain SENSOR_CHANNEL, ORIENTATION_CHANNEL, and EVENT_CHANNEL and notify the connected BLE central.
async fn stream_sensor_data_ble(server: &Server, conn: &Connection) {
    loop {
        match select(
            select(SENSOR_CHANNEL.receive(), ORIENTATION_CHANNEL.receive()),
            EVENT_CHANNEL.receive(),
        )
        .await
        {
            Either::First(Either::First(data)) => {
                let bytes = data.to_bytes();
                if let Err(e) = server.swd.sensor_data_notify(conn, &bytes) {
                    let _ = server.swd.sensor_data_set(&bytes);
                    info!("BLE sensor_data notify skipped: {:?}", e);
                }
            }
            Either::First(Either::Second(quat_bytes)) => {
                if let Err(e) = server.swd.orientation_data_notify(conn, &quat_bytes) {
                    let _ = server.swd.orientation_data_set(&quat_bytes);
                    info!("BLE orientation_data notify skipped: {:?}", e);
                }
            }
            Either::Second(tlv_event) => {
                if let Err(e) = server.swd.command_control_notify(conn, &tlv_event) {
                    let _ = server.swd.command_control_set(&tlv_event);
                    info!("BLE command_control notify skipped: {:?}", e);
                }
            }
        }
    }
}

/// 50 Hz WS2812B LED animation task driving 60 addressable RGB LEDs on P0.11 via `PWM0`.
#[embassy_executor::task]
async fn led_task(
    pwm0: embassy_nrf::Peri<'static, peripherals::PWM0>,
    led_pin: embassy_nrf::Peri<'static, peripherals::P0_11>,
) -> ! {
    let mut pwm_config = PwmConfig::default();
    pwm_config.prescaler = Prescaler::Div1; // 16 MHz base clock
    pwm_config.max_duty = PWM_MAX_DUTY; // 20 ticks = 1.25 µs (800 kHz WS2812B bit rate)

    let mut seq_pwm = match SequencePwm::new_1ch(pwm0, led_pin, pwm_config) {
        Ok(pwm) => pwm,
        Err(_) => defmt::panic!("Failed to initialise SequencePwm on PWM0/P0.11"),
    };
    let mut engine = LedEngine::new();
    let mut pwm_words = [0u16; PWM_BUFFER_WORDS];

    info!("💡 WS2812B LED engine initialised on P0.11 (60 LEDs @ 50 Hz)");

    loop {
        // Drain any pending LED commands before rendering the next frame
        while let Ok(cmd) = LED_CMD_CHANNEL.try_receive() {
            match cmd {
                LedCommand::MotionGesture(anim) => {
                    engine.trigger_motion(anim);
                }
                LedCommand::SetMode {
                    mode,
                    color,
                    duration_ms,
                } => {
                    engine.set_control_mode(mode);
                    match mode {
                        LedControlMode::ManualOverride => {
                            let dur = if duration_ms == 0 {
                                None
                            } else {
                                Some(duration_ms as u32)
                            };
                            engine.start_animation(LedAnimation::SolidColor(color), dur);
                        }
                        LedControlMode::AutoMotion => {
                            engine.start_animation(LedAnimation::IdleBreathing, None);
                        }
                    }
                }
                LedCommand::Trigger {
                    animation,
                    duration_ms,
                } => {
                    let dur = if duration_ms == 0 {
                        None
                    } else {
                        Some(duration_ms as u32)
                    };
                    engine.start_animation(animation, dur);
                }
                LedCommand::SetBrightness(b) => {
                    engine.set_brightness(b);
                }
            }
        }

        let frame = engine.tick(LED_FRAME_INTERVAL_MS);
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
            // The 1480-word DMA transfer completes in ~1.85 ms, well within 20 ms.
            Timer::after_millis(LED_FRAME_INTERVAL_MS as u64).await;
        }
    }
}

// ============================================================================
// I2C / IMU INITIALISATION
// ============================================================================

fn init_i2c(
    twim0: embassy_nrf::Peri<'static, peripherals::TWISPI0>,
    sda: embassy_nrf::Peri<'static, peripherals::P0_26>,
    scl: embassy_nrf::Peri<'static, peripherals::P0_27>,
) -> I2cInterface<Twim<'static>> {
    let i2c_config = twim::Config::default();
    static RAM_BUFFER: ConstStaticCell<[u8; 255]> = ConstStaticCell::new([0; 255]);
    let i2c = twim::Twim::new(twim0, Irqs, sda, scl, i2c_config, RAM_BUFFER.take());
    info!("I2C on TWISPI0 – SDA P0.26, SCL P0.27, 100 kHz");
    I2cInterface::new(i2c)
}

fn init_imu(i2c: I2cInterface<Twim<'static>>) -> LSM6DS3TR<I2cInterface<Twim<'static>>> {
    let imu = LSM6DS3TR::new(i2c);
    info!("IMU LSM6DS3TR @ 0x6A");
    imu
}

// ============================================================================
// SENSOR READING
// ============================================================================

fn read_sensor_data(imu: &mut LSM6DS3TR<I2cInterface<Twim<'static>>>) -> Option<SensorData> {
    match imu.read_accel_raw() {
        Ok(accel) => match imu.read_gyro_raw() {
            Ok(gyro) => Some(SensorData {
                accel_x: accel.x,
                accel_y: accel.y,
                accel_z: accel.z,
                gyro_x: gyro.x,
                gyro_y: gyro.y,
                gyro_z: gyro.z,
            }),
            Err(_) => {
                warn!("gyro read failed");
                None
            }
        },
        Err(_) => {
            warn!("accel read failed");
            None
        }
    }
}

// ============================================================================
// LED ANIMATION TRIGGER
// ============================================================================

/// Enqueue a sword blade LED animation on [`LED_CMD_CHANNEL`].
fn trigger_led_animation(anim: LedAnimation) {
    let _ = LED_CMD_CHANNEL.try_send(LedCommand::MotionGesture(anim));
}

// ============================================================================
// NFC FIELD DETECTION
// ============================================================================

/// Wait for an NFC field using the real embassy-nrf NFCT peripheral.
/// Returns `true` if a field was detected within `timeout_ms` milliseconds.
async fn detect_nfc_field(nfct: &mut NfcT<'static>, timeout_ms: u64) -> bool {
    info!("📡 NFC scan – timeout {} ms", timeout_ms);
    set_nfc_pairing_status(NfcPairingStatus::Scanning);
    let result =
        embassy_time::with_timeout(Duration::from_millis(timeout_ms), nfct.activate()).await;
    match result {
        Ok(_) => {
            info!("✅ NFC field detected");
            set_nfc_pairing_status(NfcPairingStatus::Authenticating);
            true
        }
        Err(_) => {
            info!("⏱️  NFC timeout");
            set_nfc_pairing_status(NfcPairingStatus::Timeout);
            false
        }
    }
}

// ============================================================================
// SOFTDEVICE FLASH & MEMORY-MAPPED READ OPERATIONS
// ============================================================================

/// Read the bonded device + 10-byte UID directly from memory-mapped flash.
///
/// Flash is mapped at `0x0000_0000` on nRF52840, so reads can be performed
/// synchronously via `read_volatile` without disturbing the SoftDevice radio.
fn flash_read_bonded_device_full() -> Option<(BondedDevice, [u8; 10])> {
    const FULL: usize = FLASH_RECORD_SIZE + 10;
    let addr = FLASH_BONDED_DEVICE_START as *const u8;
    let mut buf = [0u8; FULL];
    unsafe {
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = addr.add(i).read_volatile();
        }
    }
    let mut record = [0u8; FLASH_RECORD_SIZE];
    record.copy_from_slice(&buf[..FLASH_RECORD_SIZE]);
    let dev = parse_flash_record(&record)?;
    let mut uid = [0u8; 10];
    uid.copy_from_slice(&buf[FLASH_RECORD_SIZE..]);
    Some((dev, uid))
}

/// Write one bonded device record to flash using the SoftDevice Flash API
/// (`sd_flash_page_erase` + `sd_flash_write` via [`nrf_softdevice::Flash`]).
async fn sd_write_bonded_device(flash: &mut SdFlash, device: &BondedDevice) -> bool {
    let record = make_flash_record(device);
    const PADDED: usize = (FLASH_RECORD_SIZE + 3) & !3;
    let mut padded = [0xFFu8; PADDED];
    padded[..FLASH_RECORD_SIZE].copy_from_slice(&record);

    let page_start = FLASH_BONDED_DEVICE_START & !0xFFF;
    let page_end = page_start + 4096;
    if flash.erase(page_start, page_end).await.is_err() {
        warn!("SoftDevice flash erase failed");
        return false;
    }

    match flash
        .write(FLASH_BONDED_DEVICE_START, &padded[..PADDED])
        .await
    {
        Ok(()) => {
            info!(
                "💾 SoftDevice flash write OK – MAC {:?} @ 0x{:08X}",
                device.mac, FLASH_BONDED_DEVICE_START
            );
            true
        }
        Err(_) => {
            warn!("SoftDevice flash write failed");
            false
        }
    }
}

/// Write a full bonded device record (MAC + 10-byte NFC UID) to flash via SoftDevice Flash API.
async fn sd_write_bonded_device_full(
    flash: &mut SdFlash,
    device: &BondedDevice,
    uid: &[u8; 10],
) -> bool {
    let record = make_flash_record(device);
    const RAW: usize = FLASH_RECORD_SIZE + 10; // 22
    const PADDED: usize = (RAW + 3) & !3; // 24
    let mut buf = [0xFFu8; PADDED];
    buf[..FLASH_RECORD_SIZE].copy_from_slice(&record);
    buf[FLASH_RECORD_SIZE..FLASH_RECORD_SIZE + 10].copy_from_slice(uid);

    let page_start = FLASH_BONDED_DEVICE_START & !0xFFF;
    let page_end = page_start + 4096;
    if flash.erase(page_start, page_end).await.is_err() {
        warn!("SoftDevice flash erase failed (full record)");
        return false;
    }

    match flash.write(FLASH_BONDED_DEVICE_START, &buf[..PADDED]).await {
        Ok(()) => {
            info!(
                "💾 SoftDevice flash write OK (full) – MAC {:?} UID {:?} @ 0x{:08X}",
                device.mac, uid, FLASH_BONDED_DEVICE_START
            );
            true
        }
        Err(_) => {
            warn!("SoftDevice flash write failed (full record)");
            false
        }
    }
}

/// Erase the bonded-device flash page via SoftDevice Flash API.
async fn sd_clear_bonded_devices(flash: &mut SdFlash) -> bool {
    let page_start = FLASH_BONDED_DEVICE_START & !0xFFF;
    let page_end = page_start + 4096;
    match flash.erase(page_start, page_end).await {
        Ok(()) => {
            info!("🗑️  SoftDevice flash page erased @ 0x{:08X}", page_start);
            true
        }
        Err(_) => {
            warn!("SoftDevice flash erase failed during bond clear");
            false
        }
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

    // --- Spawn 50 Hz WS2812B LED animation engine on P0.11 (PWM0) ---
    spawner.spawn(unwrap!(led_task(p.PWM0, p.P0_11)));

    // --- I2C + IMU ---
    let i2c = init_i2c(p.TWISPI0, p.P0_26, p.P0_27);
    let mut imu = init_imu(i2c);

    // --- NFCT peripheral ---
    let nfct_config = NfcConfig {
        nfcid1: NfcId::SingleSize([0x04, 0x01, 0x02, 0x03]),
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

    let existing_device = read_bonded_device();
    info!(
        "   Flash: {}",
        if existing_device.is_some() {
            "bonded device found"
        } else {
            "no bonded device"
        }
    );

    let nfc_detected = detect_nfc_field(&mut nfct, NFC_PAIRING_TIMEOUT_SECS * 1000).await;

    let nfc_uid: Option<[u8; 10]> = if nfc_detected {
        let raw_nfcid = [0x04u8, 0x01, 0x02, 0x03];
        let uid = parse_nfct_uid(&raw_nfcid);
        store_activated_uid(&raw_nfcid);
        uid
    } else {
        None
    };

    // Decide pairing outcome using SoftDevice Flash API for any writes
    let bonded_mac: Option<[u8; 6]> = if nfc_detected {
        match existing_device {
            Some(ref dev) => {
                let uid_matches = nfc_uid.is_some_and(|uid| {
                    flash_read_bonded_device_full().is_some_and(|(stored, stored_uid)| {
                        stored.mac == dev.mac
                            && stored_uid == uid
                            && stored.is_active()
                            && stored.is_paired()
                    })
                });
                if uid_matches {
                    info!("✅ NFC detected – bonded MAC + UID confirmed");
                    set_nfc_pairing_status(NfcPairingStatus::Success);
                    Some(dev.mac)
                } else {
                    warn!("❌ NFC detected – bonded UID authentication failed");
                    set_nfc_pairing_status(NfcPairingStatus::Failed);
                    None
                }
            }
            None => {
                info!("🆕 NFC detected – first-time pairing: registering new device");
                let mut dev = BondedDevice::new(PLACEHOLDER_MAC);
                dev.set_paired(true);
                dev.set_verified(nfc_uid.is_some());

                if let Some(uid) = nfc_uid {
                    if sd_write_bonded_device_full(&mut sd_flash, &dev, &uid).await {
                        info!("   Bonded device + UID stored via SoftDevice flash");
                    }
                } else if sd_write_bonded_device(&mut sd_flash, &dev).await {
                    info!("   Bonded device stored via SoftDevice flash (no UID)");
                }
                set_nfc_pairing_status(NfcPairingStatus::Success);
                Some(PLACEHOLDER_MAC)
            }
        }
    } else {
        info!("⏱️  NFC timeout – using existing flash record (if any)");
        set_nfc_pairing_status(NfcPairingStatus::Timeout);
        existing_device.map(|d| d.mac)
    };

    let _ = server
        .swd
        .pairing_status_set(&get_nfc_pairing_status().to_u8());

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
    info!("📊 Sensor + Madgwick AHRS loop running at 20 Hz");

    loop {
        // Handle ClearBondedDevices TLV command
        if CLEAR_BONDS_CHANNEL.try_receive().is_ok() {
            info!("🗑️  Clearing bonded devices from flash");
            if sd_clear_bonded_devices(&mut sd_flash).await {
                WHITELIST_UPDATE_SIGNAL.signal(None);
            }
        }

        // Drain any pending MAC update from the mobile app (pairing_mac write)
        if let Ok(new_mac) = MAC_UPDATE_CHANNEL.try_receive() {
            info!(
                "🔄 Updating bonded MAC from placeholder to real: {:?}",
                new_mac
            );
            let persisted = match flash_read_bonded_device_full() {
                Some((mut dev, uid)) => {
                    dev.mac = new_mac;
                    sd_write_bonded_device_full(&mut sd_flash, &dev, &uid).await
                }
                None => match read_bonded_device() {
                    Some(mut dev) => {
                        dev.mac = new_mac;
                        sd_write_bonded_device(&mut sd_flash, &dev).await
                    }
                    None => {
                        let mut dev = BondedDevice::new(new_mac);
                        dev.set_paired(true);
                        sd_write_bonded_device(&mut sd_flash, &dev).await
                    }
                },
            };

            if persisted {
                info!("   ✅ MAC persisted via SoftDevice flash; hot-reloading BLE whitelist");
                WHITELIST_UPDATE_SIGNAL.signal(Some(new_mac));
            }
        }

        // Run one iteration of 20 Hz sensor sampling + AHRS fusion + LED gesture dispatch
        if let Some(data) = read_sensor_data(&mut imu) {
            let lib_data = LibSensorData::new(
                data.accel_x,
                data.accel_y,
                data.accel_z,
                data.gyro_x,
                data.gyro_y,
                data.gyro_z,
            );

            if !validate_sensor_data(&lib_data) {
                warn!("⚠️  sensor out of range");
            } else {
                let thrust_thresh = ACTIVE_THRUST_THRESHOLD.load(Ordering::Relaxed);
                let impact_thresh = ACTIVE_IMPACT_THRESHOLD.load(Ordering::Relaxed);

                if let Some(gesture_anim) =
                    detect_sword_gesture_with_thresholds(&lib_data, thrust_thresh, impact_thresh)
                {
                    trigger_led_animation(gesture_anim);
                    let peak = lib_data
                        .accel_x
                        .saturating_abs()
                        .max(lib_data.accel_y.saturating_abs())
                        .max(lib_data.accel_z.saturating_abs());
                    let evt = ApiEvent::GestureEvent {
                        animation_id: gesture_anim.id(),
                        peak_accel: peak,
                    };
                    let _ = EVENT_CHANNEL.try_send(evt.encode());
                }

                // Step Madgwick AHRS filter and stream 8-byte Q14 quaternion
                let quat = ahrs.update_from_sensor_data(&lib_data);
                let _ = ORIENTATION_CHANNEL.try_send(quat.to_q14_bytes());

                let _ = SENSOR_CHANNEL.try_send(data);
                info!(
                    "A:({},{},{}) G:({},{},{})",
                    data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z
                );
            }
        }

        Timer::after_millis(SENSOR_SAMPLING_INTERVAL_MS).await;
    }
}
