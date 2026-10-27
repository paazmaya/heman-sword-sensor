//! Embedded ble adapter. Hardware-independent contracts live in the library.

// nrf-softdevice generates the event enum with a Write suffix on every variant.
#![allow(clippy::enum_variant_names)]

use crate::{
    ACTIVE_IMPACT_THRESHOLD, ACTIVE_LED_ANIMATION, ACTIVE_LED_BRIGHTNESS, ACTIVE_LED_MODE,
    ACTIVE_THRUST_THRESHOLD, CONNECTION_EPOCH, EVENT_CHANNEL, GESTURE_CHANNEL, LED_CMD_CHANNEL,
    ORIENTATION_CHANNEL, SENSOR_CHANNEL,
};
use core::sync::atomic::Ordering;
use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};
use embassy_time::Timer;
use nrf_softdevice::{
    ble::{
        advertisement_builder::{
            Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList,
        },
        gatt_server, peripheral, Address, AddressType, Connection,
    },
    Softdevice,
};
use static_cell::StaticCell;
use xiao_nrf52840_sword::{
    api::{ApiEvent, TLV_PACKET_SIZE},
    control::{dispatch, CommandTarget, LedAction},
    get_nfc_pairing_status,
    pairing::is_valid_ble_mac,
};

// Custom 128-bit service UUID: he-man-sword-sensor
//   Service UUID:          6e400001-b5a3-f393-e0a9-e50e24dcca9e
//   Sensor data char:      6e400003-b5a3-f393-e0a9-e50e24dcca9e  (read/notify, 12B)
//   Pairing status char:   6e400004-b5a3-f393-e0a9-e50e24dcca9e  (read/notify, 1B)
//   Pairing MAC char:      6e400005-b5a3-f393-e0a9-e50e24dcca9e  (write, 6B)
//   Command/Control char:  6e400006-b5a3-f393-e0a9-e50e24dcca9e  (write/notify, 16B TLV)
//   Orientation data char: 6e400007-b5a3-f393-e0a9-e50e24dcca9e  (read/notify, 8B Q14 quat)

/// GATT service for He-Man Sword Sensor.
#[nrf_softdevice::gatt_service(uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e")]
pub(crate) struct SwordSensorService {
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
pub(crate) struct Server {
    pub(crate) swd: SwordSensorService,
}

// ============================================================================
// STATIC CELL FOR GATT SERVER
// ============================================================================

pub(crate) static SERVER: StaticCell<Server> = StaticCell::new();

/// Update the readable state after boot and persistence operations.
pub(crate) fn update_pairing_characteristic(server: &Server) {
    if let Err(error) = server
        .swd
        .pairing_status_set(&get_nfc_pairing_status().to_u8())
    {
        warn!("Pairing characteristic update failed: {:?}", error);
    }
}

// ============================================================================
// EMBASSY TASKS
// ============================================================================

/// Drive the nrf-softdevice event loop.  Must run on its own task.
#[embassy_executor::task]
pub(crate) async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

/// Ordered storage requests prevent a MAC update from overtaking a bond clear.
pub(crate) enum StorageCommand {
    ReplaceMac { epoch: u32, mac: [u8; 6] },
    Clear { epoch: u32 },
}

pub(crate) static STORAGE_CHANNEL: Channel<CriticalSectionRawMutex, StorageCommand, 2> =
    Channel::new();

// Signal from main → ble_task to hot-reload the BLE GAP whitelist without rebooting.
pub(crate) static WHITELIST_UPDATE_SIGNAL: Signal<CriticalSectionRawMutex, Option<[u8; 6]>> =
    Signal::new();

/// Construct the current `ApiEvent::DeviceStatus` snapshot from shared atomics.
pub(crate) fn build_device_status_event() -> ApiEvent {
    ApiEvent::DeviceStatus {
        pairing_status: get_nfc_pairing_status().to_u8(),
        led_mode: ACTIVE_LED_MODE.load(Ordering::Relaxed),
        active_animation: ACTIVE_LED_ANIMATION.load(Ordering::Relaxed),
        brightness: ACTIVE_LED_BRIGHTNESS.load(Ordering::Relaxed),
        thrust_threshold: ACTIVE_THRUST_THRESHOLD.load(Ordering::Relaxed),
        impact_threshold: ACTIVE_IMPACT_THRESHOLD.load(Ordering::Relaxed),
    }
}

pub(crate) fn emit_event(epoch: u32, event: ApiEvent) {
    if epoch != CONNECTION_EPOCH.load(Ordering::Relaxed) {
        warn!("Discarding response for a disconnected client");
        return;
    }
    if EVENT_CHANNEL.try_send((epoch, event.encode())).is_err() {
        warn!("Control response queue full; client must retry");
    }
}

struct BleCommandTarget {
    epoch: u32,
}

impl CommandTarget for BleCommandTarget {
    fn response_capacity(&self) -> usize {
        EVENT_CHANNEL.free_capacity()
    }

    fn enqueue_led(&mut self, action: LedAction) -> bool {
        let accepted = LED_CMD_CHANNEL.try_send(action).is_ok();
        if !accepted {
            warn!("LED command queue full; command rejected");
        }
        accepted
    }

    fn enqueue_clear(&mut self) -> bool {
        let accepted = STORAGE_CHANNEL
            .try_send(StorageCommand::Clear { epoch: self.epoch })
            .is_ok();
        if !accepted {
            warn!("Storage queue full; command rejected");
        }
        accepted
    }

    fn set_thresholds(&mut self, thrust: i16, impact: i16) {
        ACTIVE_THRUST_THRESHOLD.store(thrust, Ordering::Relaxed);
        ACTIVE_IMPACT_THRESHOLD.store(impact, Ordering::Relaxed);
    }

    fn status(&self) -> ApiEvent {
        build_device_status_event()
    }

    fn respond(&mut self, event: ApiEvent) {
        emit_event(self.epoch, event);
    }
}

/// Dispatch using the same execution rules exercised by host tests.
pub(crate) fn handle_tlv_command(raw_packet: &[u8; TLV_PACKET_SIZE]) {
    let mut target = BleCommandTarget {
        epoch: CONNECTION_EPOCH.load(Ordering::Relaxed),
    };
    if let Err(error) = dispatch(raw_packet, &mut target) {
        warn!(
            "Control response queue saturated; command not executed: {:?}",
            defmt::Debug2Format(&error)
        );
    }
}

/// BLE peripheral task: advertise → accept connection → stream sensor/orientation/events.
///
/// Supports hot-reloading `bonded_mac` via [`WHITELIST_UPDATE_SIGNAL`] without rebooting.
#[embassy_executor::task]
pub(crate) async fn ble_task(
    sd: &'static Softdevice,
    server: &'static Server,
    mut bonded_mac: Option<[u8; 6]>,
) {
    // Build advertising payload
    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
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
        update_pairing_characteristic(server);

        let conn: Connection = match whitelist_mac {
            Some(mac) => {
                info!("   Whitelist advertising for MAC: {:?}", mac);
                let addr = Address::new(AddressType::RandomStatic, mac);
                if let Err(e) = nrf_softdevice::ble::set_whitelist(sd, &[addr]) {
                    warn!("Failed to set BLE whitelist: {:?}", e);
                    Timer::after_millis(500).await;
                    continue;
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
                if let Err(e) = nrf_softdevice::ble::set_whitelist(sd, &[]) {
                    warn!("Failed to clear BLE whitelist: {:?}", e);
                    Timer::after_millis(500).await;
                    continue;
                }
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
        let epoch = CONNECTION_EPOCH
            .fetch_add(1, Ordering::Relaxed)
            .wrapping_add(1);
        EVENT_CHANNEL.clear();
        SENSOR_CHANNEL.reset();
        ORIENTATION_CHANNEL.reset();
        GESTURE_CHANNEL.reset();
        update_pairing_characteristic(server);

        // Run GATT server and sensor/orientation/event streaming concurrently until disconnect
        let subscriptions = core::cell::Cell::new(0u8);
        let stream_fut = stream_sensor_data_ble(server, &conn, &subscriptions);
        let gatt_fut = gatt_server::run(&conn, server, |e| match e {
            ServerEvent::Swd(e) => match e {
                SwordSensorServiceEvent::SensorDataCccdWrite { notifications } => {
                    update_subscription(&subscriptions, 1, notifications);
                    info!("sensor_data notifications: {}", notifications);
                }
                SwordSensorServiceEvent::PairingStatusCccdWrite { notifications } => {
                    update_subscription(&subscriptions, 2, notifications);
                    info!("pairing_status notifications: {}", notifications);
                }
                SwordSensorServiceEvent::CommandControlCccdWrite { notifications } => {
                    update_subscription(&subscriptions, 4, notifications);
                    info!("command_control notifications: {}", notifications);
                }
                SwordSensorServiceEvent::OrientationDataCccdWrite { notifications } => {
                    update_subscription(&subscriptions, 8, notifications);
                    info!("orientation_data notifications: {}", notifications);
                }
                SwordSensorServiceEvent::PairingMacWrite(value) => {
                    info!("📲 pairing_mac written by central: {:?}", value);
                    if is_valid_ble_mac(&value) {
                        if STORAGE_CHANNEL
                            .try_send(StorageCommand::ReplaceMac { epoch, mac: value })
                            .is_err()
                        {
                            warn!("Storage queue full – MAC update rejected");
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
        CONNECTION_EPOCH.fetch_add(1, Ordering::Relaxed);
        EVENT_CHANNEL.clear();
        STORAGE_CHANNEL.clear();
    }
}

fn update_subscription(subscriptions: &core::cell::Cell<u8>, bit: u8, enabled: bool) {
    let current = subscriptions.get();
    subscriptions.set(if enabled {
        current | bit
    } else {
        current & !bit
    });
}

/// Prioritize replies over lossy latest-sample telemetry.
pub(crate) async fn stream_sensor_data_ble(
    server: &Server,
    conn: &Connection,
    subscriptions: &core::cell::Cell<u8>,
) {
    let mut previous_pairing = None;
    loop {
        match select(
            EVENT_CHANNEL.receive(),
            select(
                SENSOR_CHANNEL.wait(),
                select(ORIENTATION_CHANNEL.wait(), GESTURE_CHANNEL.wait()),
            ),
        )
        .await
        {
            Either::First((epoch, packet)) => {
                if epoch != CONNECTION_EPOCH.load(Ordering::Relaxed) {
                    warn!("Discarding stale control response");
                    continue;
                }
                if subscriptions.get() & 4 == 0 {
                    warn!("Control response not delivered: subscribe to command_control first");
                } else if let Err(e) = server.swd.command_control_notify(conn, &packet) {
                    warn!(
                        "Control response delivery failed; client must retry: {:?}",
                        e
                    );
                }
            }
            Either::Second(Either::First(data)) => {
                let bytes = data.to_bytes();
                if let Err(e) = server.swd.sensor_data_set(&bytes) {
                    warn!("Sensor characteristic update failed: {:?}", e);
                }
                if subscriptions.get() & 1 != 0 {
                    if let Err(e) = server.swd.sensor_data_notify(conn, &bytes) {
                        info!("BLE sensor_data notify skipped: {:?}", e);
                    }
                }
            }
            Either::Second(Either::Second(Either::First(quat_bytes))) => {
                if let Err(e) = server.swd.orientation_data_set(&quat_bytes) {
                    warn!("Orientation characteristic update failed: {:?}", e);
                }
                if subscriptions.get() & 8 != 0 {
                    if let Err(e) = server.swd.orientation_data_notify(conn, &quat_bytes) {
                        info!("BLE orientation_data notify skipped: {:?}", e);
                    }
                }
            }
            Either::Second(Either::Second(Either::Second(tlv_event))) => {
                if subscriptions.get() & 4 != 0 {
                    if let Err(e) = server.swd.command_control_notify(conn, &tlv_event) {
                        info!("BLE command_control notify skipped: {:?}", e);
                    }
                }
            }
        }
        let pairing = get_nfc_pairing_status().to_u8();
        if subscriptions.get() & 2 != 0 && previous_pairing != Some(pairing) {
            match server.swd.pairing_status_notify(conn, &pairing) {
                Ok(()) => previous_pairing = Some(pairing),
                Err(e) => warn!("Pairing status delivery failed: {:?}", e),
            }
        }
    }
}
