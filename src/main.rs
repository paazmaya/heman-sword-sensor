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

use defmt::{info, unwrap, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    config::Config as NrfConfig,
    nfct::{Config as NfcConfig, NfcId, NfcT, SddPat, SelResProtocol},
    nvmc::Nvmc,
    peripherals,
    twim::{self, Twim},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use linked_list_allocator::LockedHeap;
use lsm6ds3tr::interface::I2cInterface;
use lsm6ds3tr::LSM6DS3TR;
use nrf_softdevice::ble::advertisement_builder::{
    Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList, ServiceUuid16,
};
use nrf_softdevice::ble::{gatt_server, peripheral, Address, AddressType, Connection};
use nrf_softdevice::{raw, Softdevice};
use panic_probe as _;
use static_cell::{ConstStaticCell, StaticCell};

// Lib re-exports
use xiao_nrf52840_sword::{
    detect_upward_thrust,
    nfc::{parse_nfct_uid, store_activated_uid},
    pairing::{
        make_flash_record, parse_flash_record, FLASH_BONDED_DEVICE_START, FLASH_RECORD_SIZE,
    },
    BondedDevice, NFC_PAIRING_TIMEOUT_SECS, SENSOR_SAMPLING_INTERVAL_MS,
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

// ============================================================================
// GATT SERVER DEFINITION
// ============================================================================

// Custom 128-bit service UUID: he-man-sword-sensor
// Base: 0x48454d41-4e53-574f-5244-53454e534f52  ("HEMANSWORDINSENSO" → padded)
// Using a fixed UUID in the Nordic vendor range for clarity:
//   Service UUID:          6e400001-b5a3-f393-e0a9-e50e24dcca9e  (Nordic UART-style)
//   Sensor data char:      6e400003-b5a3-f393-e0a9-e50e24dcca9e  (notify)
//   Pairing status char:   6e400004-b5a3-f393-e0a9-e50e24dcca9e  (read/notify)

/// GATT service for He-Man Sword Sensor.
///
/// Exposes three characteristics:
/// - `sensor_data` (notify): 12-byte packed accel+gyro at 20 Hz
/// - `pairing_status` (read/notify): 1-byte pairing status code
/// - `pairing_mac` (write): mobile app writes its real 6-byte BLE MAC here
///   during first-time pairing to replace the placeholder stored in flash
#[nrf_softdevice::gatt_service(uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e")]
struct SwordSensorService {
    /// 12-byte sensor data (accel XYZ + gyro XYZ, i16 LE each), notify only.
    #[characteristic(uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e", read, notify)]
    sensor_data: [u8; 12],

    /// 1-byte pairing status code (0=Idle, 1=Scanning, 2=Auth, 3=Success, 4=Failed, 5=Timeout).
    #[characteristic(uuid = "6e400004-b5a3-f393-e0a9-e50e24dcca9e", read, notify)]
    pairing_status: u8,

    /// 6-byte BLE MAC written by the mobile app during first-time pairing.
    ///
    /// When a central writes a valid 6-byte value here, the firmware
    /// updates the bonded device record in NVMC flash to replace the
    /// placeholder MAC.  Subsequent connections will use whitelist
    /// advertising targeting this real MAC.
    #[characteristic(uuid = "6e400005-b5a3-f393-e0a9-e50e24dcca9e", write)]
    pairing_mac: [u8; 6],
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

/// BLE peripheral task: advertise → accept connection → stream sensor data.
///
/// When `bonded_mac` is `Some`, advertising uses a whitelist so only the
/// paired central can connect.  When `None`, open undirected advertising is
/// used (first-time pairing / fallback mode).
///
/// If the connected central writes to the `pairing_mac` characteristic, the
/// new MAC is forwarded via [`MAC_UPDATE_CHANNEL`] so the caller can persist
/// it to NVMC flash and restart advertising with whitelist mode.
#[embassy_executor::task]
async fn ble_task(sd: &'static Softdevice, server: &'static Server, bonded_mac: Option<[u8; 6]>) {
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
    if let Some(mac) = bonded_mac {
        info!("   Whitelist MAC: {:?}", mac);
    } else {
        info!("   Open advertising (no bonded device)");
    }

    loop {
        let mut config = peripheral::Config::default();

        // When we have a bonded MAC, reduce the advertising interval to
        // 20 ms (32 units × 0.625 ms) for faster reconnect.  Open
        // advertising uses the default 100 ms interval.
        if bonded_mac.is_some() {
            config.interval = 32; // 32 × 0.625 ms = 20 ms
        }

        // Advertise until a central connects.
        // With a bonded MAC we use a whitelist so only the paired device
        // can connect.  Without one we use open undirected advertising.
        let conn: Connection = match bonded_mac {
            Some(mac) => {
                // Set the GAP whitelist to the bonded MAC before advertising.
                let addr = Address::new(AddressType::RandomStatic, mac);
                if let Err(e) = nrf_softdevice::ble::set_whitelist(sd, &[addr]) {
                    warn!("Failed to set BLE whitelist: {:?}", e);
                }
                // FilterPolicy::Both restricts both scan and connection requests
                // to the whitelist, so only the paired central can connect.
                config.filter_policy = peripheral::FilterPolicy::Both;
                let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
                    adv_data: &ADV_DATA,
                    scan_data: &SCAN_DATA,
                };
                match peripheral::advertise_connectable(sd, adv, &config).await {
                    Ok(c) => c,
                    Err(e) => {
                        warn!("BLE advertise error (whitelist): {:?}", e);
                        Timer::after_millis(500).await;
                        continue;
                    }
                }
            }
            None => {
                // Clear any previously set whitelist so open advertising works.
                let _ = nrf_softdevice::ble::set_whitelist(sd, &[]);
                config.filter_policy = peripheral::FilterPolicy::Any;
                let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
                    adv_data: &ADV_DATA,
                    scan_data: &SCAN_DATA,
                };
                match peripheral::advertise_connectable(sd, adv, &config).await {
                    Ok(c) => c,
                    Err(e) => {
                        warn!("BLE advertise error (open): {:?}", e);
                        Timer::after_millis(500).await;
                        continue;
                    }
                }
            }
        };

        info!("✅ BLE central connected");

        // Update pairing_status to Success (3)
        let _ = server.swd.pairing_status_set(&3u8);

        // Run GATT server and sensor streaming concurrently until disconnect
        let sensor_fut = stream_sensor_data_ble(server, &conn);
        let gatt_fut = gatt_server::run(&conn, server, |e| match e {
            ServerEvent::Swd(e) => match e {
                SwordSensorServiceEvent::SensorDataCccdWrite { notifications } => {
                    info!("sensor_data notifications: {}", notifications);
                }
                SwordSensorServiceEvent::PairingStatusCccdWrite { notifications } => {
                    info!("pairing_status notifications: {}", notifications);
                }
                SwordSensorServiceEvent::PairingMacWrite(value) => {
                    info!("📲 pairing_mac written by central: {:?}", value);
                    if MAC_UPDATE_CHANNEL.try_send(value).is_err() {
                        warn!("MAC_UPDATE_CHANNEL full – MAC update dropped");
                    }
                }
            },
        });
        embassy_futures::select::select(sensor_fut, gatt_fut).await;

        info!("🔌 BLE central disconnected – restarting advertising");
        let _ = server.swd.pairing_status_set(&0u8); // 0 = Idle
    }
}

/// Drain SENSOR_CHANNEL and notify the connected BLE central.
async fn stream_sensor_data_ble(server: &Server, conn: &Connection) {
    loop {
        let data = SENSOR_CHANNEL.receive().await;
        let bytes = data.to_bytes();

        match server.swd.sensor_data_notify(conn, &bytes) {
            Ok(()) => {}
            Err(e) => {
                // Notification failed (e.g. no CCCD subscription yet) — set value silently
                let _ = server.swd.sensor_data_set(&bytes);
                info!("BLE notify skipped: {:?}", e);
            }
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
// LED ANIMATION (WS2812B stub)
// ============================================================================

/// Signal a thrust on the LED strip.
fn animate_led_thrust(_duration_ms: u32) {
    info!("💡 LED thrust animation (stub)");
}

// ============================================================================
// NFC FIELD DETECTION
// ============================================================================

/// Wait for an NFC field using the real embassy-nrf NFCT peripheral.
/// Returns `true` if a field was detected within `timeout_ms` milliseconds.
async fn detect_nfc_field(nfct: &mut NfcT<'static>, timeout_ms: u64) -> bool {
    info!("📡 NFC scan – timeout {} ms", timeout_ms);
    let result =
        embassy_time::with_timeout(Duration::from_millis(timeout_ms), nfct.activate()).await;
    match result {
        Ok(_) => {
            info!("✅ NFC field detected");
            true
        }
        Err(_) => {
            info!("⏱️  NFC timeout");
            false
        }
    }
}

// ============================================================================
// NVMC FLASH OPERATIONS
// ============================================================================

/// Write one bonded device record to flash using the NVMC peripheral.
///
/// The nRF52840 requires:
/// 1. Erase the 4 KB page first (flash can only be written from 1→0).
/// 2. Write aligned 4-byte words.
///
/// We write the 12-byte flash record at [`FLASH_BONDED_DEVICE_START`].
/// Any existing record in that page is overwritten (page erase clears it).
fn nvmc_write_bonded_device(nvmc: &mut Nvmc<'static>, device: &BondedDevice) -> bool {
    let record = make_flash_record(device);

    // Pad the record to a 4-byte multiple (NVMC requires word-aligned writes)
    const PADDED: usize = (FLASH_RECORD_SIZE + 3) & !3; // = 12 or 16 depending on FLASH_RECORD_SIZE
    let mut padded = [0xFFu8; PADDED]; // 0xFF = erased flash byte
    padded[..FLASH_RECORD_SIZE].copy_from_slice(&record);

    // Erase the page that contains our record
    let page_start = FLASH_BONDED_DEVICE_START & !0xFFF; // align down to 4 KB
    let page_end = page_start + 4096;
    match nvmc.erase(page_start, page_end) {
        Ok(()) => {}
        Err(_) => {
            warn!("NVMC erase failed");
            return false;
        }
    }

    match nvmc.write(FLASH_BONDED_DEVICE_START, &padded[..PADDED]) {
        Ok(()) => {
            info!(
                "💾 NVMC write OK – MAC {:?} @ 0x{:08X}",
                device.mac, FLASH_BONDED_DEVICE_START
            );
            true
        }
        Err(_) => {
            warn!("NVMC write failed");
            false
        }
    }
}

/// Read the bonded device record from flash via direct NVMC read.
fn nvmc_read_bonded_device(nvmc: &mut Nvmc<'static>) -> Option<BondedDevice> {
    let mut record = [0u8; FLASH_RECORD_SIZE];
    match nvmc.read(FLASH_BONDED_DEVICE_START, &mut record) {
        Ok(()) => parse_flash_record(&record),
        Err(_) => {
            warn!("NVMC read failed");
            None
        }
    }
}

/// Read the bonded device + 10-byte UID from flash.
fn nvmc_read_bonded_device_full(nvmc: &mut Nvmc<'static>) -> Option<(BondedDevice, [u8; 10])> {
    const FULL: usize = FLASH_RECORD_SIZE + 10;
    let mut buf = [0u8; FULL];
    match nvmc.read(FLASH_BONDED_DEVICE_START, &mut buf) {
        Ok(()) => {
            let mut record = [0u8; FLASH_RECORD_SIZE];
            record.copy_from_slice(&buf[..FLASH_RECORD_SIZE]);
            let dev = parse_flash_record(&record)?;
            let mut uid = [0u8; 10];
            uid.copy_from_slice(&buf[FLASH_RECORD_SIZE..]);
            Some((dev, uid))
        }
        Err(_) => {
            warn!("NVMC full read failed");
            None
        }
    }
}

/// Write a full bonded device record (MAC + 10-byte NFC UID) to flash.
///
/// The record layout in flash is:
///   Bytes 0–11:  Primary 12-byte flash record (magic + BondedDevice)
///   Bytes 12–22: 10-byte NFC UID (appended immediately after)
///
/// Both regions are written in a single page erase+write cycle so they
/// stay consistent even if power is lost between writes.
fn nvmc_write_bonded_device_full(
    nvmc: &mut Nvmc<'static>,
    device: &BondedDevice,
    uid: &[u8; 10],
) -> bool {
    let record = make_flash_record(device);

    // Layout: [12-byte primary record][10-byte UID] = 22 bytes.
    // Pad to 4-byte word boundary for NVMC.
    const RAW: usize = FLASH_RECORD_SIZE + 10; // 22
    const PADDED: usize = (RAW + 3) & !3; // 24
    let mut buf = [0xFFu8; PADDED];
    buf[..FLASH_RECORD_SIZE].copy_from_slice(&record);
    buf[FLASH_RECORD_SIZE..FLASH_RECORD_SIZE + 10].copy_from_slice(uid);

    let page_start = FLASH_BONDED_DEVICE_START & !0xFFF;
    let page_end = page_start + 4096;
    match nvmc.erase(page_start, page_end) {
        Ok(()) => {}
        Err(_) => {
            warn!("NVMC erase failed (full record)");
            return false;
        }
    }
    match nvmc.write(FLASH_BONDED_DEVICE_START, &buf[..PADDED]) {
        Ok(()) => {
            info!(
                "💾 NVMC write OK (full) – MAC {:?} UID {:?} @ 0x{:08X}",
                device.mac, uid, FLASH_BONDED_DEVICE_START
            );
            true
        }
        Err(_) => {
            warn!("NVMC write failed (full record)");
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

    // --- I2C + IMU ---
    let i2c = init_i2c(p.TWISPI0, p.P0_26, p.P0_27);
    let mut imu = init_imu(i2c);

    // --- NVMC flash driver ---
    // SAFETY: We hold the single NVMC peripheral token for the lifetime of main.
    let mut nvmc = Nvmc::new(p.NVMC);

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
    // STEP 1 – NFC PAIRING GATE (15-second window)
    // =========================================================================
    info!("");
    info!(
        "🔌 NFC pairing gate – {} s window",
        NFC_PAIRING_TIMEOUT_SECS
    );

    // Read whatever is already in flash
    let existing_device = nvmc_read_bonded_device(&mut nvmc);
    info!(
        "   Flash: {}",
        if existing_device.is_some() {
            "bonded device found"
        } else {
            "no bonded device"
        }
    );

    let nfc_detected = detect_nfc_field(&mut nfct, NFC_PAIRING_TIMEOUT_SECS * 1000).await;

    // Extract the NFC UID from the bytes returned by NFCT activation.
    // A real tag provides its own UID bytes after activate() returns; this
    // sample uses the configured four-byte Type 2 identifier.
    //
    // The validated UID is deposited into the shared NFC_UID static so
    // pairing_mode() (and any future pairing logic) can read it without
    // receiving it as a function argument.
    let nfc_uid: Option<[u8; 10]> = if nfc_detected {
        // These are the four bytes configured in NfcId::SingleSize.
        let raw_nfcid = [0x04u8, 0x01, 0x02, 0x03];
        let uid = parse_nfct_uid(&raw_nfcid);
        // Deposit into the shared static so lib-level helpers can see it
        store_activated_uid(&raw_nfcid);
        uid
    } else {
        None
    };

    // Decide pairing outcome
    let bonded_mac: Option<[u8; 6]> = if nfc_detected {
        match existing_device {
            Some(ref dev) => {
                let uid_matches = nfc_uid.is_some_and(|uid| {
                    nvmc_read_bonded_device_full(&mut nvmc).is_some_and(|(stored, stored_uid)| {
                        stored.mac == dev.mac
                            && stored_uid == uid
                            && stored.is_active()
                            && stored.is_paired()
                    })
                });
                if uid_matches {
                    info!("✅ NFC detected – bonded MAC + UID confirmed");
                    Some(dev.mac)
                } else {
                    warn!("❌ NFC detected – bonded UID authentication failed");
                    None
                }
            }
            None => {
                info!("🆕 NFC detected – first-time pairing: registering new device");
                // The mobile app will update the real MAC via the writable
                // pairing_mac GATT characteristic once it connects.
                // We store a placeholder MAC now so the device can advertise.
                let new_mac = [0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01];
                let mut dev = BondedDevice::new(new_mac);
                dev.set_paired(true);
                dev.set_verified(nfc_uid.is_some());

                if let Some(uid) = nfc_uid {
                    // Write primary record + NFC UID together
                    if nvmc_write_bonded_device_full(&mut nvmc, &dev, &uid) {
                        info!("   Bonded device + UID stored in flash");
                    }
                } else {
                    // No valid UID — write primary record only
                    if nvmc_write_bonded_device(&mut nvmc, &dev) {
                        info!("   Bonded device stored in flash (no UID)");
                    }
                }
                Some(new_mac)
            }
        }
    } else {
        info!("⏱️  NFC timeout – using existing flash record (if any)");
        existing_device.map(|d| d.mac)
    };

    info!(
        "   BLE mode: {}",
        if bonded_mac.is_some() {
            "whitelist"
        } else {
            "open"
        }
    );

    // =========================================================================
    // STEP 2 – SOFTDEVICE ENABLE
    //
    // The nRF52840 S140 SoftDevice must be flashed separately (once) using:
    //   probe-rs download --verify --binary-format hex \
    //     --chip nRF52840_xxAA s140_nrf52_7.x.x_softdevice.hex
    //
    // memory.x must reserve the first 0x26000 bytes for the SoftDevice.
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

    // Softdevice event loop must run on its own task
    spawner.spawn(unwrap!(softdevice_task(sd)));

    // =========================================================================
    // STEP 3 – SPAWN BLE TASK
    // =========================================================================
    spawner.spawn(unwrap!(ble_task(sd, server, bonded_mac)));

    // =========================================================================
    // STEP 4 – MAIN LOOP: sensor sampling + MAC update from GATT writes
    // =========================================================================
    info!("📊 Sensor loop running at 20 Hz");
    loop {
        // Drain any pending MAC update from the mobile app (pairing_mac write)
        if let Ok(new_mac) = MAC_UPDATE_CHANNEL.try_receive() {
            info!(
                "🔄 Updating bonded MAC from placeholder to real: {:?}",
                new_mac
            );
            match nvmc_read_bonded_device_full(&mut nvmc) {
                Some((mut dev, uid)) => {
                    dev.mac = new_mac;
                    // Preserve existing flags/timestamp, only update MAC
                    if nvmc_write_bonded_device_full(&mut nvmc, &dev, &uid) {
                        info!("   ✅ MAC persisted to flash (with preserved UID + flags)");
                    }
                }
                None => {
                    // No full record — try reading basic record only
                    match nvmc_read_bonded_device(&mut nvmc) {
                        Some(mut dev) => {
                            dev.mac = new_mac;
                            if nvmc_write_bonded_device(&mut nvmc, &dev) {
                                info!("   ✅ MAC persisted to flash (no UID present)");
                            }
                        }
                        None => {
                            warn!("   No bonded record in flash – cannot update MAC");
                        }
                    }
                }
            }
        }

        // Run one iteration of sensor sampling
        if let Some(data) = read_sensor_data(&mut imu) {
            let accel_ok = data.accel_x.abs() < 10000
                && data.accel_y.abs() < 10000
                && data.accel_z.abs() < 10000;
            let gyro_ok =
                data.gyro_x.abs() < 20000 && data.gyro_y.abs() < 20000 && data.gyro_z.abs() < 20000;

            if !accel_ok || !gyro_ok {
                warn!("⚠️  sensor out of range");
            } else {
                if detect_upward_thrust(data.accel_z) {
                    animate_led_thrust(200);
                }
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
