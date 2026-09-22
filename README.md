# He-Man Sword Sensor

> By the power of Grayskull!

A high-tech sword sensor with real-time motion tracking, NFC pairing security, and dynamic LED animations synchronized to your strikes. Track your swings with Bluetooth and watch the power flow through the blade.

## Project Status

### Completed

- Rust + Embassy embedded runtime working on nRF52840
- I2C communication with LSM6DS3TR IMU at 100 kHz on TWIM0
- 20 Hz sensor sampling with 50 ms intervals
- Synchronous I2C reads with error handling
- 12-byte sensor data structure for BLE transmission with `to_bytes` / `from_bytes` round-trip
- 15-second NFC pairing gate with Bluetooth fallback
- Multi-task architecture with `embassy_sync` channels and `embassy_futures::select`
- Structured logging through `defmt 1.x` + RTT
- `embassy-nrf` NFCT peripheral wired up with real hardware types (`NfcId`, `SddPat`, `SelResProtocol`, `Config`); `NfcT::new` + `nfct.activate()` called in `main.rs`
- NFC field detection with configurable timeout via `embassy_time::with_timeout`
- NFC UID extraction helper (`nfc::extract_uid_from_nfct`) and validation (`is_valid_uid`, `is_valid_uid_7`)
- **NVMC flash peripheral wired**: `embassy_nrf::nvmc::Nvmc` used for real flash erase + word-aligned writes via `embedded_storage::NorFlash`; direct `read_volatile` reads at boot; `nvmc_write_bonded_device` and `nvmc_read_bonded_device` in `main.rs`
- Flash record format: 12-byte records with magic byte (`0xAB`) + serialised `BondedDevice`; `make_flash_record` / `parse_flash_record` helpers in `pairing` module
- `BondedDevice` struct with `to_bytes` / `from_bytes`, MAC, timestamp, and flag helpers (`is_active`, `is_paired`, `is_verified`, clear-capable setters)
- `BondedDeviceFull` struct storing MAC + 10-byte NFC UID for secondary authentication
- `NfcPairingStatus` enum covering all pairing lifecycle states
- `pairing` module: `authenticate_bonded_device`, `authenticate_with_uid`, `read_bonded_device_mac`, `read_bonded_device`, `read_bonded_device_full`, `write_bonded_device_to_flash`, `write_bonded_device_to_flash_full`, `register_bonded_device`, `unregister_bonded_device`, `get_bonded_devices`, `is_bonded_device`, `get_nfc_pairing_status`
- `nfc` module: `NfcFieldState`, `get_field_state`, `detect_field_with_timeout`, `detect_field`, `is_valid_uid`, `is_valid_uid_7`, `extract_uid_from_nfct`, `write_nfc_page`
- `pairing::pairing_mode()` async gate: polls for NFC field, reads bonded MAC, authenticates, falls back to BLE on timeout
- `BleConfig` struct with bonded MAC whitelist, advertising interval (units + ms helper), connection timeout, and `bonded_only()` constructor
- **nrf-softdevice S140 BLE stack integrated**: `Softdevice::enable` with full `Config` (clock, GAP, GATT, GATTS attr table, device name)
- **GATT server defined**: `SwordSensorService` with two characteristics — `sensor_data` (12-byte notify) and `pairing_status` (1-byte read/notify) — using custom 128-bit UUIDs
- **BLE advertising task** (`ble_task`): advertises as "He-Man Sword", accepts connections, supports whitelist mode when a bonded device is present
- **BLE sensor streaming task** (`stream_sensor_data_ble`): drains `SENSOR_CHANNEL` and sends notify updates to the connected central at 20 Hz
- **Softdevice task** (`softdevice_task`): drives the S140 event loop on a dedicated embassy task
- First-time pairing flow: NFC tap on a device with no stored record registers a new `BondedDevice` in NVMC flash
- `nfc` Cargo feature: compiles `nfc` and `pairing` modules on the host target with no embedded hardware required
- `memory.x` updated for S140: FLASH at `0x00026000` (after 152 KB softdevice), RAM at `0x20002000`
- Desktop tests: **91 tests** — 34 base + 57 NFC-gated (run with `cargo test --features nfc`)
  - New tests: `SensorData` serialisation round-trip, NFC UID validation, flash record magic/round-trip/inactive-slot, `BleConfig` default/whitelist/interval, `BondedDeviceFull` round-trip and UID offset, `authenticate_with_uid`, `MAX_BONDED_DEVICES` page-fit, `BondedDevice` flag clear, `FLASH_RECORD_SIZE` constant, motion classification priority

### In Progress

- Full NFC Type 2 tag UID read from the activated tag: `extract_uid_from_nfct` uses the programmed `NfcId` bytes; a real ISO 14443-4 NDEF read path would retrieve the UID directly from the reader after `activate()` completes

### TODO

- Replace placeholder MAC in first-time pairing with a real mobile-app-provided MAC (written via NDEF or a writable GATT characteristic)
- LED animation engine (WS2812B via PWM on P0.11)
- Mobile app for pairing, visualization, and real-time LED control
- Sensor fusion algorithms such as Madgwick AHRS
- Flash the nRF-Softdevice S140 hex once before first use (see flashing instructions below)

## 1. Hardware: XIAO nRF52840 Sense + LSM6DS3TR IMU

### Board: Seeed XIAO nRF52840 Sense

https://wiki.seeedstudio.com/XIAO_BLE/

| Feature          | Details                                                           |
| ---------------- | ----------------------------------------------------------------- |
| **MCU**          | nRF52840, ARM Cortex-M4, Bluetooth 5.0                            |
| **IMU**          | LSM6DS3TR-C, 6-axis accelerometer + gyroscope                     |
| **Connectivity** | Bluetooth 5.0 and built-in NFC Type 2 Tag / NFCT                  |
| **Power**        | 3.3V, BQ25101 charging, approximately 300 mAh LiPo or larger      |
| **Size**         | 20 x 17.5 mm, suitable for a sword handle                         |
| **I2C Bus**      | IMU connected through TWIM0 on P0.26 SDA and P0.27 SCL at 100 kHz |

### NFC Pairing Hardware

The nRF52840 Sense board includes a built-in NFC Type 2 Tag controller.

| Feature        | Details               |
| -------------- | --------------------- |
| **Peripheral** | NFCT, NFC Type 2 Tag  |
| **Standard**   | ISO/IEC 14443 Type A  |
| **Max Range**  | Approximately 10 cm   |
| **Detection**  | Passive tag detection |
| **UID Length** | 10 bytes              |

### LED Strip: WS2812B / NeoPixel

The sword features an LED strip that animates in real time based on motion.

| Component       | Details                                             |
| --------------- | --------------------------------------------------- |
| **LED Type**    | WS2812B addressable RGB, 5V                         |
| **Control Pin** | Configurable GPIO, for example P0.11                |
| **Behavior**    | Flash animation moves guard-to-tip on upward thrust |
| **Power**       | External 5V source recommended for the strip        |
| **Data Rate**   | 800 kHz WS2812B protocol                            |

Available GPIO pins after I2C/IMU:

- P0.02, P0.03, P0.04, P0.05, P0.06, P0.07
- P0.08, P0.09, P0.10, P0.11

### Sensor Specifications

The LSM6DS3TR provides:

- 3-axis accelerometer with configurable range: ±2, ±4, ±8, ±16 g
- 3-axis gyroscope with configurable range: ±125, ±250, ±500, ±1000, ±2000 dps
- Raw 16-bit integers for each axis
- I2C address: 0x6A

### Sensor Placement

- Best location: 10-15 cm from the guard
- Close to the wrist pivot for accurate rotational data
- Far enough from the tip to avoid excessive linear acceleration noise
- Avoid the sword tip or center of mass because they introduce centrifugal force noise

## 2. Firmware: Rust + Embassy Framework

### Tech Stack

| Component                 | Version    | Purpose                                            |
| ------------------------- | ---------- | -------------------------------------------------- |
| **Embassy**               | 0.10.0     | Async runtime with executor, timers, and channels  |
| **embassy-nrf**           | 0.11.0     | nRF HAL with NFCT, NVMC, TWIM peripheral support   |
| **nrf-softdevice**        | git/master | S140 BLE stack bindings (peripheral + GATT server) |
| **nrf-softdevice-s140**   | git/master | S140 SoftDevice FFI bindings                       |
| **lsm6ds3tr**             | 0.2.2      | LSM6DS3TR IMU driver                               |
| **defmt + defmt-rtt**     | 1.x        | Structured logging over RTT                        |
| **embassy-sync**          | 0.8.0      | Inter-task channel communication                   |
| **embassy-futures**       | 0.1        | `select` for concurrent async tasks                |
| **embedded-storage**      | 0.3.1      | `NorFlash` trait for NVMC flash operations         |
| **linked_list_allocator** | 0.10.6     | 4 KB heap allocator                                |
| **panic-probe**           | 1.0        | Panic handler                                      |

### Code Architecture

| Component                   | Purpose                                                                          | Build Target                 |
| --------------------------- | -------------------------------------------------------------------------------- | ---------------------------- |
| `src/lib.rs`                | Pure logic: motion detection, sensor validation, NFC/pairing/BLE data structures | Host `std`, desktop testable |
| `src/main.rs`               | Embedded code: I2C, IMU, NFCT, NVMC flash, nrf-softdevice BLE, async tasks       | nRF52840 `no_std`            |
| `tests/integration_test.rs` | Desktop tests for motion, validation, NFC, pairing, BLE config                   | Host `std`, `cargo test`     |

Key design decisions:

- Logic in `src/lib.rs` is testable without hardware using standard library traits (`Debug`, `PartialEq`, `Eq`)
- Embedded-specific code stays in `src/main.rs` with embedded-specific traits (`defmt::Format`)
- Optional dependencies and feature gates allow desktop testing without the nRF52840 target
- NVMC flash reads use direct `read_volatile` (flash is memory-mapped on nRF52840); writes use `embedded_storage::NorFlash` via `embassy_nrf::nvmc::Nvmc`
- nrf-softdevice S140 BLE stack runs as a dedicated embassy task (`softdevice_task`) and must be flashed once before first use

### Boot Sequence

```text
1. Initialize 4 KB heap
2. Initialize embassy-nrf peripherals
   - gpiote and time interrupt priorities set to P2 (required by S140)
3. Configure I2C on TWIM0
   - SDA: P0.26
   - SCL: P0.27
   - Frequency: 100 kHz
4. Initialize LSM6DS3TR IMU at address 0x6A
5. Initialize NVMC flash driver
6. Initialize NFCT peripheral (NFC Type 2 Tag)
7. NFC pairing gate (15-second window)
   - Read existing bonded device from NVMC flash
   - Wait for NFC field via NfcT::activate()
   - If detected + existing record: confirm bonded MAC
   - If detected + no record: first-time pairing, write new BondedDevice to flash
   - If timeout: use existing flash record (if any) or open advertising
8. Enable nrf-softdevice S140
   - Configure GAP device name, connection parameters, GATTS attr table
9. Build GATT server (SwordSensorService)
   - sensor_data characteristic (12-byte, notify)
   - pairing_status characteristic (1-byte, read + notify)
10. Spawn softdevice_task (S140 event loop)
11. Spawn ble_task (advertise → connect → stream)
12. Main sensor loop at 20 Hz (feeds SENSOR_CHANNEL → ble_task)
```

### NFC Pairing

The firmware uses NFC as the primary pairing gate. On boot, it waits up to 15 seconds for an NFC field. If a field is detected, it reads the bonded device from NVMC flash and authenticates the device using both the stored MAC and the NFC UID. If NFC times out, it falls back to BLE advertising using whatever bonded MAC is already in flash (if any).

The implementation uses embassy-nrf's NFCT peripheral (`NfcT::new` + `nfct.activate()`) and the nRF52840 NVMC peripheral (`embassy_nrf::nvmc::Nvmc`) for persistent bonded-device storage. BLE is driven by the nrf-softdevice S140 stack via Rust async bindings.

#### Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                    NFC PAIRING SYSTEM                        │
├─────────────────────────────────────────────────────────────┤
│  NFCT Peripheral ──▶ NFC Field Detection ──▶ Bonded Device  │
│       │                    │                  Storage        │
│       ▼                    ▼                    ▼            │
│  Field Detected ──▶ Read MAC from Flash ──▶ Authenticate     │
│       │                    │                    ▼            │
│       ▼                    ▼                  Pairing         │
│  Pairing Success ──▶ BLE Connection ──▶ Sensor Streaming      │
└─────────────────────────────────────────────────────────────┘
```

#### Pairing Flow

1. **Boot and initialization**
   - Initialize NFCT peripheral
   - Initialize I2C for IMU
   - Start NFC pairing mode

2. **NFC field detection**
   - Listen for NFC Type 2 Tag field presence
   - Poll every 100 ms
   - Wait up to 15 seconds

3. **Bonded device authentication**
   - Read bonded MAC from flash
   - Validate stored timestamp and flags
   - Allow BLE connection only for the paired device

4. **Pairing success**
   - Establish secure BLE connection
   - Start sensor data streaming
   - Enable real-time motion tracking

5. **Timeout fallback**
   - If NFC does not complete within 15 seconds, switch to Bluetooth fallback mode

#### Flash Storage Layout

Bonded device data is stored starting at `0x0002_0000` (128 KB offset, 4 KB page-aligned, well above application code and softdevice).

```text
┌──────────────────────────────────────────────────────────────┐
│ nRF52840 flash address map (with S140 softdevice)            │
├──────────────────────────────────────────────────────────────┤
│ 0x00000000 - 0x00025FFF │ S140 SoftDevice (152 KB)           │
│ 0x00026000 - 0x0001FFFF │ Application code (starts here)     │
│ 0x00020000 - 0x00020FFF │ Bonded device storage (4 KB page)  │
│                         │   Record format (12 bytes each):   │
│                         │   - 1 byte  magic (0xAB)           │
│                         │   - 6 bytes MAC address            │
│                         │   - 4 bytes timestamp (LE u32)     │
│                         │   - 1 byte  flags                  │
│ 0x000FFFFF              │ End of flash (1 MB total)           │
└──────────────────────────────────────────────────────────────┘
```

> **Note**: When the S140 SoftDevice is present, flash writes must go through the SoftDevice flash API (`sd_flash_write` / `sd_flash_page_erase`) to avoid conflicts with the radio scheduler. The current implementation uses `embassy_nrf::nvmc::Nvmc` directly, which is correct for bare-metal use (no softdevice loaded) or for development. Production builds with the softdevice should migrate flash writes to the softdevice flash API.

#### Bonded Device Structure

```rust
#[repr(C)]
struct BondedDevice {
    mac: [u8; 6],      // MAC address
    timestamp: u32,    // Last pairing timestamp
    flags: u8,         // Active, paired, verified
}
```

Flags:

- Bit 0: active
- Bit 1: paired
- Bit 2: verified

#### Pairing Status

```rust
enum NfcPairingStatus {
    Idle,
    Scanning,
    Authenticating,
    Success,
    Failed,
    Timeout,
}
```

#### API Reference

Top-level crate functions (available with `nfc` or `embedded` feature):

| Function                       | Description                               | Returns            |
| ------------------------------ | ----------------------------------------- | ------------------ |
| `authenticate_bonded_device()` | Verify bonded MAC (active + paired check) | `bool`             |
| `get_nfc_pairing_status()`     | Get current pairing status                | `NfcPairingStatus` |

`nfc` module (sync helpers available with `nfc` or `embedded`; async and embedded-only functions require `embedded`):

| Function                           | Description                                       | Returns            |
| ---------------------------------- | ------------------------------------------------- | ------------------ |
| `nfc::get_field_state()`           | Current field state                               | `NfcFieldState`    |
| `nfc::is_valid_uid()`              | Validate 10-byte NFC UID (cascade tag byte check) | `bool`             |
| `nfc::is_valid_uid_7()`            | Validate 7-byte NFC UID                           | `bool`             |
| `nfc::extract_uid_from_nfct()`     | Extract UID from raw NFCT bytes — `embedded` only | `Option<[u8; 10]>` |
| `nfc::detect_field()`              | Detect NFC field (500 ms default) — async         | `bool`             |
| `nfc::detect_field_with_timeout()` | Detect NFC field with custom timeout — async      | `bool`             |
| `nfc::write_nfc_page()`            | Write 4-byte Type 2 tag page — async stub         | `bool`             |

`pairing` module (sync helpers available with `nfc` or `embedded`; async requires `embedded`):

| Function                                       | Description                                        | Returns                    |
| ---------------------------------------------- | -------------------------------------------------- | -------------------------- |
| `pairing::read_bonded_device_mac()`            | Read MAC from flash (NVMC on target, stub on host) | `Option<[u8; 6]>`          |
| `pairing::read_bonded_device()`                | Read `BondedDevice` from flash                     | `Option<BondedDevice>`     |
| `pairing::read_bonded_device_full()`           | Read `BondedDeviceFull` (MAC + UID) from flash     | `Option<BondedDeviceFull>` |
| `pairing::authenticate_bonded_device()`        | Verify MAC against stored record                   | `bool`                     |
| `pairing::authenticate_with_uid()`             | Verify MAC + NFC UID against stored record         | `bool`                     |
| `pairing::get_nfc_pairing_status()`            | Current pairing status                             | `NfcPairingStatus`         |
| `pairing::write_bonded_device_to_flash()`      | Write MAC to flash (delegates to NVMC in main.rs)  | `bool`                     |
| `pairing::write_bonded_device_to_flash_full()` | Write full struct to flash                         | `bool`                     |
| `pairing::make_flash_record()`                 | Serialise device to 12-byte flash record           | `[u8; 12]`                 |
| `pairing::parse_flash_record()`                | Deserialise and validate a flash record            | `Option<BondedDevice>`     |
| `pairing::register_bonded_device()`            | Register new device                                | `bool`                     |
| `pairing::unregister_bonded_device()`          | Remove device                                      | `bool`                     |
| `pairing::get_bonded_devices()`                | Get all bonded devices                             | `[BondedDevice; 2]`        |
| `pairing::is_bonded_device()`                  | Check if MAC is bonded (active records only)       | `bool`                     |
| `pairing::pairing_mode()`                      | Full async pairing gate — async, `embedded` only   | `bool`                     |

`pairing::BleConfig`:

| Method / Field             | Description                                        |
| -------------------------- | -------------------------------------------------- |
| `BleConfig::default()`     | Open advertising, 100 ms interval, no whitelist    |
| `BleConfig::bonded_only()` | Whitelist config targeting a specific bonded MAC   |
| `adv_interval_ms()`        | Advertising interval in milliseconds               |
| `use_whitelist`            | When `true`, restricts advertising to `bonded_mac` |

GATT server (`SwordSensorService`, UUID `6e400001-b5a3-f393-e0a9-e50e24dcca9e`):

| Characteristic   | UUID suffix | Properties    | Format                                                              |
| ---------------- | ----------- | ------------- | ------------------------------------------------------------------- |
| `sensor_data`    | `…0003`     | Read + Notify | 12 bytes, i16 LE × 6 (accel XYZ, gyro XYZ)                          |
| `pairing_status` | `…0004`     | Read + Notify | 1 byte (0=Idle, 1=Scanning, 2=Auth, 3=Success, 4=Failed, 5=Timeout) |

#### Security Considerations

- Only registered MAC addresses can pair
- Bonded MAC addresses persist across reboots via NVMC flash
- 12-byte flash records include a magic byte and active/paired flags to detect stale or erased slots
- `authenticate_with_uid` adds a second factor: both the BLE MAC and the NFC UID must match
- BLE advertising is restricted to the paired device when `BleConfig::use_whitelist` is true
- Flash writes should go through the SoftDevice flash API in production (see flash storage note above)

#### Testing NFC Pairing

```bash
# Build with embedded support (requires S140 softdevice pre-flashed on device)
cargo build --features embedded --target thumbv7em-none-eabihf

# Build release image for flashing
cargo build --release --features embedded --target thumbv7em-none-eabihf

# Run desktop tests (34 tests, no feature flag needed)
cargo test --test integration_test

# Run desktop tests including all NFC/pairing/BLE tests (91 tests)
cargo test --test integration_test --features nfc

# Run on hardware (requires S140 softdevice pre-flashed; see Flashing below)
cargo run --release --features embedded --target thumbv7em-none-eabihf
```

NFC-specific tests cover:

- `NfcPairingStatus` and `NfcFieldState` all variants
- `BondedDevice` flag logic (active, paired, verified), clear-capable setters, and timestamp helpers
- `BondedDevice` `to_bytes` / `from_bytes` round-trip and field layout
- `BondedDeviceFull` (MAC + UID) serialisation round-trip and UID field offset
- `authenticate_bonded_device` with matching and wrong MACs
- `authenticate_with_uid` with correct and wrong MAC/UID combinations
- NFC UID validation (`is_valid_uid`, `is_valid_uid_7`) — cascade tag byte, all-zero rejection
- Flash record serialisation: `make_flash_record` / `parse_flash_record` round-trip, magic corruption, inactive slot
- Flash storage constants (`FLASH_BONDED_DEVICE_START`, `FLASH_BONDED_DEVICE_SIZE`, `FLASH_RECORD_SIZE`, `FLASH_RECORD_MAGIC`, `MAX_BONDED_DEVICES` page-fit)
- `BleConfig` default values, interval calculation, `bonded_only()` whitelist constructor
- All pairing CRUD helpers (`read_bonded_device_mac`, `write_bonded_device_to_flash`, `register_bonded_device`, etc.)
- `get_bonded_devices` and `is_bonded_device` lookups

#### Troubleshooting

| Issue                               | Check                                                         |
| ----------------------------------- | ------------------------------------------------------------- |
| NFC pairing times out               | Verify field detection, NFCT setup, and polling interval      |
| Bonded device not found             | Verify MAC is stored in flash and the flash region is correct |
| Authentication fails                | Verify MAC format, timestamp validity, and device flags       |
| BLE is still visible to all devices | Confirm bonding and whitelist logic are implemented           |

#### Current Implementation Status

**Hardware types in use:**

- `NfcId::SingleSize([0x01, 0x02, 0x03, 0x04])` — NFC identifier configuration
- `SddPat::Sdd00000` — single device detection pattern
- `SelResProtocol::Type2` — Type 2 Tag protocol selection
- `embassy_nrf::nvmc::Nvmc` — NVMC flash driver for bonded device persistence

**Hardware activation in `main.rs`:**

```rust
let nfct_config = NfcConfig {
    nfcid1: NfcId::SingleSize([0x01, 0x02, 0x03, 0x04]),
    sdd_pat: SddPat::Sdd00000,
    plat_conf: 0x00,
    protocol: SelResProtocol::Type2,
};
let mut nfct = NfcT::new(p.NFCT, Irqs, &nfct_config);
let detected = embassy_time::with_timeout(
    Duration::from_millis(timeout_ms),
    nfct.activate(),
).await;
```

**NVMC flash write in `main.rs`:**

```rust
// Erase the 4 KB page, then write a padded 12-byte record
nvmc.erase(page_start, page_end)?;
nvmc.write(FLASH_BONDED_DEVICE_START, &padded_record)?;
```

**GATT server in `main.rs` (proc-macro generated):**

```rust
#[nrf_softdevice::gatt_service(uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e")]
struct SwordSensorService {
    #[characteristic(uuid = "6e400003-…", read, notify)]
    sensor_data: [u8; 12],     // accel XYZ + gyro XYZ, i16 LE

    #[characteristic(uuid = "6e400004-…", read, notify)]
    pairing_status: u8,        // NfcPairingStatus numeric value
}
```

**What remains:**

- NFC UID is provisioned from the `NfcId` bytes; a real read after `activate()` requires the ISO 14443-4 APDU read path
- First-time pairing stores a placeholder MAC; the mobile app should provide its real MAC via NDEF or a writable characteristic
- Flash writes go through the NVMC driver directly; production builds should use `sd_flash_write` when S140 is loaded

#### Future Enhancements

- Replace placeholder MAC in first-time pairing with NDEF-provided or app-written MAC
- LED animation engine (WS2812B via PWM on P0.11)
- Madgwick AHRS sensor fusion
- Multiple bonded devices across flash pages
- OTA firmware update via DFU bootloader

### Sensor Data Structure

Data is packed into a 12-byte `SensorData` struct for efficient BLE transmission.

```rust
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorData {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
}

impl SensorData {
    pub fn new(
        accel_x: i16,
        accel_y: i16,
        accel_z: i16,
        gyro_x: i16,
        gyro_y: i16,
        gyro_z: i16,
    ) -> Self { /* ... */ }

    pub fn size() -> usize {
        core::mem::size_of::<SensorData>()
    }
}
```

### Sensor Sampling

- Frequency: 20 Hz, 50 ms interval
- Method: synchronous I2C reads with error handling
- Fallback: skip failed reads and log warnings
- Communication: non-blocking `try_send()` to BLE channel
- Validation: range-check sensor data before processing

```rust
async fn main_sensor_loop(imu: &mut LSM6DS3TR<I2cInterface<Twim>>) {
    loop {
        if let Some(data) = read_sensor_data(imu) {
            if !validate_sensor_data(&data) {
                continue;
            }

            if detect_upward_thrust(data.accel_z) {
                animate_led_thrust(200);
            }

            let _ = SENSOR_CHANNEL.try_send(data);
        }

        Timer::after_millis(50).await;
    }
}
```

### Refactored Core Functions

| Function                                | Purpose                                                | Status                               |
| --------------------------------------- | ------------------------------------------------------ | ------------------------------------ |
| `init_i2c()`                            | Configure TWIM0 I2C pins and frequency                 | Implemented                          |
| `init_imu()`                            | Initialize LSM6DS3TR sensor                            | Implemented                          |
| `read_sensor_data()`                    | Single IMU read for accelerometer and gyroscope        | Implemented                          |
| `detect_nfc_field()`                    | Wait for NFC field via `NfcT::activate()` with timeout | Implemented in `main.rs`             |
| `nvmc_write_bonded_device()`            | Erase flash page and write bonded device record        | Implemented in `main.rs`             |
| `nvmc_read_bonded_device()`             | Read bonded device record from flash via NVMC          | Implemented in `main.rs`             |
| `pairing::pairing_mode()`               | Full async NFC pairing gate with BLE fallback          | Implemented (async, `embedded` only) |
| `pairing::authenticate_bonded_device()` | MAC + active/paired flag check                         | Implemented and tested               |
| `pairing::authenticate_with_uid()`      | MAC + NFC UID dual-factor check                        | Implemented and tested               |
| `sensor_loop()`                         | Read IMU at 20 Hz, detect thrust, send to BLE channel  | Implemented                          |
| `stream_sensor_data_ble()`              | Drain SENSOR_CHANNEL and send BLE notifications        | Implemented                          |
| `ble_task()`                            | Embassy task: advertise → connect → stream sensor data | Implemented                          |
| `softdevice_task()`                     | Embassy task: drive nrf-softdevice S140 event loop     | Implemented                          |
| `detect_upward_thrust()`                | Motion threshold detection on Z-axis                   | Implemented and tested               |
| `classify_motion()`                     | Classify idle, moderate, intense, or upward thrust     | Implemented and tested               |
| `validate_sensor_data()`                | Range-check IMU readings                               | Implemented and tested               |

Stub functions pending full implementation:

| Function               | Purpose               | Next Step                                |
| ---------------------- | --------------------- | ---------------------------------------- |
| `animate_led_thrust()` | WS2812B LED animation | Implement PWM via `embassy-nrf` on P0.11 |

### Unit Tests

Desktop tests verify core logic without hardware:

- `SensorData` size, alignment, creation, cloning, equality, and `to_bytes`/`from_bytes` round-trip
- Thrust threshold boundaries
- Motion classification (Idle, Moderate, Intense, UpwardThrust) and priority ordering
- Sensor validation edge cases and boundary conditions
- Configuration constants: `THRUST_THRESHOLD`, `NFC_PAIRING_TIMEOUT_SECS`, and `SENSOR_SAMPLING_INTERVAL_MS`
- `BondedDevice` `to_bytes`/`from_bytes`, flag clear-capable setters, and timestamp
- `BondedDevice` struct size (serialised vs in-memory)
- `NfcPairingStatus` and `NfcFieldState` variant coverage (requires `--features nfc`)
- `BondedDeviceFull` (MAC + UID) serialisation round-trip and UID byte offset
- NFC UID validation (`is_valid_uid`, `is_valid_uid_7`)
- Flash record format: `make_flash_record`/`parse_flash_record` round-trip, magic corruption, inactive slot
- Flash storage constants and page-fit assertion for `MAX_BONDED_DEVICES`
- `authenticate_bonded_device` with matching and mismatched MACs
- `authenticate_with_uid` with correct and wrong MAC/UID combinations
- All pairing CRUD helpers and their stub return values
- `get_bonded_devices` and `is_bonded_device` lookups
- `BleConfig` default values, interval calculation, `bonded_only()` whitelist constructor

Run all tests (no NFC types):

```bash
cargo test --test integration_test
```

Run with NFC types included:

```bash
cargo test --test integration_test --features nfc
```

The base run covers **34 tests**. Adding `--features nfc` runs **91 tests** (34 base + 57 NFC/pairing/BLE). Both run on the host target with no target specification required.

### LED Animation: Motion-Triggered Flash

The sword blade features real-time LED animations driven by the accelerometer.

```rust
fn detect_upward_thrust(accel_z: i16) -> bool {
    accel_z > THRUST_THRESHOLD
}

fn animate_thrust_effect(led_strip: &mut LedStrip, duration_ms: u32) {
    for phase in 0..NUM_LEDS {
        for led_idx in 0..NUM_LEDS {
            let brightness = if led_idx <= phase {
                255 * (1.0 - (led_idx as f32 / NUM_LEDS as f32))
            } else {
                0
            };

            led_strip.set_led(led_idx, color_from_brightness(brightness));
        }

        Timer::after_millis(20).await;
    }
}
```

Animation behavior:

| Motion Type    | LED Pattern                                 | Duration   |
| -------------- | ------------------------------------------- | ---------- |
| Upward thrust  | Flash propagates guard to tip in white/blue | 200 ms     |
| Downward slash | Reverse wave or red/orange pulse fade       | 150 ms     |
| Impact hit     | Bright white flash then decay               | 100 ms     |
| Idle/breathing | Slow pulse or dim glow                      | Continuous |
| Motion stop    | Fade to off over 1 second                   | 1000 ms    |

Performance constraints:

- LED update rate: 50 Hz, 20 ms per frame
- Sensor data rate: 20 Hz, 50 ms per sample
- Latency target: under 100 ms from motion to LED update
- LED strip power budget: approximately 100-500 mA, external supply recommended

## 3. Mobile App: Flutter (Dart) - Planned

### Workflow

1. NFC pairing: tap sword to phone to establish bond
2. Scan for "He-man Power Sword"
3. Connect through BLE only if paired MAC matches
4. Subscribe to sensor characteristic at 20 Hz
5. Receive 12-byte packets: `accel_x`, `accel_y`, `accel_z`, `gyro_x`, `gyro_y`, `gyro_z`
6. Analyze motion in real time
7. Optionally send LED feedback to the device
8. Visualize sword trajectory in 3D

### Data Packet Format

```text
┌───────────────────────────────────────────────────┐
│ BLE Notification, 12 bytes                        │
├───────────┬───────────┬───────────────────────────┤
│ Accel X   │ Accel Y   │ Accel Z + Gyro X/Y/Z      │
│ i16       │ i16       │ 4 x i16                   │
├───────────┴───────────┴───────────────────────────┤
│ Frequency: 20 Hz, approximately 50 ms per packet  │
└───────────────────────────────────────────────────┘
```

### Sensor Fusion and Visualization

| Algorithm            | Purpose                                                                   |
| -------------------- | ------------------------------------------------------------------------- |
| Madgwick AHRS        | Combine accelerometer and gyroscope into 3D orientation                   |
| Complementary filter | Blend low-frequency accelerometer data with high-frequency gyroscope data |
| Zero-velocity update | Correct drift by detecting stationary periods                             |

Planned packages:

- `flutter_blue` for BLE communication
- `vector_math` for 3D vector and matrix math
- `syncfusion_flutter_charts` or `flutter_3d_obj` for visualization

Android permissions:

- `BLUETOOTH`
- `BLUETOOTH_ADMIN`
- `ACCESS_FINE_LOCATION`

## 4. Build and Development

### Prerequisites

```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Add ARM target (required for embedded builds)
rustup target add thumbv7em-none-eabihf

# Install Probe-rs for flashing and debugging
cargo install probe-rs-tools
```

**Important**: All embedded build commands must specify the target architecture with `--target thumbv7em-none-eabihf`. Desktop tests run on the host target automatically.

### Flashing the nRF-Softdevice S140 (once per device)

The nRF-Softdevice S140 is a pre-compiled BLE stack from Nordic. It must be flashed to the device **once** before running the application firmware. You only need to repeat this step after a full chip erase.

1. Download S140 v7.x.x from [Nordic's website](https://www.nordicsemi.com/Products/nRF52840/Compatible-downloads) and unzip.
2. Erase the chip: `probe-rs erase --chip nRF52840_xxAA --allow-erase-all`
3. Flash the softdevice: `probe-rs download --verify --binary-format hex --chip nRF52840_xxAA s140_nrf52_7.x.x_softdevice.hex`

After the softdevice is flashed, build and flash the application normally.

### Build and Test Commands

```bash
# Embedded build (requires target specification)
cargo build --features embedded --target thumbv7em-none-eabihf

# Embedded release build, optimized for size
cargo build --release --features embedded --target thumbv7em-none-eabihf

# Desktop unit tests (runs on host target automatically)
cargo test --test integration_test

# Format and lint
cargo fmt
cargo clippy

# Fast compile check
cargo check --features embedded --target thumbv7em-none-eabihf
```

### Feature Flags

The project uses Cargo features to control which dependencies are included:

| Feature    | Purpose                                                                             | Default |
| ---------- | ----------------------------------------------------------------------------------- | ------- |
| `embedded` | Enables Embassy runtime, embassy-nrf HAL, defmt, and all hardware-specific deps     | No      |
| `std`      | Enables standard library support (for testing)                                      | No      |
| `nfc`      | Enables `nfc` and `pairing` modules on the host target; no embedded hardware needed | No      |

The `nfc` feature lets you compile and test all NFC types, pairing logic, and bonded-device management on a desktop machine. The `embedded` feature implicitly activates the same modules plus the async embassy-time helpers.

The `embedded` feature includes Embassy runtime, defmt logging, and hardware-specific dependencies. Tests run without this feature to enable standard library support.

### Build Profile

- Optimization: `-O s`
- LTO: disabled by default
- Codegen units: 1
- Target: `thumbv7em-none-eabihf` (must be specified for embedded builds)

### Flashing the Firmware

```bash
# Build release binary
cargo build --release --features embedded --target thumbv7em-none-eabihf

# Flash to board using probe-rs
probe-rs download target/thumbv7em-none-eabihf/release/xiao_nrf52840_sword

# Or use cargo-flash shortcut
cargo flash --release --features embedded --target thumbv7em-none-eabihf
```

## 5. Debugging with RTT

The firmware uses `defmt` + RTT for real-time logging. Connect a debugger such as J-Link and use:

```bash
# Start RTT viewer
probe-rs rtt

# Or run and view RTT output
cargo run --release --features embedded
```

Expected logs include sensor readings and thrust detection events.

## 6. References

- Rust Embedded
  - [Embassy Framework](https://github.com/embassy-rs/embassy)
  - [nrf52840-hal](https://github.com/nrf-rs/nrf-hal)
  - [lsm6ds3tr driver](https://crates.io/crates/lsm6ds3tr)
- LED Control
  - [ws2812-spi crate](https://crates.io/crates/ws2812-spi)
  - [nRF52840 PWM module](https://docs.rs/nrf52840-hal/latest/nrf52840_hal/pwm/index.html)
  - [smart-leds crate](https://crates.io/crates/smart-leds)
- NFC and Security
  - [nRF52840 NFC Type 2 Tag](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcomp_5_0%2Fnfct_overview.html)
  - [nrf-softdevice bonding](https://github.com/embassy-rs/nrf-softdevice)
  - [NFC documentation, ISO/IEC 14443](https://www.nxp.com/docs/en/user-guide/UM10528.pdf)
- nRF52840
  - [Datasheet](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v3.2.pdf)
  - [NFC NFCT peripheral](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcomp_5_0%2Fnfct_overview.html)
- Sensor Fusion
  - [Madgwick AHRS reference](https://github.com/arduino-libraries/MadgwickAHRS)

## 7. Architecture Decision Records

### ADR-1: NFC as Primary Pairing Gate

- **Decision**: NFC is required for Bluetooth pairing, not just connection
- **Rationale**: Security and user experience; tap-to-pair is intuitive
- **Alternative**: QR code or PIN, rejected because they are less natural for a physical device

### ADR-2: LED Animations Driven by Accelerometer

- **Decision**: Real-time motion detection drives LED flash effects
- **Rationale**: Immediate feedback without phone latency
- **Alternative**: Phone-only control, rejected because it adds too much lag

### ADR-3: Bonded Device Whitelist

- **Decision**: Only paired MAC address can connect after pairing
- **Rationale**: Prevents accidental connection from other devices
- **Alternative**: Require PIN each time, rejected because it has poor UX

## License

[MIT](./LICENSE)
