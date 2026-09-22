# He-Man Sword Sensor

> By the power of Grayskull!

A high-tech sword sensor with real-time motion tracking, NFC pairing security, and dynamic LED animations synchronized to your strikes. Track your swings with Bluetooth and watch the power flow through the blade.

## Project Status

### Completed

- Rust + Embassy embedded runtime working on nRF52840
- I2C communication with LSM6DS3TR IMU at 100 kHz on TWIM0
- 20 Hz sensor sampling with 50 ms intervals
- Synchronous I2C reads with error handling
- 12-byte sensor data structure for BLE transmission
- 15-second NFC pairing mode with Bluetooth fallback
- Multi-task architecture with `embassy_sync` channels
- Structured logging through `defmt` + RTT
- `embassy-nrf` NFCT peripheral wired up with real hardware types (`NfcId`, `SddPat`, `SelResProtocol`, `Config`); `NfcT::new` + `nfct.activate()` called in `main.rs`
- NFC field detection with configurable timeout via `embassy_time::with_timeout`
- `BondedDevice` struct with MAC, timestamp, and flag helpers (`is_active`, `is_paired`, `is_verified`, setters)
- `NfcPairingStatus` enum covering all pairing lifecycle states
- `pairing` module: `authenticate_bonded_device`, `read_bonded_device_mac`, `read_bonded_device`, `write_bonded_device_to_flash`, `write_bonded_device_to_flash_full`, `register_bonded_device`, `unregister_bonded_device`, `get_bonded_devices`, `is_bonded_device`, `get_nfc_pairing_status`
- `nfc` module: `NfcFieldState`, `get_field_state`, `detect_field_with_timeout`, `detect_field`, `read_nfc_uid`, `read_nfc_page`, `write_nfc_page`
- Top-level `authenticate_bonded_device()` and `get_nfc_pairing_status()` crate re-exports
- `pairing::pairing_mode()` async gate: polls for NFC field, reads bonded MAC, authenticates, falls back to BLE on timeout
- `BleConfig` struct with device name, bonded MAC whitelist, advertising interval, and connection timeout fields
- `ble_advertise_bonded_device()` stub logging the configured parameters
- `nfc` Cargo feature: compiles `nfc` and `pairing` modules on the host target with no embedded hardware required
- Desktop NFC tests: 29 tests covering all types, flag logic, authentication, flash stubs, and bonded device lookup (run with `cargo test --features nfc`)

### In Progress

- NFC Type 2 tag UID read path after `NfcT::activate()` completes (stub returns placeholder bytes)
- Flash read/write via NVMC peripheral (all flash functions are stubs returning simulated values)

### TODO

- Wire NVMC peripheral to replace flash stubs with real read/write
- nrf-softdevice S140 BLE stack integration
- GATT service and characteristic definitions
- BLE advertise-only-to-bonded-device logic
- NFC UID as secondary authentication factor
- LED animation engine (WS2812B via PWM)
- Mobile app for pairing, visualization, and real-time LED control
- Sensor fusion algorithms such as Madgwick AHRS

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

| Component                 | Version   | Purpose                                           |
| ------------------------- | --------- | ------------------------------------------------- |
| **Embassy**               | 0.10.0    | Async runtime with executor, timers, and channels |
| **embassy-nrf**           | 0.11.0    | nRF HAL with NFCT peripheral support              |
| **lsm6ds3tr**             | 0.2.2     | LSM6DS3TR IMU driver                              |
| **defmt + defmt-rtt**     | 0.3 / 0.4 | Structured logging over RTT                       |
| **embassy-sync**          | 0.8.0     | Inter-task channel communication                  |
| **linked_list_allocator** | 0.10.6    | 4 KB heap allocator                               |
| **panic-probe**           | 0.2       | Panic handler                                     |

### Code Architecture

| Component                   | Purpose                                                          | Build Target                 |
| --------------------------- | ---------------------------------------------------------------- | ---------------------------- |
| `src/lib.rs`                | Pure logic: motion detection, sensor validation, data structures | Host `std`, desktop testable |
| `src/main.rs`               | Embedded code: I2C, IMU, NFC/BLE stubs, async main loop          | nRF52840 `no_std`            |
| `tests/integration_test.rs` | Desktop tests for motion, validation, and configuration          | Host `std`, `cargo test`     |

Key design decisions:

- Logic in `src/lib.rs` is testable without hardware using standard library traits (`Debug`, `PartialEq`, `Eq`)
- Embedded-specific code stays in `src/main.rs` with embedded-specific traits (`defmt::Format`)
- Optional dependencies and feature gates allow desktop testing
- Unimplemented features such as NFC, BLE, and LED control have sensible stubs
- Integration tests automatically run on host target without manual target specification

### Boot Sequence

```text
1. Initialize 4 KB heap
2. Initialize Embassy runtime
   - Async executor
   - Timer and PWM modules
3. Configure I2C on TWIM0
   - SDA: P0.26
   - SCL: P0.27
   - Frequency: 100 kHz
4. Initialize LSM6DS3TR IMU at address 0x6A
5. Initialize LED strip placeholder
6. NFC pairing gate
   - Wait up to 15 seconds for NFC field detection
   - Store bonded device MAC when implemented
   - Fall back to Bluetooth advertising on timeout
7. Bluetooth advertising
   - Device name: "He-man Power Sword"
   - Visible only to paired device after bonding is implemented
   - 20 Hz sensor sampling
   - Real-time LED animation
```

### NFC Pairing

The firmware uses NFC as the primary pairing gate. On boot, it waits up to 15 seconds for an NFC field. If a field is detected, it reads the bonded device MAC from flash and authenticates the device. If NFC times out, it falls back to legacy BLE advertising.

The current implementation uses embassy-nrf's NFCT peripheral with real hardware types (NfcId, SddPat, SelResProtocol, Config). The implementation demonstrates the exact embassy-nrf API calls for hardware activation: `NfcT::new(p.NFCT, Irqs, &config)` and `nfct.activate()`. Full hardware activation requires migration from nrf52840-hal to embassy-nrf HAL for complete embassy runtime integration.

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

Bonded device data is planned for a dedicated flash region starting at 0x2000.

```text
┌─────────────────────────────────────────────────────────┐
│ nRF52840 address map excerpt                            │
├─────────────────────────────────────────────────────────┤
│ 0x0000-0x0FFF  │ Bootloader                             │
│ 0x1000-0x1FFF  │ Application code                       │
│ 0x2000-0x2FFF  │ Bonded device storage, 8 KB             │
│                │   - MAC address                         │
│                │   - Pairing timestamp                   │
│                │   - Device flags                        │
│ 0x3000-0x3FFF  │ Reserved                               │
│ 0x4000-0x7FFF  │ RAM, 192 KB                            │
└─────────────────────────────────────────────────────────┘
```

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

`nfc` module (sync helpers available with `nfc` or `embedded`; async requires `embedded`):

| Function                           | Description                                       | Returns            |
| ---------------------------------- | ------------------------------------------------- | ------------------ |
| `nfc::get_field_state()`           | Current field state                               | `NfcFieldState`    |
| `nfc::detect_field()`              | Detect NFC field (500 ms default timeout) — async | `bool`             |
| `nfc::detect_field_with_timeout()` | Detect NFC field with custom timeout — async      | `bool`             |
| `nfc::read_nfc_uid()`              | Read 10-byte UID from tag — async stub            | `Option<[u8; 10]>` |
| `nfc::read_nfc_page()`             | Read 4-byte Type 2 tag page — async stub          | `Option<[u8; 4]>`  |
| `nfc::write_nfc_page()`            | Write 4-byte Type 2 tag page — async stub         | `bool`             |

`pairing` module (sync helpers available with `nfc` or `embedded`; async requires `embedded`):

| Function                                       | Description                                      | Returns                |
| ---------------------------------------------- | ------------------------------------------------ | ---------------------- |
| `pairing::read_bonded_device_mac()`            | Read MAC from flash (stub)                       | `Option<[u8; 6]>`      |
| `pairing::read_bonded_device()`                | Read full `BondedDevice` (stub)                  | `Option<BondedDevice>` |
| `pairing::authenticate_bonded_device()`        | Verify MAC against stored record                 | `bool`                 |
| `pairing::get_nfc_pairing_status()`            | Current pairing status                           | `NfcPairingStatus`     |
| `pairing::write_bonded_device_to_flash()`      | Write MAC to flash (stub)                        | `bool`                 |
| `pairing::write_bonded_device_to_flash_full()` | Write full struct to flash (stub)                | `bool`                 |
| `pairing::register_bonded_device()`            | Register new device                              | `bool`                 |
| `pairing::unregister_bonded_device()`          | Remove device                                    | `bool`                 |
| `pairing::get_bonded_devices()`                | Get all bonded devices (stub)                    | `[BondedDevice; 2]`    |
| `pairing::is_bonded_device()`                  | Check if MAC is bonded                           | `bool`                 |
| `pairing::pairing_mode()`                      | Full async pairing gate — async, `embedded` only | `bool`                 |

#### Security Considerations

- Only registered MAC addresses can pair
- Bonded MAC addresses persist across reboots
- Timestamp and flags can detect stale or invalid records
- BLE advertising should be restricted to the paired device after bonding is implemented
- NFC UID reading can be added as an extra authentication factor

#### Testing NFC Pairing

```bash
# Build with embedded support
cargo build --features embedded --target thumbv7em-none-eabihf

# Build release image for flashing
cargo build --release --features embedded --target thumbv7em-none-eabihf

# Run desktop tests (18 tests, no feature flag needed)
cargo test --test integration_test

# Run desktop tests including all NFC types (47 tests)
cargo test --test integration_test --features nfc

# Run on hardware (requires embedded target)
cargo run --release --features embedded --target thumbv7em-none-eabihf
```

NFC-specific tests cover:

- `NfcPairingStatus` and `NfcFieldState` all variants
- `BondedDevice` flag logic (active, paired, verified) and timestamp helpers
- `authenticate_bonded_device` with matching and wrong MACs
- Flash storage constants (`FLASH_BONDED_DEVICE_START`, `FLASH_BONDED_DEVICE_SIZE`, `BONDED_DEVICE_STRUCT_SIZE`)
- All pairing stub functions (`read_bonded_device_mac`, `write_bonded_device_to_flash`, `register_bonded_device`, etc.)
- `get_bonded_devices` and `is_bonded_device` lookups

#### Troubleshooting

| Issue                               | Check                                                         |
| ----------------------------------- | ------------------------------------------------------------- |
| NFC pairing times out               | Verify field detection, NFCT setup, and polling interval      |
| Bonded device not found             | Verify MAC is stored in flash and the flash region is correct |
| Authentication fails                | Verify MAC format, timestamp validity, and device flags       |
| BLE is still visible to all devices | Confirm bonding and whitelist logic are implemented           |

#### Current Implementation Status

The NFC implementation uses `embassy-nrf` v0.11.0 with real hardware types and calls the actual hardware API at boot:

**Hardware types in use:**

- `NfcId::SingleSize([0x01, 0x02, 0x03, 0x04])` — NFC identifier configuration
- `SddPat::Sdd00000` — single device detection pattern
- `SelResProtocol::Type2` — Type 2 Tag protocol selection
- `Config` struct with proper `embassy-nrf` field names

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

**Library API (`src/lib.rs`, available with `nfc` or `embedded` feature):**

- `NfcPairingStatus` — six-variant enum covering the full pairing lifecycle
- `BondedDevice` — `repr(C)` struct with MAC, timestamp, flag helpers
- `nfc::NfcFieldState`, `nfc::get_field_state()`
- `nfc::detect_field_with_timeout()`, `nfc::detect_field()` — async, `embedded` only
- `nfc::read_nfc_uid()`, `nfc::read_nfc_page()`, `nfc::write_nfc_page()` — async stubs, `embedded` only
- `pairing::authenticate_bonded_device()` — MAC + active/paired flag check
- `pairing::pairing_mode()` — full async pairing gate, `embedded` only
- All flash CRUD helpers (`read_bonded_device_mac`, `write_bonded_device_to_flash`, `register_bonded_device`, `unregister_bonded_device`, `get_bonded_devices`, `is_bonded_device`) — stubs returning simulated values until NVMC is wired
- Top-level crate re-exports: `authenticate_bonded_device()`, `get_nfc_pairing_status()`

**What remains stubbed:**

- Flash functions use hard-coded return values; NVMC peripheral integration is pending
- `read_nfc_uid`, `read_nfc_page`, `write_nfc_page` return placeholder data; the real implementation reads/writes NFCT memory after `activate()` completes

#### Future Enhancements

- Wire NVMC peripheral to replace flash stubs with real read/write
- Read the full NFC UID from the tag after `activate()` and use it as a secondary authentication factor
- Implement BLE bonding and secure connection via nrf-softdevice
- Support multiple bonded devices stored across flash pages
- Support additional NFC tag types and custom NDEF record formats
- Remote provisioning and OTA flash updates

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
| `detect_nfc_field_real()`               | Wait for NFC field via `NfcT::activate()` with timeout | Implemented in `main.rs`             |
| `pairing::pairing_mode()`               | Full async NFC pairing gate with BLE fallback          | Implemented (async, `embedded` only) |
| `pairing::authenticate_bonded_device()` | MAC + active/paired flag check                         | Implemented                          |
| `main_sensor_loop()`                    | Read IMU, detect thrust, send to BLE                   | Implemented                          |
| `detect_upward_thrust()`                | Motion threshold detection on Z-axis                   | Implemented and tested               |
| `classify_motion()`                     | Classify idle, moderate, intense, or upward thrust     | Implemented and tested               |
| `validate_sensor_data()`                | Range-check IMU readings                               | Implemented and tested               |

Stub functions pending full implementation:

| Function                                  | Purpose                 | Next Step                                |
| ----------------------------------------- | ----------------------- | ---------------------------------------- |
| `nfc::read_nfc_uid()`                     | Read UID after activate | Wire NFCT memory read post-`activate()`  |
| `nfc::read_nfc_page()`                    | Read Type 2 tag page    | Wire NFCT memory read post-`activate()`  |
| `nfc::write_nfc_page()`                   | Write Type 2 tag page   | Wire NFCT memory write post-`activate()` |
| `pairing::read_bonded_device_mac()`       | Read MAC from flash     | Implement NVMC peripheral read           |
| `pairing::write_bonded_device_to_flash()` | Write MAC to flash      | Implement NVMC peripheral write          |
| `animate_led_thrust()`                    | WS2812B LED animation   | Implement PWM via `embassy-nrf`          |
| `ble_advertise_bonded_device()`           | BLE radio advertising   | Integrate nrf-softdevice S140            |

### Unit Tests

Desktop tests verify core logic without hardware:

- `SensorData` size, alignment, creation, cloning, and equality
- Thrust threshold boundaries
- Motion classification (Idle, Moderate, Intense, UpwardThrust)
- Sensor validation edge cases
- Configuration constants: `THRUST_THRESHOLD`, `NFC_PAIRING_TIMEOUT_SECS`, and `SENSOR_SAMPLING_INTERVAL_MS`
- `NfcPairingStatus` and `NfcFieldState` variant coverage (requires `--features nfc`)
- `BondedDevice` construction, flag setters (`set_active`, `set_paired`, `set_verified`), and timestamp
- `authenticate_bonded_device` with matching and mismatched MACs
- Flash storage constants and all pairing stub return values
- `get_bonded_devices` and `is_bonded_device` lookups

Run all tests (no NFC types):

```bash
cargo test --test integration_test
```

Run with NFC types included:

```bash
cargo test --test integration_test --features nfc
```

The base run covers 18 tests. Adding `--features nfc` runs 47 tests (18 existing + 29 NFC-specific). Both run on the host target with no target specification required.

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

# Install Probe-rs for flashing
cargo install probe-rs-tools
```

**Important**: All embedded build commands must specify the target architecture with `--target thumbv7em-none-eabihf`. Desktop tests run on the host target automatically.

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
