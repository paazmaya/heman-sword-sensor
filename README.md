# He-Man Sword Sensor

> By the power of Grayskull!

A high-tech sword sensor with real-time motion tracking, NFC pairing security, and dynamic LED animations synchronized to your strikes. Track your swings with Bluetooth and watch the power flow through the blade.

---

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
- Desktop tests for motion detection, sensor validation, and configuration constants

### In Progress

- Real NFC field detection through the nRF52840 NFCT peripheral
- Complete BLE stack integration
- GATT service and characteristic definitions
- BLE bonding and MAC whitelisting
- WS2812B LED strip control

### TODO

- nrf-softdevice S140 BLE stack integration
- NFC Type 2 tag read/write and bonded MAC persistence
- BLE advertise-only-to-bonded-device logic
- LED animation engine
- Mobile app for pairing, visualization, and real-time LED control
- Sensor fusion algorithms such as Madgwick AHRS

---

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

---

## 2. Firmware: Rust + Embassy Framework

### Tech Stack

| Component                 | Version       | Purpose                                           |
| ------------------------- | ------------- | ------------------------------------------------- |
| **Embassy**               | 0.10.0        | Async runtime with executor, timers, and channels |
| **nrf52840-hal**          | 0.19.0        | Peripheral access and synchronous I2C             |
| **lsm6ds3tr**             | 0.2.2         | LSM6DS3TR IMU driver                              |
| **defmt + defmt-rtt**     | 1.1.0 / 1.2.0 | Structured logging over RTT                       |
| **embassy-sync**          | 0.8.0         | Inter-task channel communication                  |
| **linked_list_allocator** | 0.10          | 4 KB heap allocator                               |
| **panic-probe**           | 1.0.0         | Panic handler                                     |

### Code Architecture

| Component                   | Purpose                                                          | Build Target                 |
| --------------------------- | ---------------------------------------------------------------- | ---------------------------- |
| `src/lib.rs`                | Pure logic: motion detection, sensor validation, data structures | Host `std`, desktop testable |
| `src/main.rs`               | Embedded code: I2C, IMU, NFC/BLE stubs, async main loop          | nRF52840 `no_std`            |
| `tests/integration_test.rs` | Desktop tests for motion, validation, and configuration          | Host `std`, `cargo test`     |

Key design decisions:

- Logic in `src/lib.rs` is testable without hardware
- Embedded-specific code stays in `src/main.rs`
- Optional dependencies and feature gates allow desktop testing
- Unimplemented features such as NFC, BLE, and LED control have sensible stubs

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

The current implementation is still a placeholder: field detection and flash persistence are stubbed, while the timeout and fallback behavior are implemented.

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

| Function                            | Description                | Returns            |
| ----------------------------------- | -------------------------- | ------------------ |
| `nfc::detect_field()`               | Detect NFC field presence  | `bool`             |
| `nfc::read_nfc_uid()`               | Read NFC UID from tag      | `Option<[u8; 10]>` |
| `pairing::pairing_mode()`           | Start pairing sequence     | `bool`             |
| `pairing::read_bonded_device_mac()` | Read MAC from flash        | `Option<[u8; 6]>`  |
| `authenticate_bonded_device()`      | Verify bonded MAC          | `bool`             |
| `get_nfc_pairing_status()`          | Get current pairing status | `NfcPairingStatus` |

Bonded device management planned for flash storage:

| Function                         | Description            | Returns             |
| -------------------------------- | ---------------------- | ------------------- |
| `write_bonded_device_to_flash()` | Write MAC to flash     | `bool`              |
| `register_bonded_device()`       | Register new device    | `bool`              |
| `unregister_bonded_device()`     | Remove device          | `bool`              |
| `get_bonded_devices()`           | Get all bonded devices | `Vec<BondedDevice>` |
| `is_bonded_device()`             | Check if MAC is bonded | `bool`              |

#### Security Considerations

- Only registered MAC addresses can pair
- Bonded MAC addresses persist across reboots
- Timestamp and flags can detect stale or invalid records
- BLE advertising should be restricted to the paired device after bonding is implemented
- NFC UID reading can be added as an extra authentication factor

#### Testing NFC Pairing

```bash
# Build with embedded support
cargo build --features embedded

# Build release image for flashing
cargo build --release --features embedded

# Run desktop tests
cargo test --test integration_test

# Run with NFC pairing mode enabled
cargo run --release --features embedded
```

NFC-specific tests should cover:

- Pairing timeout behavior
- Bonded device structure and flags
- MAC validation
- Flash read/write behavior once persistence is implemented
- NFCT field detection once hardware integration is complete

#### Troubleshooting

| Issue                               | Check                                                         |
| ----------------------------------- | ------------------------------------------------------------- |
| NFC pairing times out               | Verify field detection, NFCT setup, and polling interval      |
| Bonded device not found             | Verify MAC is stored in flash and the flash region is correct |
| Authentication fails                | Verify MAC format, timestamp validity, and device flags       |
| BLE is still visible to all devices | Confirm bonding and whitelist logic are implemented           |

#### Future Enhancements

- Read the full NFC UID from the tag
- Use UID as an additional authentication factor
- Implement BLE bonding and secure connection
- Support multiple bonded devices
- Support additional NFC tag types and custom tag formats
- Add remote provisioning and OTA flash updates

### Sensor Data Structure

Data is packed into a 12-byte `SensorData` struct for efficient BLE transmission.

```rust
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

| Function                 | Purpose                                            | Status                       |
| ------------------------ | -------------------------------------------------- | ---------------------------- |
| `init_i2c()`             | Configure TWIM0 I2C pins and frequency             | Implemented                  |
| `init_imu()`             | Initialize LSM6DS3TR sensor                        | Implemented                  |
| `read_sensor_data()`     | Single IMU read for accelerometer and gyroscope    | Implemented                  |
| `nfc_pairing_mode()`     | Wait for NFC field or timeout                      | Timeout fallback implemented |
| `main_sensor_loop()`     | Read IMU, detect thrust, send to BLE               | Implemented                  |
| `detect_upward_thrust()` | Motion threshold detection on Z-axis               | Implemented and tested       |
| `classify_motion()`      | Classify idle, moderate, intense, or upward thrust | Implemented and tested       |
| `validate_sensor_data()` | Range-check IMU readings                           | Implemented and tested       |

Placeholder functions:

| Function                        | Purpose               | Next Step                       |
| ------------------------------- | --------------------- | ------------------------------- |
| `animate_led_thrust()`          | WS2812B LED animation | Implement PWM via `embassy-nrf` |
| `nfc_detect_field()`            | NFC field detection   | Use nRF52840 NFCT peripheral    |
| `nfc_read_bonded_device_mac()`  | Read MAC from flash   | Implement flash storage API     |
| `ble_advertise_bonded_device()` | BLE radio advertising | Use `embassy-nrf` radio module  |

### Unit Tests

Desktop tests verify core logic without hardware:

- `SensorData` size, alignment, creation, cloning, and equality
- Thrust threshold boundaries
- Motion classification
- Sensor validation edge cases
- Configuration constants: `THRUST_THRESHOLD`, `NFC_PAIRING_TIMEOUT_SECS`, and `SENSOR_SAMPLING_INTERVAL_MS`

Run tests:

```bash
cargo test --test integration_test
```

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

---

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

---

## 4. Build and Development

### Prerequisites

```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Add ARM target
rustup target add thumbv7em-none-eabihf

# Install Probe-rs for flashing
cargo install probe-rs-tools
```

### Build and Test Commands

```bash
# Embedded build
cargo build --features embedded

# Embedded release build, optimized for size
cargo build --release --features embedded

# Desktop unit tests
cargo test --test integration_test

# Format and lint
cargo fmt
cargo clippy

# Fast compile check
cargo check --features embedded
```

### Build Profile

- Optimization: `-O s`
- LTO: disabled by default
- Codegen units: 1
- Target: `thumbv7em-none-eabihf`

### Flashing the Firmware

```bash
# Build release binary
cargo build --release --features embedded

# Flash to board using probe-rs
probe-rs download target/thumbv7em-none-eabihf/release/xiao_nrf52840_sword

# Or use cargo-flash shortcut
cargo flash --release --features embedded
```

---

## 5. Debugging with RTT

The firmware uses `defmt` + RTT for real-time logging. Connect a debugger such as J-Link and use:

```bash
# Start RTT viewer
probe-rs rtt

# Or run and view RTT output
cargo run --release --features embedded
```

Expected logs include sensor readings and thrust detection events.

---

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

---

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

---

## License

MIT
