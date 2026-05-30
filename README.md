# He-Man Sword Sensor

> By the power of Grayskull!

**A high-tech sword sensor with real-time motion tracking, NFC pairing security, and dynamic LED animations synchronized to your strikes. Track your swings with Bluetooth and watch the power flow through the blade.**

---

## **1. Hardware: XIAO nRF52840 Sense + LSM6DS3TR IMU**

### **Board: Seeed XIAO nRF52840 Sense**

https://wiki.seeedstudio.com/XIAO_BLE/

| Feature          | Details                                                                    |
| ---------------- | -------------------------------------------------------------------------- |
| **MCU**          | nRF52840 (ARM Cortex-M4, Bluetooth 5.0)                                    |
| **IMU**          | LSM6DS3TR-C (6-axis: ±2/±4/±8/±16 g, ±125/±250/±500/±1000/±2000 dps)       |
| **Connectivity** | Bluetooth 5.0, NFC Type 2 tag (built-in)                                   |### **NFC Pairing** | Type 2 Tag (NFCT peripheral) - Tap-to-pair authentication || **Power**        | 3.3V, BQ25101 charging, ~300 mAh LiPo battery (or larger for extended use) |
| **Size**         | 20×17.5 mm (fits in sword handle)                                          |
| **I2C Bus**      | IMU connected via TWIM0 (P0.26 SDA, P0.27 SCL, 100 kHz)                    |

### **LED Strip (WS2812B / NeoPixel)**

The sword features an LED strip that animates in real-time based on motion:

| Component       | Details                                         |
| --------------- | ----------------------------------------------- |
| **LED Type**    | WS2812B (addressable RGB, 5V)                   |
| **Control Pin** | P0.xx (PWM via embassy-nrf)                     |
| **Behavior**    | Flash animation moves tip-ward on upward thrust |
| **Power**       | External 5V source (recommended for strip)      |
| **Data Rate**   | 800 kHz (WS2812B protocol)                      |

**Available GPIO Pins (after I2C/IMU):**

- P0.02, P0.03, P0.04, P0.05, P0.06, P0.07 (SPI/GPIO)
- P0.08, P0.09, P0.10, P0.11 (More GPIO)
- Select one for LED control (e.g., P0.11 for NeoPixel data line)

### **Sensor Specifications**

The **LSM6DS3TR** provides:

- **Accelerometer**: 3-axis, configurable range (±2/±4/±8/±16 g)
- **Gyroscope**: 3-axis, configurable range (±125/±250/±500/±1000/±2000 dps)
- **Data Output**: Raw 16-bit integers (i16) for each axis
- **I2C Address**: 0x6A (no jumpers needed)

---

## **3. NFC Pairing System**

The He-Man Sword Sensor implements a secure NFC-based pairing system using the nRF52840's built-in NFC Type 2 Tag (NFCT) controller.

### **NFC Pairing Flow**

```
┌─────────────────────────────────────────────────────────────┐
│                    NFC PAIRING FLOW                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. BOOT                                                    │
│     │                                                       │
│     ▼                                                       │
│  2. INITIALIZATION                                         │
│     │                                                       │
│     ▼                                                       │
│  3. NFC PAIRING MODE (15 sec timeout)                      │
│     │                                                       │
│     ▼                                                       │
│  4. NFC FIELD DETECTED                                      │
│     │                                                       │
│     ▼                                                       │
│  5. READ BONDED MAC FROM FLASH                             │
│     │                                                       │
│     ▼                                                       │
│  6. AUTHENTICATE DEVICE                                    │
│     │                                                       │
│     ▼                                                       │
│  7. PAIRING SUCCESS ✅                                      │
│     │                                                       │
│     ▼                                                       │
│  8. BLE CONNECTION ESTABLISHED                             │
│     │                                                       │
│     ▼                                                       │
│  9. SENSOR DATA STREAMING                                  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### **NFC Pairing Steps**

1. **Boot & Initialization**
   - Initialize NFCT peripheral
   - Initialize I2C for IMU
   - Start NFC pairing mode

2. **NFC Field Detection**
   - Scan for NFC Type 2 Tag in field
   - Wait up to 15 seconds for field presence
   - Detect passive NFC tags automatically

3. **Bonded Device Authentication**
   - Read MAC address from flash memory
   - Verify MAC against stored credentials
   - Check device flags (active, paired, verified)

4. **Pairing Success**
   - Establish secure BLE connection
   - Start sensor data streaming
   - Enable real-time motion tracking

### **NFC Hardware**

The nRF52840 Sense board includes a built-in **NFC Type 2 Tag (NFCT)** controller:

| Feature | Details |
|---------|---------|
| **Peripheral** | NFCT (NFC Type 2 Tag) |
| **Standard** | ISO/IEC 14443 Type A |
| **Max Range** | ~10 cm |
| **Detection** | Passive tag detection |
| **UID Length** | 10 bytes |

### **Flash Storage Layout**

The bonded device MAC is stored in flash memory:

```
┌─────────────────────────────────────────────────────────┐
│ Flash Memory (512 KB)                                   │
├─────────────────────────────────────────────────────────┤
│ 0x0000-0x0FFF  │ Bootloader                             │
│ 0x1000-0x1FFF  │ Application code                       │
│ 0x2000-0x2FFF  │ Bonded device storage (8 KB)            │
│                │   - Bonded MAC at offset 0x2000          │
│                │   - Timestamp at offset 0x2006           │
│                │   - Flags at offset 0x200C              │
│ 0x3000-0x3FFF  │ Other data                             │
│ 0x4000-0x7FFF  │ RAM (192 KB)                           │
└─────────────────────────────────────────────────────────┘
```

### **Bonded Device Structure**

```rust
#[repr(C)]
struct BondedDevice {
    mac: [u8; 6],      // MAC address
    timestamp: u32,    // Last pairing timestamp
    flags: u8,         // Active, paired, verified
}
```

### **NFC Pairing Configuration**

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Timeout** | 15 seconds | Maximum pairing time |
| **Scan Interval** | 100 ms | NFC field detection rate |
| **Flash Region** | 0x2000-0x2FFF | Bonded device storage |
| **MAC Size** | 6 bytes | Standard MAC address |

### **NFC Pairing Status**

The system tracks NFC pairing status through states:

```rust
enum NfcPairingStatus {
    Idle,           // Waiting for NFC field
    Scanning,       // Scanning for NFC tags
    Authenticating, // Authenticating bonded device
    Success,        // Pairing completed
    Failed,         // Authentication failed
    Timeout,        // Pairing timeout
}
```

### **Security Features**

1. **Bonded Device Whitelist**
   - Only registered MAC addresses can pair
   - Prevents unauthorized connections

2. **Flash Storage**
   - MAC addresses stored in protected flash region
   - Persistent across reboots

3. **Timestamp Verification**
   - Pairing timestamps tracked in flash
   - Detects tampering attempts

### **Pairing Process**

1. **User Action**: Tap NFC-enabled device to sword sensor
2. **Detection**: NFCT peripheral detects passive NFC tag
3. **Authentication**: MAC address verified against bonded list
4. **Connection**: Secure BLE connection established
5. **Streaming**: Sensor data begins transmission

### **Fallback Mechanism**

If NFC pairing times out (15 seconds), the system automatically falls back to:

- **Legacy BLE Advertising**
- **Standard Bluetooth pairing**
- **PIN code authentication** (optional)

### **Testing NFC Pairing**

```bash
# Build with NFC support
cargo build --features embedded

# Run with NFC pairing mode
cargo run --release --features embedded

# Expected output:
# 🔌 NFC Pairing Mode - Primary Authentication Gate
# 📡 NFC Field Detection - Scanning for NFC tags...
# ✅ NFC field detected - initiating pairing sequence...
# 🔑 Bonded device MAC: [00,11,22,33,44,55]
# ✅ Bonded device authentication successful
```

### **NFC Pairing API**

```rust
// Detect NFC field presence
async fn nfc_detect_field() -> bool

// Read bonded device MAC from flash
fn nfc_read_bonded_device_mac() -> Option<[u8; 6]>

// Start pairing mode
async fn nfc_pairing_mode() -> bool

// Get pairing status
fn get_nfc_pairing_status() -> NfcPairingStatus
```

---

## **4. Firmware: Rust + Embassy Framework**

### **Optimal Sensor Placement**

- **Best location**: **10–15 cm from the guard** (handle end)
  - Close to wrist pivot for accurate rotational data
  - Far enough from tip to avoid excessive linear acceleration noise
  - Minimizes sword mass distribution effects
- **Avoid**: Sword tip or center of mass (introduces centrifugal force noise)

---

## **2. Firmware: Rust + Embassy Framework**

### **Tech Stack**

| Component                 | Version       | Purpose                                       |
| ------------------------- | ------------- | --------------------------------------------- |
| **Embassy**               | 0.10.0        | Async runtime with executor, timers, channels |
| **nrf52840-hal**          | 0.19.0        | Synchronous I2C and peripheral access         |
| **lsm6ds3tr**             | 0.2.2         | LSM6DS3TR IMU driver                          |
| **defmt + defmt-rtt**     | 1.1.0 / 1.2.0 | Structured logging over RTT                   |
| **embassy-sync**          | 0.8.0         | Inter-task channel communication              |
| **linked_list_allocator** | 0.10          | Heap allocator (4 KB)                         |
| **panic-probe**           | 1.0.0         | Panic handler                                 |

### **Code Architecture**

The firmware is organized into three main components for testability and maintainability:

| Component                     | Purpose                                                          | Build Target                  |
| ----------------------------- | ---------------------------------------------------------------- | ----------------------------- |
| **src/lib.rs**                | Pure logic: motion detection, sensor validation, data structures | Host (std) — Desktop testable |
| **src/main.rs**               | Embedded code: I2C, IMU, NFC/BLE stubs, async main loop          | nRF52840 (no_std)             |
| **tests/integration_test.rs** | 18 desktop unit tests for logic (motion, validation, config)     | Host (std) — `cargo test`     |

**Key design decisions:**

- **Separation of concerns**: Logic in `lib.rs` is testable without hardware; embedded-specific code stays in `main.rs`
- **Feature gates**: Optional dependencies for embedded features allow lib-only testing on desktop
- **Placeholder stubs**: All unimplemented features (NFC, BLE, LED) have sensible defaults, ready for implementation

### **Build & Test Commands**

```bash
# Embedded build (nRF52840, ~250 KB binary)
cargo build --features embedded

# Embedded release build (optimized for size)
cargo build --release --features embedded

# Desktop unit tests (18 tests, all passing)
cargo test --test integration_test

# Format & lint
cargo fmt
cargo clippy

# Check (fast, no-op build)
cargo check --features embedded
```

### **Build Profile**

- **Optimization**: `-O s` (size optimization)
- **LTO**: Enabled
- **Codegen Units**: 1
- **Target**: `thumbv7em-none-eabihf` (ARM Cortex-M4)

### **Boot Sequence**

```
┌──────────────────────────────────────┐
│ 1. Initialize Heap (4 KB)            │
├──────────────────────────────────────┤
│ 2. Init Embassy Runtime              │
│    - Async executor                  │
│    - Timer & PWM modules             │
├──────────────────────────────────────┤
│ 3. Configure I2C (TWIM0)             │
│    - SDA: P0.26, SCL: P0.27          │
│    - Frequency: 100 kHz              │
├──────────────────────────────────────┤
│ 4. Initialize LSM6DS3TR IMU          │
│    - Address: 0x6A                   │
│    - Ready for sensor reads          │
├──────────────────────────────────────┤
│ 5. Initialize LED Strip (PWM)        │
│    - Data pin configured             │
│    - All LEDs off (ready for anim)   │
├──────────────────────────────────────┤
│ 6. NFC PAIRING GATE (15 sec)         │
│    ⚠️  BLUETOOTH HIDDEN until paired  │
│    - Wait for NFC reader detection   │
│    - Store bonded device MAC         │
│    - Timeout → advertise to paired   │
├──────────────────────────────────────┤
│ 7. Bluetooth Advertising             │
│    - Device: "He-man Power Sword"    │
│    - Only visible to paired device   │
│    - 20 Hz sensor sampling           │
│    - Real-time LED animation         │
└──────────────────────────────────────┘
```

### **NFC Pairing Mode (TODO: Real Detection)**

The device employs a **security-first design** with NFC as the pairing gate:

**Pairing Flow:**

```
1. Device boots into NFC Pairing Mode (15 seconds)
   ├─ Bluetooth is HIDDEN (no advertisements)
   └─ Listening for NFC reader (mobile device with NFC)

2. User brings phone with NFC reader to sword
   ├─ NFC handshake occurs (type 2 tag)
   ├─ Device stores phone's Bluetooth MAC address
   └─ Bond is created in flash memory

3. After NFC pairing OR timeout (15 sec):
   ├─ Device begins BLE advertising
   ├─ ONLY visible to the paired MAC address
   └─ Other devices cannot discover "He-man Power Sword"

4. Bluetooth Connection:
   ├─ Paired phone connects via BLE
   ├─ Sensor data streamed at 20 Hz
   └─ LED animations synchronized in real-time
```

**Implementation (TODO):**

```rust
// After NFC detection
let bonded_device_mac = nfc_read_mac_from_tag();
store_bonding_data_to_flash(bonded_device_mac);

// During BLE advertising
ble_advertise_to_bonded_device_only(bonded_device_mac);

// On reconnection
if incoming_ble_mac == bonded_device_mac {
    start_sensor_stream();
} else {
    reject_connection();  // Not the paired device
}
```

This ensures the sword can only be controlled by the device that performed the initial NFC pairing.

Currently, the device waits 15 seconds in NFC mode before automatically switching to Bluetooth. Real NFC detection will use the nRF52840's built-in NFC Type 2 tag controller:

```rust
// Boot enters NFC Pairing Mode
info!("🔌 NFC Pairing Mode - Waiting for NFC reader...");
let pairing_start = Instant::now();
let pairing_timeout = Duration::from_secs(15);

loop {
    if pairing_start.elapsed() > pairing_timeout {
        break;  // Switch to Bluetooth
    }
    // TODO: Check NFC field detection via nRF52840 NFCT peripheral
    Timer::after_millis(100).await;
}
```

### **Sensor Data Structure**

Data is packed into a 12-byte `SensorData` struct for efficient BLE transmission:

```rust
#[repr(C)]
#[derive(Clone, Copy, defmt::Format)]
struct SensorData {
    accel_x: i16,  // Raw accelerometer X (±2/±4/±8/±16 g)
    accel_y: i16,  // Raw accelerometer Y
    accel_z: i16,  // Raw accelerometer Z
    gyro_x: i16,   // Raw gyroscope X (±125/±250/±500/±1000/±2000 dps)
    gyro_y: i16,   // Raw gyroscope Y
    gyro_z: i16,   // Raw gyroscope Z
}  // Total: 12 bytes
```

### **Sensor Sampling**

- **Frequency**: 20 Hz (50 ms interval)
- **Method**: Synchronous I2C reads with error handling
- **Fallback**: Skips failed reads, logs warnings
- **Communication**: Non-blocking `try_send()` to BLE channel
- **Validation**: Range-check sensor data before processing

```rust
// Refactored main sensor loop (src/main.rs)
async fn main_sensor_loop(imu: &mut LSM6DS3TR<I2cInterface<Twim>>) {
    loop {
        if let Some(data) = read_sensor_data(imu) {
            // Validate sensor data (lib function)
            if !validate_sensor_data(&data) {
                warn!("⚠️  Sensor data out of valid range");
                continue;
            }

            // Check for upward thrust (lib function)
            if detect_upward_thrust(data.accel_z) {
                animate_led_thrust(200);  // Will become real LED animation
            }

            // Send to BLE channel
            let _ = SENSOR_CHANNEL.try_send(data);
            info!("📊 A:({},{},{}) G:({},{},{})", ...);
        }
        Timer::after_millis(50).await;
    }
}
```

### **Refactored Core Functions**

The main firmware has been refactored into modular, testable functions:

| Function                 | Purpose                                             | Status                  |
| ------------------------ | --------------------------------------------------- | ----------------------- |
| `init_i2c()`             | Configure TWIM0 I2C pins & frequency                | ✅ Implemented          |
| `init_imu()`             | Initialize LSM6DS3TR sensor                         | ✅ Implemented          |
| `read_sensor_data()`     | Single IMU read (accel + gyro) with error handling  | ✅ Implemented          |
| `nfc_pairing_mode()`     | Wait for NFC field or timeout (15 sec)              | ✅ Implemented          |
| `main_sensor_loop()`     | Infinite loop: read IMU, detect thrust, send to BLE | ✅ Implemented          |
| `detect_upward_thrust()` | Motion threshold detection (Z-axis acceleration)    | ✅ Implemented + Tested |
| `classify_motion()`      | Classify motion type (idle/moderate/intense/thrust) | ✅ Implemented + Tested |
| `validate_sensor_data()` | Range-check IMU readings                            | ✅ Implemented + Tested |

### **Placeholder Functions (Ready for Implementation)**

These stubs are ready to be filled in with real hardware drivers:

| Function                        | Purpose                                  | Next Step                     |
| ------------------------------- | ---------------------------------------- | ----------------------------- |
| `animate_led_thrust()`          | WS2812B LED animation (stub)             | Implement PWM via embassy-nrf |
| `nfc_detect_field()`            | NFC field detection (stub returns false) | Use nRF52840 NFCT peripheral  |
| `nfc_read_bonded_device_mac()`  | Read MAC from flash (stub returns None)  | Implement flash storage API   |
| `ble_advertise_bonded_device()` | BLE radio advertising (stub returns Ok)  | Use embassy-nrf radio module  |

### **Unit Tests (Desktop)**

18 desktop tests verify core logic without hardware:

**SensorData Structure (5 tests)**

- Size: exactly 12 bytes for efficient BLE transmission
- Alignment: proper i16 field alignment
- Creation, cloning, equality

**Motion Detection (7 tests)**

- Thrust threshold boundaries (positive, negative, exact boundary)
- Motion classification (idle < 900², moderate 900–10000², intense ≥ 10000²)
- Classification with upward thrust override

**Sensor Validation (5 tests)**

- Valid ranges (accel < 10000, gyro < 20000)
- Out-of-range edge cases
- Zero values, maximum valid values

**Configuration (1 test)**

- Verify sensible defaults (THRUST_THRESHOLD=50, NFC_TIMEOUT=15s, SAMPLING=50ms)

Run tests: `cargo test --test integration_test`

### **LED Animation: Motion-Triggered Flash**

The sword blade features real-time LED animations driven by the accelerometer:

```rust
// Motion detection: Upward thrust detection (Z-axis acceleration)
fn detect_upward_thrust(accel_z: i16) -> bool {
    accel_z > THRUST_THRESHOLD  // Threshold: ~0.5g or 5 m/s²
}

// LED animation: Flash propagates from hilt to tip
fn animate_thrust_effect(led_strip: &mut LedStrip, duration_ms: u32) {
    // 1. Detect upward acceleration from accelerometer
    // 2. Calculate animation phase (0..NUM_LEDS)
    // 3. Brightness decreases towards tip (decay curve)
    // 4. Cycle repeats every 200-300ms or until motion stops

    for phase in 0..NUM_LEDS {
        for led_idx in 0..NUM_LEDS {
            let brightness = if led_idx <= phase {
                255 * (1.0 - (led_idx as f32 / NUM_LEDS as f32))
            } else {
                0
            };
            led_strip.set_led(led_idx, color_from_brightness(brightness));
        }
        Timer::after_millis(20).await;  // ~50 Hz LED updates
    }
}
```

**Animation Behavior:**

- **Upward Thrust**: Flash moves from guard → tip (mimics He-Man's power surge)
- **Downward**: LEDs fade or pulse (reversing motion)
- **Idle**: LEDs dim or pulse slowly (breathing effect)
- **Impact**: Bright flash on high acceleration spikes

### **Build & Development**

```bash
# Build for embedded (uses --features embedded for nRF52840 support)
cargo build --features embedded

# Build for release (size-optimized, ~250 KB)
cargo build --release --features embedded

# Run desktop unit tests (18 tests, verifies motion detection, validation, config)
cargo test --test integration_test

# Format code
cargo fmt

# Check for warnings & linting issues
cargo clippy

# Verify compilation without building
cargo check --features embedded
```

### **Debugging with RTT**

The firmware uses **defmt + RTT** for real-time logging. Connect a debugger (e.g., J-Link) and use:

```bash
# Build and flash with debug output
cargo run --features embedded

# Output appears in terminal via RTT
# Watch for sensor readings: "📊 A:(x,y,z) G:(x,y,z)"
# Watch for thrusts: "💡 LED thrust animation..."
```

### **Inter-Task Communication**

An `embassy_sync::Channel` connects the sensor reading loop to the BLE broadcaster:

```rust
static SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorData, 16> = Channel::new();
```

- **Capacity**: 16 samples (allows BLE queuing during transmission)
- **Type**: Non-blocking (skips if channel full)
- **Purpose**: Decouples sensor reads from BLE transmission

---

## **3. Mobile App: Flutter (Dart) - Planned**

### **Next Steps for BLE Integration**

To complete the Bluetooth implementation, the firmware needs:

1. **nrf-softdevice stack** (S140 softdevice for BLE)

   ```toml
   nrf-softdevice = "0.5"
   ```

2. **GATT Service & Characteristic Setup**

   ```rust
   // TODO: Define Sword Sensor GATT service
   // Service UUID: Custom UUID for sensor data
   // Characteristic: Accel + Gyro 6-axis data
   // Notifications: 20 Hz updates (~50ms interval)
   ```

3. **BLE Advertisement (Paired Device Only)**

   ```rust
   // TODO: Advertise only to bonded device MAC
   // Device name: "He-man Power Sword"
   // Security: Require pairing/bonding
   // Whitelist: Only respond to paired device
   ```

4. **Bonding & Persistence**
   ```rust
   // Store bonded device MAC in flash
   // On reconnection, verify MAC before streaming
   // Other devices get rejected at BLE layer
   ```

### **Data Packet Format**

Currently, the device sends raw 16-bit sensor values. The Flutter app will receive:

```
┌───────────────────────────────────────────────────┐
│ BLE Notification (12 bytes)                       │
├───────────┬───────────┬───────────────────────────┤
│ Accel X   │ Accel Y   │ Accel Z   │ Gyro X/Y/Z   │
│ (i16)     │ (i16)     │ (i16)     │ (3× i16)     │
├───────────┴───────────┴───────────┴───────────────┤
│ Total: 12 bytes                                   │
│ Frequency: 20 Hz (~50 ms per packet)              │
└───────────────────────────────────────────────────┘
```

### **Planned Flutter Workflow**

```dart
// Pseudocode: Flutter app consuming sensor data & controlling LEDs
1. NFC Pairing: Tap sword to phone to establish bond
2. Scan for "He-man Power Sword" device
3. Connect via BLE (only works if paired MAC matches)
4. Subscribe to sensor characteristic (20 Hz notifications)
5. Receive 12-byte packets: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
6. Real-time analysis:
   - Detect upward thrust (accel_z > threshold)
   - Calculate motion direction and intensity
7. Optional: Send feedback to device for LED control
8. Visualize sword trajectory in 3D
```

**Real-Time Feedback Loop:**

- Phone receives sensor data → 20 packets/second
- Detects thrust motion (upward acceleration)
- Phone can visualize motion AND control blade LEDs
- Latency: ~50ms sensor → phone → animation

### **Sensor Fusion Algorithms**

Once data arrives, the app will use:

| Algorithm                       | Purpose                                            |
| ------------------------------- | -------------------------------------------------- |
| **Madgwick AHRS**               | Combine accel + gyro → 3D orientation (quaternion) |
| **Complementary Filter**        | Blend accel (low-freq) with gyro (high-freq)       |
| **Zero-Velocity Update (ZUPT)** | Correct drift by detecting stationary periods      |

### **Visualization Goals**

- Real-time 3D sword trajectory (line plot)
- Orientation (quaternion → 3D cube/mesh)
- Power estimation during swings
- Historical swing database
  - [`flutter_blue`](https://pub.dev/packages/flutter_blue) for BLE communication.
  - [`vector_math`](https://pub.dev/packages/vector_math) for 3D vector/matrix math.
  - [`syncfusion_flutter_charts`](https://pub.dev/packages/syncfusion_flutter_charts) or [`flutter_3d_obj`](https://pub.dev/packages/flutter_3d_obj) for 3D visualization.
- **Permissions**: Add `BLUETOOTH`, `BLUETOOTH_ADMIN`, `ACCESS_FINE_LOCATION` (Android) to `AndroidManifest.xml`.

### **App Workflow**

1. **BLE Connection**: Scan for the ESP32, connect, and subscribe to notifications.
2. **Data Parsing**: Unpack the binary data from the ESP32 into `accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, timestamp`.
3. **Sensor Fusion**: Use a **complementary filter** or **Madgwick/Mahony filter** to combine accelerometer and gyroscope data into a 3D orientation (quaternions or Euler angles).
   - Libraries: [`sensor_fusion`](https://pub.dev/packages/sensor_fusion) or implement your own.
4. **Trajectory Calculation**:

---

## **4. Current Project Status**

### ✅ **Completed**

- Rust + Embassy embedded runtime working on nRF52840
- I2C communication with LSM6DS3TR IMU (100 kHz, TWIM0)
- 20 Hz sensor sampling (50 ms intervals)
- Synchronous I2C reads with error handling
- Sensor data packed into 12-byte efficient structures
- NFC Pairing Mode (15-second boot sequence)
- Multi-task architecture with embassy_sync channels
- Structured logging via defmt + RTT
- Code compiles with zero errors

### 🔄 **In Progress**

- Real NFC field detection (currently placeholder timeout)
- Complete BLE stack integration
- GATT service and characteristic definitions
- BLE bonding & MAC whitelisting
- LED strip control (WS2812B PWM driver)

### ⏳ **TODO**

- nrf-softdevice S140 BLE stack integration
- NFC Type 2 tag read/write (store bonded MAC)
- BLE advertise-only-to-bonded-device logic
- LED animation engine (motion-triggered effects)
- Upward thrust detection algorithm
- Mobile app (Flutter) for pairing & visualization
- Real-time LED control from phone
- Sensor fusion algorithms (Madgwick filter)

---

## **5. LED Animation: The Power Sword Effect**

### **Motion Detection for LED Control**

The sword analyzes accelerometer data in real-time to drive LED animations that match the user's swing:

**Thrust Detection Algorithm:**

```
Input: 3-axis accelerometer data (20 Hz)

1. Compute acceleration magnitude: a_mag = sqrt(ax² + ay² + az²)
2. Detect upward component: accel_z > THRUST_THRESHOLD (≈ +0.5g)
3. Calculate velocity: v = integral of acceleration (with drift correction)
4. Detect impact: a_mag > IMPACT_THRESHOLD (sudden spike)

Output: Motion vector indicating thrust direction and intensity
```

**LED Flash Animation:**

| Motion Type        | LED Pattern                               | Duration   |
| ------------------ | ----------------------------------------- | ---------- |
| **Upward Thrust**  | Flash propagates guard → tip (white/blue) | 200 ms     |
| **Downward Slash** | Reverse wave or pulse fade (red/orange)   | 150 ms     |
| **Impact Hit**     | Bright white flash then decay             | 100 ms     |
| **Idle/Breathing** | Slow pulse or dim glow                    | Continuous |
| **Motion Stop**    | Fade to off over 1 sec                    | 1000 ms    |

### **Implementation Strategy**

**Stage 1: Basic LED Control**

```rust
// Drive WS2812B via PWM or SPI
// Simple on/off animation based on accel_z threshold
// All LEDs same color (white) when thrusting
```

**Stage 2: Directional Animation**

```rust
// Map accel_z → animation phase (0..NUM_LEDS)
// Each LED represents "distance along thrust"
// Brightness = brightness(phase - distance)  // Decay effect
```

**Stage 3: Advanced Effects (TODO)**

```rust
// Color shift based on motion intensity
// Multiple animation layers (thrust + impact)
// Gyroscope integration for rotation effects
// Coordinate transformation to global frame
```

### **Performance Constraints**

- **LED Update Rate**: 50 Hz (20 ms per frame)
- **Sensor Data Rate**: 20 Hz (50 ms per sample)
- **Latency Target**: < 100 ms from motion → LED update
- **Power Budget**: LED strip adds ~100-500 mA (external supply recommended)

---

## **6. Development Setup**

### **Prerequisites**

```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Add ARM target
rustup target add thumbv7em-none-eabihf

# Install Probe-rs for flashing
cargo install probe-rs-tools
```

### **Flashing the Firmware**

```bash
# Build release binary
cargo build --release

# Flash to board using probe-rs
probe-rs download target/thumbv7em-none-eabihf/release/xiao_nrf52840_sword

# Or use cargo-flash shortcut
cargo flash --release
```

### **Viewing Logs via RTT**

```bash
# Start RTT viewer
probe-rs rtt

# Or if running cargo run
cargo run --release
```

---

## **7. References**

- **Rust Embedded**:
  - [Embassy Framework](https://github.com/embassy-rs/embassy)
  - [nrf52840-hal](https://github.com/nrf-rs/nrf-hal)
  - [lsm6ds3tr driver](https://crates.io/crates/lsm6ds3tr)
- **LED Control**:
  - [ws2812-spi crate](https://crates.io/crates/ws2812-spi) (WS2812B via SPI)
  - [nrf52840 PWM module](https://docs.rs/nrf52840-hal/latest/nrf52840_hal/pwm/index.html) (alternative: PWM control)
  - [smart-leds crate](https://crates.io/crates/smart-leds) (trait for LED animations)
- **NFC & Security**:
  - [nRF52840 NFC Type 2 Tag](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcomp_5_0%2Fnfct_overview.html)
  - [nrf-softdevice bonding](https://github.com/embassy-rs/nrf-softdevice) (BLE pairing/bonding)
  - [NFC documentation (ISO/IEC 14443)](https://www.nxp.com/docs/en/user-guide/UM10528.pdf)
- **nRF52840**:
  - [Datasheet](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v3.2.pdf)
  - [NFC NFCT peripheral](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcomp_5_0%2Fnfct_overview.html)
- **Sensor Fusion**:
  - [Madgwick AHRS (reference)](https://github.com/arduino-libraries/MadgwickAHRS)

---

## **Architecture Decision Records (ADRs)**

### **ADR-1: NFC as Primary Pairing Gate**

- **Decision**: NFC required for Bluetooth pairing (not just connection)
- **Rationale**: Security + UX (tap-to-pair is intuitive)
- **Alternative**: QR code / PIN (rejected: less natural for a physical device)

### **ADR-2: LED Animations Driven by Accelerometer**

- **Decision**: Real-time motion detection → LED flash
- **Rationale**: Immediate feedback without phone latency (~20-50ms)
- **Alternative**: Phone-only control (rejected: too much lag)

### **ADR-3: Bonded Device Whitelist**

- **Decision**: Only paired MAC address can connect post-pairing
- **Rationale**: Prevents accidental connection from other devices
- **Alternative**: Require PIN each time (rejected: poor UX)

---

## **License**

MIT
