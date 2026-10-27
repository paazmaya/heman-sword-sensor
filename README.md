# He-Man Sword Sensor

Rust firmware for a **Seeed XIAO nRF52840 Sense** prop: onboard LSM6DS3TR
motion sensing, relative orientation over BLE, and a 60-pixel WS2812B blade.
Embassy runs the hardware tasks; Nordic S140 provides the BLE stack.

**Validation boundary:** host tests and ELF inspection do not establish that the
board boots or that radio, NFC, timing, power, or LEDs work on hardware.
Hardware smoke testing is still required. This is not a safety-certified prop.

## Developer quickstart

Use the stable Rust toolchain selected by `rust-toolchain.toml`. Host tests need
no board, SoftDevice image, or embedded feature. Run from the repository root:

```sh
rustup component add rustfmt clippy
cargo fmt --all -- --check
cargo check --locked --no-default-features
cargo test --locked
cargo test --locked --features nfc
cargo test --locked --release
cargo test --locked --release --features nfc
cargo clippy --locked --all-targets --features nfc -- -D warnings
RUSTDOCFLAGS='-D warnings -D missing_docs' cargo doc --locked --no-deps --features nfc
cargo test --locked --doc --features nfc
```

Open [generated API documentation](target/doc/xiao_nrf52840_sword/index.html)
after `cargo doc`; its examples describe the actual public APIs.
For host-library line coverage, install `cargo-tarpaulin` and run:

```sh
cargo tarpaulin --locked --features nfc --engine llvm --out Xml --fail-under 95
```

The CI gate is **95% host-library line coverage**. The latest local NFC-feature
run passed **165 integration tests plus 3 doctests**, measuring **98.20%**
(818/833 executable host-library lines). This is not branch or embedded
coverage. NFC-feature tests matter: default features do not exercise all
pairing paths. Targeted mutations of byte order, a threshold boundary, an LED
terminal frame, and the flash write were all rejected by the regression tests.

Features: default is empty; `nfc` enables host-testable NFC/pairing logic;
`embedded` enables the ARM firmware and hardware dependencies. `std` is currently
an empty compatibility feature, not a simulator or a different runtime.

## Architecture and runtime

The library contains deterministic logic; firmware adapters own peripherals.
Compatibility re-exports in `lib.rs` preserve existing import paths.
Persistence helpers now require an explicit backend and return errors instead
of reporting success without writing. `BondedDeviceFull` encodes the actual
22-byte flash envelope (including magic), and decoding can reject invalid data;
the previous 21-byte helper format was inconsistent with firmware storage.

| Location                                         | Responsibility                                                      |
| ------------------------------------------------ | ------------------------------------------------------------------- |
| `src/lib.rs`                                     | Crate documentation, feature gates, re-exports                      |
| `src/sensor.rs`, `src/motion.rs`                 | Sensor codecs/units and gesture rules                               |
| `src/led.rs`, `src/fusion.rs`                    | Animation/PWM encoding and Madgwick orientation                     |
| `src/api.rs`, `src/control.rs`                   | BLE wire codecs and command dispatch/outcomes                       |
| `src/nfc.rs`, `src/pairing.rs`, `src/storage.rs` | Local tag identity, registration, flash records                     |
| `src/main.rs`                                    | Composition, task wiring, sampling; calls `process_storage_request` |
| `src/firmware/board.rs`, `ble.rs`                | Board/IMU setup and GATT/advertising                                |
| `src/firmware/flash.rs`, `led.rs`, `nfc.rs`      | Ordered storage processing, PWM output, NFC boot gate               |
| `tests/`                                         | Host regressions and protocol/persistence scenarios                 |

Boot initializes the board, IMU, SoftDevice, and GATT server, then waits up to
**15 seconds** for an external NFC reader to select the local tag. BLE advertising
starts after this gate. NFC selection is **boot-only**; there is no continuous
re-tap/re-pair loop.

On timeout, an existing usable record selects whitelist advertising; without a
usable record, advertising remains open. Initial registration may store a
placeholder MAC until the client writes a real address. Successful MAC
persistence interrupts advertising immediately to refresh the whitelist without
rebooting. While connected, the update applies only after that connection ends;
it neither disconnects nor re-authenticates the active central. MAC replacement
and clear requests share one storage FIFO, preserving their arrival order.

The sampling target is **20 Hz** and LED rendering is **50 Hz**, not guaranteed
wall-clock delivery rates. Latest-value signals overwrite unread telemetry;
responses have priority. On reconnect, latest data and queued events reset, and
a connection epoch prevents stale deferred ACK delivery. BLE notification-buffer
overflow is logged and telemetry skipped, not delivered losslessly.

IMU packets contain raw counts, not SI units. Configured scales are ±2 g and
±245 degrees/s (approximately 0.061 mg/LSB and 8.75 mdps/LSB).
Gesture thresholds are uncalibrated raw-count settings, not physical impact or
jerk measurements; gravity and mounting affect classification.

Fusion estimates **relative orientation**, not position. There is no
magnetometer-based absolute yaw reference; yaw drifts. Acceleration includes
gravity. Do not interpret orientation or gesture output as position tracking.
Manual LED mode suppresses motion overrides. Thrust/slash/impact effects return
to breathing; MotionStop fades to black and remains Off.

For a change, find the domain module above, add a focused regression under
`tests/`, run that test with `--features nfc` where needed, then run the checks
above. Hardware adapter changes also need the embedded build and smoke checks.

## Board wiring and electrical safety

| Function                 | Pin / connection                                            |
| ------------------------ | ----------------------------------------------------------- |
| External WS2812B data    | **D0 / P0.02**, PWM0 output                                 |
| Onboard IMU SDA / SCL    | **P0.07 / P0.27**                                           |
| Onboard IMU power enable | **P1.08**, held high                                        |
| Onboard IMU interrupt    | **P0.11**; not the external LED output                      |
| NFC                      | Dedicated NFC antenna connection; attach a suitable antenna |

The onboard IMU is at I²C address `0x6A`. Its pins are internal board wiring:
do not repurpose them for the strip. Follow Seeed's
[Sense board pin mapping](https://github.com/Seeed-Studio/Adafruit_nRF52_Arduino/blob/master/variants/Seeed_XIAO_nRF52840_Sense/variant.cpp)
and antenna guidance, not generic nRF52840 pin diagrams.

- Budget conservatively for **about 3.6 A at 5 V** for 60 pixels at full white
  using the traditional **60 mA/pixel** estimate. Verify the actual LED variant;
  this is a worst-case design estimate, not a measurement.
- Use an appropriately rated **external regulated supply**, wiring, connectors,
  current limiting/fusing, and a **common ground** with the controller.
  Do not power the strip through the board's GPIO or assume its regulator/USB
  path can supply the strip.
- Use a suitable 3.3 V-to-strip-data **level shifter**; verify logic thresholds
  and signal integrity for the actual strip and supply.
- Use protected batteries and suitable charging/power circuitry. Software
  brightness is not an electrical current limit or battery protection.
- Insulate and strain-relieve connections; enclose batteries and electronics.
  Secure components in a soft, safe prop and avoid striking people or objects.

## One BLE protocol contract

Advertised name: **`He-Man Sword`**.
Service UUID: **`6e400001-b5a3-f393-e0a9-e50e24dcca9e`**.
In this table, `000N` means `6e40000N-b5a3-f393-e0a9-e50e24dcca9e`.
All multi-byte integers are little-endian.

| Characteristic   | UUID   | Properties    | Value                                                                        |
| ---------------- | ------ | ------------- | ---------------------------------------------------------------------------- |
| Sensor           | `0003` | Read, Notify  | 12 bytes: six `i16`, accel XYZ then gyro XYZ                                 |
| Pairing status   | `0004` | Read, Notify  | 1 byte: 0 Idle, 1 Scanning, 2 Authenticating, 3 Success, 4 Failed, 5 Timeout |
| Registration MAC | `0005` | Write         | 6 address bytes, passed directly to the BLE address constructor              |
| Command/event    | `0006` | Write, Notify | Fixed 16-byte TLV packet                                                     |
| Orientation      | `0007` | Read, Notify  | 8 bytes: `i16` Q14 quaternion `[w,x,y,z]`, divide by 16384                   |

Subscribe to command_control **before writing commands**; enable the other CCCDs
as needed. Read/Notify properties do not promise every state transition is notified.
Pairing status reflects the NFC/storage lifecycle, not automatic success on BLE
connection. Sensor/orientation delivery
targets 20 Hz. Registration writes are asynchronous, not proof of persistence.
Address bytes are stack order, not a colon-separated string; clients must account
for their platform's address representation.

### TLV commands and responses (`0006`)

Every write/notification is 16 bytes: `[opcode:u8, len:u8, payload..., padding...]`.
`len` counts payload bytes only (maximum 14) and must exactly match a length
listed below. Truncated payloads and lengths above 14 are InvalidLength.
Send zero padding; bytes outside the declared payload are ignored, not
authenticated, and do not change the command.

| Opcode | Command            | Payload length | Payload in wire order                                   |
| ------ | ------------------ | -------------- | ------------------------------------------------------- |
| `01`   | SetLedMode         | 6              | `mode:u8, r:u8, g:u8, b:u8, duration_ms:u16`            |
| `02`   | SetBrightness      | 1              | `brightness:u8` (0–255)                                 |
| `03`   | TriggerAnimation   | 3 or 6         | `animation:u8, duration_ms:u16[, r:u8, g:u8, b:u8]`     |
| `04`   | SetThresholds      | 4              | `thrust:i16, impact:i16`; require `0 < thrust < impact` |
| `05`   | GetStatus          | 0              | Empty; requests ACK and DeviceStatus                    |
| `06`   | ClearBondedDevices | 0              | Empty; erase stored registration and reopen advertising |

Mode IDs: `0` AutoMotion, `1` ManualOverride (solid RGB).
Animation IDs: `0` Off, `1` IdleBreathing, `2` UpwardThrust,
`3` DownwardSlash, `4` ImpactHit, `5` MotionStop, `6` SolidColor.
Duration is milliseconds; zero selects the animation default/continuous behavior.
For TriggerAnimation, omitted RGB defaults to white; RGB is used only for
SolidColor and ignored for other animation IDs. Both payload forms are accepted
for every valid animation ID.

| Opcode | Event        | Payload length | Payload in wire order                                                      |
| ------ | ------------ | -------------- | -------------------------------------------------------------------------- |
| `80`   | CommandAck   | 2              | `command_opcode:u8, status:u8`                                             |
| `81`   | GestureEvent | 3              | `animation:u8, peak_accel:i16` (raw counts)                                |
| `82`   | DeviceStatus | 8              | `pairing:u8, mode:u8, animation:u8, brightness:u8, thrust:i16, impact:i16` |

ACK status codes remain `0` Ok, `1` InvalidOpcode, `2` InvalidLength,
`3` InvalidParam, `4` StorageError (`0x04`, also used for command-queue rejection;
there is no Busy code). These are distinct from pairing-status codes.
LED ACK success means **queue acceptance**, not rendering; DeviceStatus changes
only after the LED task applies the command. ClearBondedDevices ACK is sent
**after erase completes**, reporting its outcome rather than queue acceptance.
If the response queue is full, the command is logged and refused **before side
effects**; the client must time out/retry. Notifications remain best-effort:
serialize commands, and never treat a missing ACK as success. There is no
transaction ID or delivery guarantee.

Example packets (hex, each exactly 16 bytes):

```text
GetStatus:         05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
SetBrightness 128: 02 01 80 00 00 00 00 00 00 00 00 00 00 00 00 00
ACK brightness OK: 80 02 02 00 00 00 00 00 00 00 00 00 00 00 00 00
```

### NFC, registration, and access limitations

“Pairing”/“bond” names here describe an application registration record, **not
cryptographic BLE pairing or encryption**. No phone UID authentication is
implemented: NFCT acts as a tag selected by a reader and the UID is the device's
local tag identity, not the phone's UID. NFC page writes update a **RAM shadow**;
there is no real NFCT tag-memory read/write service.

The timeout/open-advertising fallback is intentional. A self-reported MAC and
local UID comparison do not authenticate a central. Whitelist entries currently
use **RandomStatic** addresses; central private-address resolution and general
phone compatibility are not supported claims. Connected clients can issue
control and ownership-changing commands. Do not rely on this design to protect
secrets or enforce ownership.

## Embedded build, memory, and SWD flashing

```sh
rustup target add thumbv7em-none-eabihf
rustup component add llvm-tools-preview
cargo build --locked --release --features embedded --target thumbv7em-none-eabihf
bash scripts/check-firmware.sh target/thumbv7em-none-eabihf/release/xiao_nrf52840_sword
```

`.cargo/config.toml` supplies Cortex-M/defmt linker flags; `memory.x` reserves:

| Region                     | Address range (end exclusive) |
| -------------------------- | ----------------------------- |
| S140 flash reservation     | `0x00000000..0x00026000`      |
| Application flash          | `0x00026000..0x000EF000`      |
| Retained registration page | `0x000EF000..0x000F0000`      |
| Application RAM            | `0x20010000..0x20040000`      |

The registration page is **not the last flash page** (that starts at `0xFF000`).
S140 has a conservative **64 KiB RAM reservation** below `0x20010000`;
its configured stack requirement is checked at boot and still needs a real
smoke test. The ELF checker verifies vectors, executable entry/content, load
segments, and reserved-region separation; it cannot prove successful boot.
Storage uses SoftDevice flash operations, not direct NVMC writes while BLE runs.
A single-record erase/write is not power-loss atomic or multi-device storage.

Flashing needs an SWD probe (for example a supported J-Link/CMSIS-DAP), board SWD
connections and reference voltage/common ground, plus `probe-rs-tools`.
Install **Nordic S140** separately: locked `nrf-softdevice-s140` bindings
(`b0ac850`) identify header version **7.0.1**, variant 140. This is v7 API evidence,
not proof that an arbitrary v7 binary is compatible; confirm the chosen release
and `0x26000` application origin. A factory USB bootloader alone is not this stack.

With the compatible S140 already present, program the application ELF:

```sh
probe-rs download --chip nRF52840_xxAA --verify target/thumbv7em-none-eabihf/release/xiao_nrf52840_sword
# Program, reset/run, and attach logging through the configured SWD runner:
cargo run --locked --release --features embedded --target thumbv7em-none-eabihf
```

For initial stack provisioning, obtain the licensed Nordic HEX, then use
`probe-rs download --chip nRF52840_xxAA --binary-format hex --verify path/to/s140.hex`.
Do not default to full-chip erase. If recovery/chip erasure is required, it
**destroys the bootloader, S140, and stored registrations/bonds**; back up what
is needed and plan restoration first. No flashing procedure here is hardware-verified.

## Hardware checks still required

- Confirm IMU identity, stationary gravity/gyro readings and configured scales.
- Measure sampling/fusion timing, including delays during flash operations.
- Exercise BLE reads, subscriptions, commands, queue saturation and reconnects.
- Check NFC reader selection and timeout/open fallback after separate boots.
- Verify registration/update/clear across reboot and injected flash failures.
- Check physical LED direction, terminal frames, MotionStop staying off,
  logic levels, full-load current and temperature with current limiting.

No simulator was added. [Wokwi's supported hardware](https://docs.wokwi.com/getting-started/supported-hardware)
does not include this nRF52840 target; a different-MCU LED demo would not validate
NFCT, S140, flash mapping or board wiring. Prefer deterministic host tests, then
the real-board checks above.
