# He-man Sword Sensor

> ... because I can.

## **1. Hardware: ESP32 + Accelerometer + Battery**
### **What You Need**
- **ESP32 Board**: The **Seeed Studio XIAO ESP32C3** or **XIAO ESP32S3** are excellent choices—small, powerful, and Rust-compatible.
- **Accelerometer**: Use a **3-axis accelerometer + gyroscope** (IMU) for accurate motion tracking. The **MPU6050** or **BNO055** (9-axis) are popular and easy to interface with ESP32.
- **Battery**: A **300 mAh LiPo** is fine for testing, but for longer use, consider a **500–1000 mAh** battery. Use a **TP4056** module for charging.
- **Bluetooth**: ESP32 has built-in BLE, so no extra hardware is needed.
- **Wiring**: Use thin, flexible wires to connect the IMU to the ESP32 (I2C interface).
- **Mounting**: Secure the ESP32 and IMU inside the sword handle with foam or 3D-printed brackets to prevent movement.

### **Optimal Sensor Placement**
- **Best location**: **10–15 cm from the guard (handle end)**.
  - **Why?**
    - Close to the pivot point (your wrist) for accurate rotational data.
    - Far enough from the tip to avoid excessive linear acceleration noise.
    - Minimizes the effect of the sword’s mass distribution on readings.
  - Avoid the very tip or center of mass, as this can introduce misleading centrifugal forces during swings.

## **2. Firmware: Rust on ESP32**
### **What You Need**
- **Rust Toolchain**: Install Rust and the `espup` tool for ESP32 development.
  ```bash
  cargo install espup
  espup install
  ```
- **ESP32 Rust Crates**:
  - [`esp-idf-hal`](https://github.com/esp-rs/esp-idf-hal) or [`esp-rs`](https://github.com/esp-rs) for hardware abstraction.
  - [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) for IMU communication (I2C).
  - [`esp-idf-svc`](https://github.com/esp-rs/esp-idf-svc) for BLE.
- **IMU Driver**: Use a Rust crate like [`mpu6050`](https://crates.io/crates/mpu6050) or write your own I2C driver for the BNO055.

### **Firmware Tasks**
1. **Read IMU Data**: Sample accelerometer and gyroscope data at **100–200 Hz** (higher = smoother, but more power).
2. **Preprocess Data**: Apply a **low-pass filter** to reduce noise (e.g., moving average or exponential filter).
3. **Send Data via BLE**: Pack the data into a binary format (e.g., `f32` for each axis) and stream it to the phone.
   - Use **BLE notifications** for real-time updates.
   - Example BLE service UUID: `0000180f-0000-1000-8000-00805f9b34fb` (Battery Service can be repurposed).

### **Example Rust Workflow**
```rust
// Pseudocode
loop {
    let accel = imu.read_accelerometer();
    let gyro = imu.read_gyroscope();
    let timestamp = system_timer.now();
    ble.notify(&[accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, timestamp]);
    delay(5ms); // ~200 Hz
}
```


## **3. Mobile App: Flutter (Dart)**
### **What You Need**
- **Flutter Packages**:
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
   - Double-integrate acceleration to get **position** (drift is a challenge—see below).
   - Use **gyroscope data to rotate the acceleration vector** into a global frame.
5. **Power Estimation**:
   - **Power = Force × Velocity**.
   - Approximate force as `mass × acceleration` (use sword mass, e.g., 1 kg).
   - Velocity is the integral of acceleration (correct for drift with zero-velocity updates when the sword is stationary).
6. **3D Visualization**:
   - Plot the sword’s trajectory in 3D space using the calculated positions.
   - Use a **3D scatter plot** or a **line render** (e.g., with `flutter_3d_obj`).
   - Color-code segments by power or speed.


## **4. Algorithms for the Flutter App**
### **Key Algorithms**
| Task                | Algorithm                          | Notes                                                                 |
|---------------------|------------------------------------|-----------------------------------------------------------------------|
| Sensor Fusion       | Madgwick/Mahony AHRS               | Combines accel + gyro to estimate orientation (quaternions).          |
| Drift Correction    | Zero-Velocity Update (ZUPT)        | Assume sword is stationary at rest to correct integration drift.      |
| Trajectory          | Double Integration                 | Integrate accel → velocity → position. Use ZUPT to reset velocity.   |
| Power Calculation   | `Power = mass × accel × velocity`  | Simplify: `Power ≈ mass × |accel| × |velocity|`.                        |
| Smoothing           | Low-Pass Filter (Butterworth)      | Reduce high-frequency noise in accel/gyro data.                     |

### **Drift Handling**
- **Problem**: Double-integrating acceleration leads to **massive drift** over time.
- **Solutions**:
  1. **Zero-Velocity Updates (ZUPT)**: When the sword is stationary (detected via accel/gyro noise thresholds), reset velocity to 0.
  2. **Complementary Filter**: Blend high-frequency gyro data with low-frequency accel data.
  3. **Assumptions**: Assume the sword starts at rest and returns to rest between swings.

## **5. How to Start**
### **Step-by-Step Plan**
1. **Hardware Setup**:
   - Solder the IMU to the ESP32 (I2C: SDA, SCL, VCC, GND).
   - Test IMU readings with Rust (print raw accel/gyro values to serial).
2. **BLE Communication**:
   - Write Rust code to send IMU data over BLE.
   - Write a Flutter app to receive and log the data.
3. **Sensor Fusion**:
   - Implement Madgwick filter in Dart (or use a library).
   - Visualize orientation (e.g., 3D cube) in the app.
4. **Trajectory & Power**:
   - Add double integration and ZUPT.
   - Plot 3D trajectory and power estimates.
5. **Optimize**:
   - Calibrate IMU (remove offsets).
   - Tune filter parameters (e.g., Madgwick beta).
   - Reduce BLE latency (increase MTU size).


## **6. Example Resources**
- **Rust + ESP32**:
  - [ESP-RS Book](https://esp-rs.github.io/book/)
  - [MPU6050 Rust Driver](https://github.com/eldruin/mpu6050-rs)
- **Flutter + BLE**:
  - [flutter_blue example](https://github.com/Freeyourgadget/Gadgetbridge/tree/master/app/src/main/java/nodomain/freeyourgadget/gadgetbridge/service/ble)
- **Sensor Fusion**:
  - [Madgwick AHRS (Python reference)](https://github.com/arduino-libraries/MadgwickAHRS)
  - [Dart implementation](https://github.com/lettier/3d-game-shaders-for-beginners/blob/master/assets/3d-game-shaders-for-beginners.md#quaternions)


## **7. Challenges & Tips**
- **Power Consumption**:
  - Use **deep sleep** for the ESP32 when idle.
  - Reduce BLE advertising interval.
- **Drift**: Expect **~1–2 meters of drift per minute** without correction. ZUPT is critical.
- **Latency**: BLE notifications add ~10–50 ms latency. Use **BLE 5.0** for lower latency.
- **Calibration**: Calibrate the IMU (remove biases) before each use.

## License

MIT
