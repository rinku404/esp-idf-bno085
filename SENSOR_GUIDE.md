# BNO085 Sensor Guide

This document describes all available sensors on the BNO085 and how to use them with the SH2 library.

## Quick Start

```c
// 1. Register callback to receive sensor data
bno085_register_sensor_callback(my_sensor_callback, NULL);

// 2. Enable sensors you want to use
bno085_enable_sensor(SH2_ROTATION_VECTOR, 100000);      // 100ms = 10Hz
bno085_enable_sensor(SH2_LINEAR_ACCELERATION, 100000);  // 100ms = 10Hz

// 3. In your callback, decode and use the data
void my_sensor_callback(sh2_SensorEvent_t *event) {
    sh2_SensorValue_t value;
    sh2_decodeSensorEvent(&value, event);
    // Process value based on sensorId
}

// 4. Call service regularly
while (1) {
    bno085_service();
    vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz polling
}
```

## Available Sensors

### 🎯 Orientation (Primary)

These fuse accelerometer, gyroscope, and magnetometer data to provide orientation:

#### **SH2_ROTATION_VECTOR (0x05)** ⭐ Most Accurate
- **Output**: Quaternion (i, j, k, real components)
- **Fuses**: Accel + Gyro + Magnetometer
- **Accuracy**: ±1° typical
- **Use Case**: AR/VR, drone stabilization, game controllers
- **Access**:
  ```c
  ESP_LOGI(TAG, "Quat: i=%f, j=%f, k=%f, real=%f, accuracy=%f°",
      value.un.rotationVector.i, value.un.rotationVector.j,
      value.un.rotationVector.k, value.un.rotationVector.real,
      value.un.rotationVector.accuracy * 57.2958f);
  ```

#### **SH2_GAME_ROTATION_VECTOR (0x08)**
- **Output**: Quaternion (no yaw reference)
- **Fuses**: Accel + Gyro only (NO magnetometer)
- **Accuracy**: Slightly lower in yaw (gyro drift)
- **Use Case**: Games, when magnetometer interference is an issue
- **Access**: Same as rotation vector

#### **SH2_GEOMAGNETIC_ROTATION_VECTOR (0x09)**
- **Output**: Quaternion
- **Fuses**: Accel + Magnetometer only (NO gyro)
- **Power**: Lower than rotation vector
- **Use Case**: Low-power applications, stationary devices
- **Access**: Same as rotation vector

#### **SH2_AR_VR_STABILIZED_RV (0x28)** ✨ For AR/VR
- **Output**: Quaternion with AR/VR optimizations
- **Fuses**: Accel + Gyro + Magnetometer
- **Accuracy**: ±0.5° (better than standard)
- **Use Case**: High-quality AR/VR applications

#### **SH2_GYRO_INTEGRATED_RV (0x2A)**
- **Output**: Quaternion
- **Rate**: Up to 1000 Hz (1kHz!)
- **Use Case**: Head tracking, high-frequency applications

---

### 📍 Raw IMU Data

#### **SH2_ACCELEROMETER (0x01)**
- **Output**: x, y, z in m/s² (includes gravity)
- **Range**: ±8g
- **Accuracy**: ±1% typical
- **Access**:
  ```c
  ESP_LOGI(TAG, "Accel: x=%f, y=%f, z=%f m/s²",
      value.un.acceleration.x, value.un.acceleration.y, value.un.acceleration.z);
  ```

#### **SH2_LINEAR_ACCELERATION (0x04)** 🎯
- **Output**: x, y, z in m/s² (gravity removed)
- **Use Case**: Motion detection without gravity bias
- **Access**:
  ```c
  ESP_LOGI(TAG, "Linear Accel: x=%f, y=%f, z=%f m/s²",
      value.un.linearAcceleration.x, value.un.linearAcceleration.y,
      value.un.linearAcceleration.z);
  ```

#### **SH2_GRAVITY (0x06)**
- **Output**: x, y, z in m/s² (gravity vector only)
- **Use Case**: Calculating linear acceleration manually

#### **SH2_GYROSCOPE_CALIBRATED (0x02)**
- **Output**: x, y, z in rad/s
- **Accuracy**: Calibrated with bias correction
- **Access**:
  ```c
  ESP_LOGI(TAG, "Gyro: x=%f, y=%f, z=%f rad/s",
      value.un.gyroscope.x, value.un.gyroscope.y, value.un.gyroscope.z);
  ```

#### **SH2_GYROSCOPE_UNCALIBRATED (0x07)**
- **Output**: x, y, z in rad/s + bias estimates
- **Use Case**: Advanced bias correction algorithms

#### **SH2_MAGNETIC_FIELD_CALIBRATED (0x03)**
- **Output**: x, y, z in µT (microtesla)
- **Accuracy**: ±10% typical
- **Access**:
  ```c
  ESP_LOGI(TAG, "Mag: x=%f, y=%f, z=%f µT",
      value.un.magneticField.x, value.un.magneticField.y,
      value.un.magneticField.z);
  ```

#### **SH2_MAGNETIC_FIELD_UNCALIBRATED (0x0F)**
- **Output**: x, y, z with bias estimates

#### **SH2_RAW_ACCELEROMETER (0x14)**
- **Output**: ADC raw counts
- **Use Case**: Debugging, custom calibration

#### **SH2_RAW_GYROSCOPE (0x15)**
- **Output**: ADC raw counts
- **Use Case**: Debugging, custom calibration

#### **SH2_RAW_MAGNETOMETER (0x16)**
- **Output**: ADC raw counts
- **Use Case**: Debugging, custom calibration

---

### 🎯 Motion Detection & Activity

#### **SH2_STEP_COUNTER (0x11)**
- Counts steps taken since reset
- Access: `value.un.stepCounter`

#### **SH2_STEP_DETECTOR (0x18)**
- Triggers event on each step detected

#### **SH2_SIGNIFICANT_MOTION (0x12)**
- Detects significant motion events

#### **SH2_TAP_DETECTOR (0x10)**
- Single and double tap detection

#### **SH2_SHAKE_DETECTOR (0x19)**
- Shake/vibration detection

#### **SH2_STABILITY_CLASSIFIER (0x13)**
- Reports device stability state

#### **SH2_PERSONAL_ACTIVITY_CLASSIFIER (0x1E)**
- Classifies activities: still, walking, running, on vehicle, etc.

#### **SH2_SLEEP_DETECTOR (0x1F)**
- Detects when device is in sleep state

#### **SH2_FLIP_DETECTOR (0x1A)**
- Detects device flipping motion

#### **SH2_PICKUP_DETECTOR (0x1B)**
- Detects when device is picked up

#### **SH2_TILT_DETECTOR (0x20)**
- Detects tilt changes

#### **SH2_POCKET_DETECTOR (0x21)**
- Detects if device is in a pocket

#### **SH2_CIRCLE_DETECTOR (0x22)**
- Detects circular motions

---

### 🌡️ Environmental Sensors

#### **SH2_TEMPERATURE (0x0E)**
- Ambient temperature in °C
- Access: `value.un.temperature`

#### **SH2_HUMIDITY (0x0C)**
- Relative humidity in %
- Access: `value.un.humidity`

#### **SH2_PRESSURE (0x0A)**
- Atmospheric pressure in Pa
- Access: `value.un.pressure`

#### **SH2_AMBIENT_LIGHT (0x0B)**
- Light intensity in lux
- Access: `value.un.ambientLight`

#### **SH2_PROXIMITY (0x0D)**
- Distance to object in cm
- Access: `value.un.proximity`

---

## API Reference

### Enable a Sensor
```c
esp_err_t bno085_enable_sensor(uint8_t sensor_id, uint32_t report_interval_us);
```
- `sensor_id`: One of the `SH2_*` constants above
- `report_interval_us`: Reporting interval in microseconds
  - 10000 = 10ms = 100Hz
  - 100000 = 100ms = 10Hz
  - 1000000 = 1000ms = 1Hz
  - 0 = disable

### Disable a Sensor
```c
esp_err_t bno085_disable_sensor(uint8_t sensor_id);
```

### Register Callback
```c
esp_err_t bno085_register_sensor_callback(
    void (*callback)(sh2_SensorEvent_t *event),
    void *cookie
);
```

### Decode Sensor Event
```c
int sh2_decodeSensorEvent(sh2_SensorValue_t *value, sh2_SensorEvent_t *event);
```
Returns: SH2_OK on success, SH2_ERR_* on error

---

## Example Usage Patterns

### Pattern 1: Get Orientation Only
```c
void my_callback(sh2_SensorEvent_t *event) {
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) return;
    
    if (value.sensorId == SH2_ROTATION_VECTOR) {
        printf("Quaternion: %f, %f, %f, %f\n",
            value.un.rotationVector.i,
            value.un.rotationVector.j,
            value.un.rotationVector.k,
            value.un.rotationVector.real);
    }
}

bno085_register_sensor_callback(my_callback, NULL);
bno085_enable_sensor(SH2_ROTATION_VECTOR, 100000);  // 10Hz
```

### Pattern 2: Get All IMU Data
```c
bno085_register_sensor_callback(my_callback, NULL);
bno085_enable_sensor(SH2_ACCELEROMETER, 50000);           // 20Hz
bno085_enable_sensor(SH2_GYROSCOPE_CALIBRATED, 50000);   // 20Hz
bno085_enable_sensor(SH2_MAGNETIC_FIELD_CALIBRATED, 50000);  // 20Hz
bno085_enable_sensor(SH2_LINEAR_ACCELERATION, 50000);    // 20Hz
```

### Pattern 3: Activity Detection
```c
bno085_enable_sensor(SH2_STEP_COUNTER, 500000);          // 2Hz
bno085_enable_sensor(SH2_PERSONAL_ACTIVITY_CLASSIFIER, 1000000);  // 1Hz
bno085_enable_sensor(SH2_SIGNIFICANT_MOTION, 1000000);  // Event-based
```

### Pattern 4: Motion + Orientation (Hybrid)
```c
bno085_enable_sensor(SH2_ROTATION_VECTOR, 100000);      // 10Hz orientation
bno085_enable_sensor(SH2_STEP_COUNTER, 1000000);        // 1Hz step count
bno085_enable_sensor(SH2_SIGNIFICANT_MOTION, 1000000);  // 1Hz motion
```

---

## Conversion Utilities

```c
// Radians to degrees
float deg = rad * 57.2958f;

// Quaternion to Euler angles (using sh2_decodeSensorEvent already does this)
// For manual conversion, the library provides sh2_computeEulerFromQuat()

// Temperature from sensor (already in °C)
// float temp_c = value.un.temperature;

// Humidity (already in %)
// float humidity_pct = value.un.humidity;

// Pressure to altitude (simplified)
// float altitude_m = 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 1.0f / 5.255f));
```

---

## Sensor Selection Guide

| Use Case | Recommended Sensors | Why |
|----------|---------------------|-----|
| **Game Controller** | Game Rotation Vector (10-100Hz) | No mag yaw drift, low latency |
| **AR/VR Headset** | Gyro Integrated RV (500Hz+) | High frequency, stable |
| **Drone/Robot** | Rotation Vector (50-100Hz) + Gyro | Most accurate + fast response |
| **Fitness Tracker** | Step Counter + Activity Classifier | Low power, activity detection |
| **Navigation** | Rotation Vector + Mag Field | Accurate yaw/heading |
| **Low Power Device** | Geomagnetic RV + Step Counter | Minimal power, sufficient accuracy |
| **Motion Gesture** | Accel + Gyro + Tap/Shake Detector | Full motion data + events |

---

## Notes

- All sensor values are calibrated by the BNO085 firmware
- The library handles quaternion-to-Euler conversion automatically
- Reporting intervals can be changed on-the-fly by calling `bno085_enable_sensor()` again
- Disable unused sensors to save power
- The device needs to be calibrated before high-accuracy orientation is available (auto-calibrates over time)
- Environmental sensors may require calibration for your specific environment
