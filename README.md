# BNO085 ESP32-S3 Sensor Application

An ESP-IDF application for the BNO085 9-axis motion sensor (IMU) on the ESP32-S3 microcontroller. Reads orientation, acceleration, gyroscope, magnetometer, and motion detection data, outputs configurable CSV format to serial.

## What This Project Does

**Hardware:** BNO085 IMU connected to ESP32-S3 via I2C with interrupt-driven data readiness signaling.

**Functionality:**
- Reads 40+ sensor types from the BNO085 (accelerometer, gyroscope, magnetometer, rotation vectors, step counting, activity classification, etc.)
- Configurable **per-sensor enable/disable** for reading
- Configurable **per-sensor print filtering** (read sensors but only output selected ones)
- **Global sensor update rate** (all sensors poll at the same frequency)
- **CSV output** with configurable formatting (separator, header, timestamps)
- **Rotation vector output** in Quaternion or Euler angle format
- All timing and sensor selection configured via **menuconfig** (no code changes needed)

**Default Configuration:**
- AR/VR Stabilized Rotation Vector in Euler angles (roll, pitch, yaw)
- Comma-space separator (`, `)
- No timestamp column
- 10ms sensor poll rate, 100ms output rate

## Quick Start

### 1. Prerequisites

**Local Setup:**
```bash
export IDF_PATH=~/esp/v5.5.5/esp-idf
source $IDF_PATH/export.sh
```

### 2. Build & Flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

**Serial Output:**
```
arvr_roll, arvr_pitch, arvr_yaw, arvr_acc
-5.99, -0.94, 4.29, 180.0
-5.99, -0.94, 4.29, 180.0
```

## Configuration (menuconfig)

Run `idf.py menuconfig` → **BNO085 Application Configuration**

### Sensors to Enable (Read from Device)

Select which sensors to read from BNO085. All enabled sensors report at the **global Sensor Update Period** (10ms default).

**Orientation Sensors:**
- Rotation Vector (auto-enables accel, gyro, mag)
- Game Rotation Vector (accel+gyro)
- Geomagnetic RV (accel+mag)
- AR/VR Stabilized RV (high-precision, default)
- Gyro Integrated RV (1kHz)

**Raw IMU Data:**
- Accelerometer, Linear Acceleration, Gravity
- Gyroscope (calibrated & uncalibrated)
- Magnetic Field (calibrated & uncalibrated)
- Raw ADC counts

**Environmental & Activity:**
- Pressure, Humidity, Temperature, Ambient Light
- Step Counter, Step Detector, Stability Classifier
- Personal Activity, Significant Motion, Shake Detector
- Tap, Flip, Pickup, Tilt, Pocket, Circle, Sleep Detectors

**Important:** Enabling a rotation vector auto-enables its required component sensors. Example: enabling "Rotation Vector" auto-enables Accelerometer, Gyroscope, and Magnetometer.

### Sensors to Print (CSV Output)

**Independent from "Sensors to Enable"** — Select which sensors to output to CSV.

Example: Enable Accelerometer (read it), but don't print it (filter from output).

**Rotation Vector Output Format** (shared for all RV types):
- **Quaternion:** i, j, k, real, accuracy
- **Euler Angles:** roll, pitch, yaw (degrees), accuracy

### Output Format

| Setting | Default | Notes |
|---------|---------|-------|
| CSV Output Enabled | ON | Enable/disable CSV |
| Output Period (ms) | 100 | How often data prints (10-10,000) |
| Sensor Update Period (ms) | 10 | How often sensor polled (5-1,000) |
| Rotation Vector Format | Euler | Applies to ALL rotation vectors |
| Timestamp Column | None | Disabled or Sensor Only |
| Print CSV Header | ON | Print column names |
| CSV Separator | Comma | Comma/Semicolon/Pipe/Tab |
| Add Space After Separator | ON | `,` vs `, ` |

**Key Concept:** 
- **Sensor Update Period** = Polling loop frequency (all sensors read at same rate)
- **Output Period** = How often CSV is printed
- **No per-sensor rates** (architectural decision: single device, global rate)

---

## Architecture

```
main/
  ├── main.c         — Initialization (~160 lines)
  ├── sensor.c/h     — Data management, ISR, polling task
  ├── output.c/h     — CSV formatting & output task
  └── Kconfig.projbuild — All configuration
```

**Design:**
- Modular: sensor / output / main separation
- Global rate: All sensors poll at same frequency
- Callback-driven: ISR → sensor callback → polling loop
- Thread-safe: Per-sensor mutexes, safe data copying

---

## Common Configurations

### Basic 9-Axis

Enable: Accel, Gyro, Magnetic Field
Print: Accel, Gyro, Magnetic Field
Format: Comma with space, no timestamp

**Output:** `ac_x, ac_y, ac_z, gy_x, gy_y, gy_z, mg_x, mg_y, mg_z`

### Full 9-Axis Rotation + Raw

Enable: Rotation Vector (auto-enables accel/gyro/mag), Accel, Gyro, Mag
Print: Rotation Vector, Accel, Gyro, Mag
Format: Euler Angles, comma with space, no timestamp

**Output:** `rv_roll, rv_pitch, rv_yaw, rv_acc, ac_x, ac_y, ac_z, ...`

### Motion Detection

Enable: Step Counter, Step Detector, Shake Detector, Tap Detector
Print: Same
Format: 500ms output period (less frequent for events)

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No data | Check "Sensors to Enable" + "Sensors to Print", verify baud rate (115200) |
| Header ≠ Data | Rebuild with `idf.py fullclean && idf.py build` |
| Accuracy ~180° | Normal during calibration. Wave sensor figure-8, rotate in all axes |
| Timestamp = 0 | Ensure "Timestamp Column" ≠ "None" |
| Wrong RV format | Check "Rotation Vector Output Format" (applies globally) |

---

## Build Commands

```bash
idf.py set-target esp32s3      # Set chip
idf.py menuconfig               # Configure
idf.py build                    # Build (incremental)
idf.py fullclean                # Clean & rebuild
idf.py flash                    # Flash
idf.py monitor                  # Serial monitor
idf.py build flash monitor      # Combined
```

---

## Hardware Wiring

| BNO085 Pin | ESP32-S3 | Signal |
|-----------|----------|--------|
| SDA | GPIO 6 | I2C Data |
| SCL | GPIO 7 | I2C Clock |
| INT | GPIO 5 | Data-Ready (active low) |
| RST | GPIO 4 | Reset (active low) |
| VCC | 3.3V | Power |
| GND | GND | Ground |

**AD0:** GND → 0x4A, VCC → 0x4B

---

## Calibration

BNO085 auto-calibrates:
- **Accel:** During varied motion
- **Gyro:** Slow rotation in all axes
- **Mag:** Figure-8 movement (needs Earth's field)

Accuracy improves after ~1 minute of motion.

---

## Timing Strategy

**Why separate polling & output rates?**
- Sensor Update Period (10ms) = Fast enough for events
- Output Period (100ms) = Reduces serial traffic

**Adjusting:**
- Real-time? → Lower Output Period (50ms)
- Low power? → Raise both periods
- Event detection? → Keep Sensor Update Period low

---

## Known Limitations

- Single BNO085 instance only
- No per-sensor rates (all use global rate)
- I2C only (no SPI)
- ESP32-S3 only (others untested)

---

## See Also

- `components/bno085/SENSOR_GUIDE.md` — All 40+ sensor types
- BNO085 datasheet — SH2 protocol reference
- ESP-IDF docs — Menuconfig & configuration

