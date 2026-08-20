# BNO085 Sensor Configuration Guide

## Overview

This document explains the cleaned-up sensor configuration system for the BNO085 driver. The system supports all 28 sensors that the BNO085 hardware provides, with proper dependency management for sensors that require others for calculation.

## Sensor Categories

The sensors are organized into logical menus in `idf.py menuconfig`:

### 1. Orientation Sensors (5 types)
- **Rotation Vector** (0x05) - Full 9-axis (accel+gyro+mag)
  - Output format selectable: **Quaternion** (i, j, k, real) or **Euler Angles** (roll, pitch, yaw in degrees)
- **Game Rotation Vector** (0x08) - 6-axis (accel+gyro, no mag)
- **Geomagnetic Rotation Vector** (0x09) - 6-axis (accel+mag, no gyro)
- **AR/VR Stabilized RV** (0x28) - High-precision 9-axis
- **Gyro Integrated RV** (0x2A) - High-frequency 1kHz from gyro

### 2. Raw IMU Data (9 types)
- **Accelerometer** (0x01) - Calibrated, includes gravity
- **Linear Acceleration** (0x04) - Gravity removed
- **Gravity** (0x06) - Extracted gravity vector
- **Gyroscope** (0x02) - Calibrated angular velocity
- **Gyroscope Uncalibrated** (0x07) - Raw + bias estimates
- **Magnetic Field** (0x03) - Calibrated 3-axis magnetometer
- **Magnetic Field Uncalibrated** (0x0F) - Raw + bias estimates
- **Raw Accelerometer** (0x14) - ADC counts
- **Raw Gyroscope** (0x15) - ADC counts
- **Raw Magnetometer** (0x16) - ADC counts

### 3. Environmental Sensors (4 types)
- **Pressure** (0x0A) - Barometric sensor (if equipped)
- **Humidity** (0x0C) - Relative humidity (if equipped)
- **Temperature** (0x0E) - Internal temperature (if equipped)
- **Ambient Light** (0x0B) - Light sensor (if equipped)

### 4. Activity & Motion Detection (10 types)
- **Step Counter** (0x11) - Cumulative step count
- **Step Detector** (0x18) - Event on each step
- **Stability Classifier** (0x13) - Device stability state
- **Personal Activity Classifier** (0x1E) - Walking, running, cycling, etc.
- **Significant Motion** (0x12) - Motion detection event
- **Shake Detector** (0x19) - High-frequency shaking
- **Tap Detector** (0x10) - Single/double tap with direction
- **Flip Detector** (0x1A) - Device flip event
- **Pickup Detector** (0x1B) - Pickup from surface event
- **Tilt Detector** (0x20) - Tilt angle change
- **Pocket Detector** (0x21) - Device in pocket
- **Circle Detector** (0x22) - Circular motion
- **Sleep Detector** (0x1F) - Sleep state detection

## Dependency Management

### Problem Solved
Rotation vectors require base sensor data to function internally:
- Rotation Vector needs: Accel + Gyro + Magnetometer
- Game Rotation Vector needs: Accel + Gyro
- Geomagnetic RV needs: Accel + Magnetometer
- AR/VR Stabilized RV needs: Accel + Gyro + Magnetometer
- Gyro Integrated RV needs: Gyroscope

Previously, if you wanted a rotation vector but didn't explicitly enable the base sensors, they wouldn't be read at all, and the rotation vector would have garbage data.

### Solution: Silent Dependencies
When you enable a rotation vector sensor, the system automatically:
1. **Marks internal flags** (via Kconfig `select` directives):
   - `APP_BNO085_ACCEL_CALC_NEEDED`
   - `APP_BNO085_GYRO_CALC_NEEDED`
   - `APP_BNO085_MAG_CALC_NEEDED`

2. **At runtime**, the application checks these flags and silently enables the required sensors **only if they're not already enabled for printing**.

3. **Result**: The base sensors are always available when needed, but only print data if explicitly selected in the config.

### How It Works in Code

**main/Kconfig.projbuild:**
```kconfig
config APP_BNO085_ENABLE_ROTATION_VECTOR
    bool "Rotation Vector (Quaternion, accel+gyro+mag)"
    default y
    select APP_BNO085_ACCEL_CALC_NEEDED    # Auto-set when enabled
    select APP_BNO085_GYRO_CALC_NEEDED
    select APP_BNO085_MAG_CALC_NEEDED
```

**main/main.c:**
```c
/* Enable all sensors explicitly selected by user (for printing) */
#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_ACCELEROMETER, ...);
#endif

/* ... other explicit sensors ... */

/* Enable sensors needed for calculations but not printing */
#if defined(CONFIG_APP_BNO085_ACCEL_CALC_NEEDED) && !defined(CONFIG_APP_BNO085_ENABLE_ACCELEROMETER)
    bno085_enable_sensor(bno085_handle, BNO085_SENSOR_ACCELEROMETER, ...);
#endif
```

## Example Configurations

### Example 1: Just Rotation Vector (default)
```
✓ Rotation Vector (printing)
✓ Accelerometer (silent, not printing)
✓ Gyroscope (silent, not printing)
✓ Magnetometer (silent, not printing)
```
Output: Only rotation vector data printed.

### Example 2: Rotation Vector + All IMU Data
```
✓ Rotation Vector (printing)
✓ Accelerometer (printing)
✓ Linear Acceleration (printing)
✓ Gyroscope (printing)
✓ Magnetometer (printing)
```
Output: All five sensor streams printed. No silent enables needed (explicitly selected).

### Example 3: Game Rotation Vector Only
```
✓ Game Rotation Vector (printing)
✓ Accelerometer (silent, not printing)
✓ Gyroscope (silent, not printing)
```
Output: Only game rotation vector data printed. Accel/gyro enabled internally but not printed.

### Example 4: Activity Detection Only
```
✓ Step Counter (printing)
✓ Stability Classifier (printing)
✓ Personal Activity Classifier (printing)
```
No rotation vector selected, so no accel/gyro/mag needed for printing. Only these sensors are enabled.

## Configuration Steps

1. **Run menuconfig:**
   ```bash
   idf.py menuconfig
   ```

2. **Navigate to:**
   ```
   BNO085 Application Configuration
   └── Sensors to Enable & Print
   └── Orientation Sensors
   ```

3. **Select any rotation vector sensors you want** and **choose output format**:
   - Enable any/all of: Rotation Vector, Game RV, Geomagnetic RV, AR/VR RV, Gyro Integrated RV
   - Then select the **Rotation Vector Output Format** choice at the bottom of "Orientation Sensors":
     - **Quaternion (default)**: Best for 3D graphics, game engines, robotics
     - **Euler Angles**: Best for intuitive visualization, human understanding
     
4. **Select other desired sensors:**
   - Enable sensors you want to **see printed in the output**
   - Dependencies are handled automatically

5. **Optional: View dependencies**
   ```
   BNO085 Application Configuration
   └── Dependency Resolution (Internal - Auto-Enabled)
   ```
   (This menu is hidden from normal editing with `visible if 0`)

6. **Build and flash:**
   ```bash
   idf.py build flash monitor
   ```

## Rotation Vector Output Formats

The **Rotation Vector Output Format** choice applies globally to **all 5 rotation vector sensors**:
- Rotation Vector (full 9-axis)
- Game Rotation Vector (accel+gyro)
- Geomagnetic Rotation Vector (accel+mag)
- AR/VR Stabilized RV (high-precision)
- Gyro Integrated RV (1kHz high-frequency)

### Quaternion Format (Default)
```
Rotation Vector: i=0.123456, j=-0.234567, k=0.345678, real=0.900000, accuracy=0.0°
Game Rotation Vector: i=0.110000, j=-0.220000, k=0.330000, real=0.910000, accuracy=0.0°
CSV: timestamp_ms,rv_i,rv_j,rv_k,rv_real,rv_acc,grv_i,grv_j,grv_k,grv_real,grv_acc
     1000,0.123456,-0.234567,0.345678,0.900000,0.0,0.110000,-0.220000,0.330000,0.910000,0.0
```

**Quaternion Representation:**
- Unit quaternion: `q = i*x̂ + j*ŷ + k*ẑ + real`
- Represents 3D rotation as `q = [i, j, k, real]`
- More compact than Euler angles (4 values vs 3)
- No singularities (gimbal lock)
- Better for interpolation and composition
- Standard in robotics and game engines

### Euler Angles Format
```
Rotation (Euler): roll=7.1°, pitch=-13.5°, yaw=42.3°, accuracy=0.0°
Game Rotation (Euler): roll=6.3°, pitch=-12.1°, yaw=41.5°, accuracy=0.0°
CSV: timestamp_ms,rv_roll,rv_pitch,rv_yaw,rv_acc,grv_roll,grv_pitch,grv_yaw,grv_acc
     1000,7.13,-13.47,42.31,0.0,6.31,-12.09,41.52,0.0
```

**Euler Angles:**
- Roll (X-axis rotation): -180° to +180°
- Pitch (Y-axis rotation): -90° to +90°
- Yaw (Z-axis rotation): -180° to +180°
- Intuitive for human understanding
- Better for visualization and debugging
- Suffers from gimbal lock at ±90° pitch (mathematical singularity)

**Conversion Note:** Euler angles are computed from quaternion at runtime, so both representations use the same underlying sensor data.

## Output Behavior

- **CSV Output**: Only columns for explicitly enabled sensors appear
- **Verbose Output**: Only enabled sensors' logs are printed
- **Silent Dependencies**: Never print data, only ensure calculations work correctly
- **Rotation Vector Format**: Selected via `Rotation Vector Output Format` menu choice

## Notes

- **All 28 supported sensors** are exposed in the configuration
- **No cleanup needed**: Kconfig matches all sensors in bno085.h exactly
- **Sampling rate** is global for all enabled sensors (1-1000 Hz)
- **Output format** can be Verbose (human-readable) or CSV
- Environmental and detector sensors may not be available on all BNO085 variants

## See Also

- `bno085.h` - Complete sensor ID definitions
- `main/main.c` - Callback and sensor printing logic
- `components/bno085/SENSOR_GUIDE.md` - Detailed sensor descriptions
