# BNO085 Basic Read Example

Demonstrates how to initialize the BNO085 sensor and read data in your application.

## Hardware Setup

Connect the BNO085 to your ESP32-S3:
- **SDA** → GPIO 6 (or configure via menuconfig)
- **SCL** → GPIO 7 (or configure via menuconfig)
- **INT** → GPIO 5 (or configure via menuconfig)
- **RST** → GPIO 4 (or configure via menuconfig)
- **AD0** → GND (I2C address 0x4A)
- **VCC** → 3.3V
- **GND** → GND

Tie **PS0** and **PS1** to GND for I2C mode.

## Building

```bash
idf.py build
```

## Configuration

```bash
idf.py menuconfig
```

Navigate to **BNO085 Application Configuration** to:
- Change GPIO pin assignments (I2C & GPIO menu)
- Enable/disable individual sensors (Sensors menu)
- Choose output format: verbose (log) or CSV (comma-separated values)
- Adjust sampling rate

## Running

```bash
idf.py flash monitor
```

The example will initialize the sensor and print data based on your configuration:
- **Verbose mode**: Logs rotation vector, accelerometer, gyroscope, linear acceleration, and magnetometer readings
- **CSV mode**: Prints comma-separated values with a header row, one sample per line

## Example Output

**Verbose:**
```
I (1234) bno085_example: Rotation Vector: i=0.1234, j=0.5678, k=0.9012, real=0.3456, accuracy=2.5°
I (1235) bno085_example: Accel: x=9.81, y=0.02, z=-0.15 m/s²
```

**CSV:**
```
timestamp_ms,qx,qy,qz,qw,ax,ay,az,gx,gy,gz,lax,lay,laz,mx,my,mz
1234,0.1234,0.5678,0.9012,0.3456,9.81,0.02,-0.15,0.001,-0.002,0.003,0.02,-0.01,0.05,-20.1,15.3,8.7
```

## Notes

- Service the sensor at ~100 Hz (10ms intervals). The example does this in the main loop.
- CSV mode buffers sensor readings by timestamp and prints complete rows. Readings within the configured tolerance window (default 5ms) are grouped into one row.
- Only 5 sensor types have decoded output (rotation vector, accelerometer, gyroscope, linear acceleration, magnetometer). Other SH2 sensors can be enabled via raw IDs but arrive without decoding.
