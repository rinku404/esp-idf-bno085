# BNO085 IMU Driver for ESP-IDF

ESP-IDF component for the Bosch BNO085 9-axis IMU (accelerometer, gyroscope, magnetometer) via CEVA's SH2 sensor hub protocol over I2C.

## Features

- 9-DOF sensor fusion with on-chip calibration
- Rotation vectors (quaternions), linear acceleration, calibrated gyroscope, and magnetic field output
- Handle-based API with opaque types — no SH2 internals exposed
- Callback-based data delivery when sensor reports are ready
- Configurable via ESP-IDF's `menuconfig`

## Hardware Setup

### Wiring (Heltec LoRa V3 example)

| BNO085 Pin | ESP32-S3 Pin | Signal       | Notes           |
|------------|--------------|--------------|-----------------|
| SDA        | GPIO 6       | I2C SDA      | Pull-up enabled |
| SCL        | GPIO 7       | I2C SCL      | Pull-up enabled |
| INT (H_INTN)| GPIO 5      | Interrupt    | Active-low      |
| RST (NRST) | GPIO 4       | Reset        | Active-low      |
| AD0        | GND          | Slave addr   | I2C addr 0x4A   |
| VCC        | 3.3V         | Power        |                 |
| GND        | GND          | Ground       |                 |

**Critical**: PS0 and PS1 pins must be tied to GND for I2C mode operation.

## Installation

### From ESP-IDF Component Registry

```bash
idf.py add-dependency bno085
```

### From Git Submodule (Development)

```bash
git submodule add https://github.com/<your-username>/bno085_aula8.git components/bno085
git submodule update --init --recursive
```

## Quick Start

```c
#include "bno085.h"
#include "driver/i2c_master.h"
#include "freertos/task.h"

static void sensor_callback(bno085_handle_t handle,
                           const bno085_sensor_value_t *value,
                           void *user_context)
{
    if (value->sensor_id == BNO085_SENSOR_ROTATION_VECTOR) {
        printf("Quaternion: i=%.4f, j=%.4f, k=%.4f, real=%.4f\n",
               value->data.rotation_vector.i,
               value->data.rotation_vector.j,
               value->data.rotation_vector.k,
               value->data.rotation_vector.real);
    }
}

void app_main(void)
{
    // Set up I2C bus (existing code)
    i2c_master_bus_config_t bus_config = { /* ... */ };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&bus_config, &bus_handle);

    i2c_device_config_t dev_config = { /* ... */ };
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);

    // Initialize BNO085 with default config
    bno085_config_t config;
    bno085_config_default(&config);

    bno085_handle_t bno085;
    bno085_init(&config, dev_handle, GPIO_INT_PIN, GPIO_RST_PIN, &bno085);

    // Register callback and enable sensor
    bno085_register_sensor_callback(bno085, sensor_callback, NULL);
    bno085_enable_sensor(bno085, BNO085_SENSOR_ROTATION_VECTOR, 100000);  // 10Hz

    // Service in main loop (10ms recommended)
    while (1) {
        bno085_service(bno085);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

## API Reference

### Initialization

```c
esp_err_t bno085_init(const bno085_config_t *config,
                      i2c_master_dev_handle_t i2c_dev,
                      gpio_num_t int_pin,
                      gpio_num_t reset_pin,
                      bno085_handle_t *out_handle);
```

Initialize the BNO085 with I2C device and GPIO pins. Only one instance can be active at a time (due to underlying SH2 library singleton design).

### Service Loop

```c
void bno085_service(bno085_handle_t handle);
```

Poll the device for pending sensor reports. Call periodically (10ms recommended) from the main loop.

### Cleanup

```c
void bno085_deinit(bno085_handle_t handle);
```

Close the device and free resources.

### Callbacks

```c
typedef void (*bno085_sensor_callback_t)(bno085_handle_t handle,
                                          const bno085_sensor_value_t *value,
                                          void *user_context);

esp_err_t bno085_register_sensor_callback(bno085_handle_t handle,
                                          bno085_sensor_callback_t callback,
                                          void *user_context);
```

Register a function to be called whenever sensor data is available. Replaces any previous callback.

### Sensor Control

```c
esp_err_t bno085_enable_sensor(bno085_handle_t handle,
                               uint8_t sensor_id,
                               uint32_t report_interval_us);

esp_err_t bno085_disable_sensor(bno085_handle_t handle,
                                uint8_t sensor_id);
```

Enable/disable individual sensors with specified reporting interval.

## Supported Sensors

The driver decodes the following five sensor types with named fields:

| Sensor ID | Constant | Output Type | Use Case |
|-----------|----------|-------------|----------|
| 0x01      | `BNO085_SENSOR_ACCELEROMETER` | `data.accelerometer` | Motion detection with gravity |
| 0x02      | `BNO085_SENSOR_GYROSCOPE_CALIBRATED` | `data.gyroscope` | Angular velocity (rad/s) |
| 0x03      | `BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED` | `data.magnetic_field` | Magnetometer (µTesla) |
| 0x04      | `BNO085_SENSOR_LINEAR_ACCELERATION` | `data.linear_acceleration` | Motion without gravity |
| 0x05      | `BNO085_SENSOR_ROTATION_VECTOR` | `data.rotation_vector` | Orientation as quaternion |

Other SH2 sensors (step counter, activity classifier, etc.) can be enabled via raw sensor IDs, but data arrives undecoded (only `sensor_id`, `status`, `timestamp_us` populated).

See `SENSOR_GUIDE.md` in this directory for the complete list of all 40+ available sensors and example usage patterns.

## Configuration

```c
typedef struct {
    uint32_t i2c_timeout_ms;        // I2C operation timeout (default: 100ms)
    uint8_t  reset_retry_count;     // Soft-reset retries (default: 5)
    uint32_t reset_retry_delay_ms;  // Delay between retries (default: 30ms)
    uint32_t reset_settle_delay_ms; // Delay after reset (default: 300ms)
} bno085_config_t;

void bno085_config_default(bno085_config_t *config);
```

Use `bno085_config_default()` to get recommended values, then customize as needed.

## Limitations

- **Single Instance Only**: The underlying SH2 library uses a global state (`sh2_t _sh2`), so only one BNO085 can be initialized at a time. Calling `bno085_init()` while another instance is active returns `ESP_ERR_INVALID_STATE`.
- **Decoded Sensors**: Only 5 sensor types (listed above) have built-in decoding to named fields. Other sensors can be enabled but arrive raw (metadata only).

## Example Projects

See `examples/basic_read/` for a self-contained example that initializes the device and prints rotation vectors.

## SH2 Library (CEVA Sensor Hub 2)

This component includes the **SH2 (Sensor Hub 2) library** v1.4.0, developed by CEVA-DSP, which implements the low-level communication protocol between the ESP32-S3 and the BNO085 sensor. The SH2 library handles SHTP (Sensor Hub Transport Protocol) over I2C and raw sensor data decoding.

### Vendored Inclusion

The SH2 library is **vendored directly** inside this component at `sh2/`, rather than as an external dependency. This means:

- **No extra steps**: When you install the `bno085` component via `idf.py add-dependency bno085`, the SH2 library is included automatically. No git submodule initialization required.
- **Self-contained**: Everything needed to communicate with the BNO085 ships together with the component.
- **Version pinned**: The included SH2 is pinned to v1.4.0 (CEVA-DSP tag from GitHub), ensuring reproducible builds.

### Attribution

The SH2 library is published under the Apache-2.0 license by CEVA-DSP and is included with the same license terms as this driver. See the individual source files inside `sh2/` for per-file attributions and NOTICE.txt.

### Reference

For low-level SH2 protocol details, refer to `sh2/README.md` in this directory or the original repository at [github.com/ceva-dsp/sh2](https://github.com/ceva-dsp/sh2).

## License

This component is licensed under Apache-2.0. It incorporates the CEVA SH2 library, also Apache-2.0. See `LICENSE` in this directory for full terms.

## Support

- GitHub Issues: Report bugs or request features
- Datasheet: BNO085 datasheet available from Bosch Sensortec
- SH2 Reference: See `sh2/README.md` for protocol details
