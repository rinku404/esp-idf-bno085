# BNO085 IMU Driver for ESP-IDF

ESP-IDF component for the Bosch BNO085 9-axis IMU over I2C using CEVA's SH2 sensor hub protocol.

## Features

- 35+ sensors (orientation, raw IMU, environmental, activity detection, gesture detection)
- Multiple rotation vector types (standard, game, geomagnetic, AR/VR optimized, 1kHz gyro-integrated)
- Up to 8 concurrent sensors on the same I2C bus (configurable)
- Flexible I2C addressing (0x28 or 0x29 via AD0 pin)
- Multiplexer support for 8+ sensors
- Handle-based API with callback-based data delivery
- Full sensor decoding into named fields

## Hardware Setup

### Wiring Example (Heltec LoRa V3)

| BNO085 Pin | ESP32-S3 Pin | Signal       | Notes           |
|------------|--------------|--------------|-----------------|
| SDA        | GPIO 6       | I2C SDA      | Pull-up enabled |
| SCL        | GPIO 7       | I2C SCL      | Pull-up enabled |
| INT (H_INTN)| GPIO 5      | Interrupt    | Active-low      |
| RST (NRST) | GPIO 4       | Reset        | Active-low      |
| AD0        | GND          | Slave addr   | I2C addr 0x28   |
| VCC        | 3.3V         | Power        |                 |
| GND        | GND          | Ground       |                 |

**Important**: PS0 and PS1 must be tied to GND for I2C mode.

### I2C Address Selection

The BNO085 has an AD0 pin that selects the I2C address:
- AD0 = GND (0V) → I2C address 0x28
- AD0 = 3.3V → I2C address 0x29

Create the I2C device handle with the correct address matching your AD0 pin configuration.

### Multiple Sensors

Use an I2C multiplexer (e.g., TCA9548A) to connect multiple BNO085 sensors on different channels. Initialize one handle per sensor and select the multiplexer channel before calling `bno085_service()` on each handle.

## Installation

### From ESP-IDF Component Registry

```bash
idf.py add-dependency bno085
```

### From Git (Development)

```bash
git clone --recursive https://github.com/<your-username>/bno085.git components/bno085
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
    // Set up I2C
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_6,
        .scl_io_num = GPIO_NUM_7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&bus_config, &bus_handle);

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x28,  // AD0 = GND
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t i2c_dev;
    i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev);

    // Initialize BNO085
    bno085_config_t config;
    bno085_config_default(&config);

    bno085_handle_t bno085;
    bno085_init(&config, i2c_dev, GPIO_NUM_5, GPIO_NUM_4, &bno085);

    // Register callback
    bno085_register_sensor_callback(bno085, sensor_callback, NULL);

    // Enable sensors
    bno085_enable_sensor(bno085, BNO085_SENSOR_ROTATION_VECTOR, 100000);  // 10Hz
    bno085_enable_sensor(bno085, BNO085_SENSOR_ACCELEROMETER, 100000);    // 10Hz

    // Service loop
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

Initialize the BNO085. Only one instance per chip due to SH2 library limitations.

### Service Loop

```c
void bno085_service(bno085_handle_t handle);
```

Call periodically (10ms recommended) to read sensor data and dispatch callbacks.

### Cleanup

```c
void bno085_deinit(bno085_handle_t handle);
```

Close the device and free resources.

### Callbacks

```c
esp_err_t bno085_register_sensor_callback(bno085_handle_t handle,
                                          bno085_sensor_callback_t callback,
                                          void *user_context);
```

Register a callback function. Replaces any previous callback.

### Sensor Control

```c
esp_err_t bno085_enable_sensor(bno085_handle_t handle,
                               uint8_t sensor_id,
                               uint32_t report_interval_us);

esp_err_t bno085_disable_sensor(bno085_handle_t handle,
                                uint8_t sensor_id);
```

Enable/disable sensors with specified reporting interval in microseconds:
- 10000 = 10ms (100Hz)
- 100000 = 100ms (10Hz)
- 1000000 = 1000ms (1Hz)

### Configuration

```c
typedef struct {
    uint32_t i2c_timeout_ms;        // I2C operation timeout (default: 100ms)
    uint8_t  reset_retry_count;     // Soft-reset retries (default: 5)
    uint32_t reset_retry_delay_ms;  // Delay between retries (default: 30ms)
    uint32_t reset_settle_delay_ms; // Delay after reset (default: 300ms)
} bno085_config_t;

void bno085_config_default(bno085_config_t *config);
```

## Available Sensors

### Orientation Sensors

| Sensor ID | Constant | Output | Use Case |
|-----------|----------|--------|----------|
| 0x05 | `BNO085_SENSOR_ROTATION_VECTOR` | Quaternion | Most accurate, AR/VR, drones |
| 0x08 | `BNO085_SENSOR_GAME_ROTATION_VECTOR` | Quaternion | Games (no mag yaw drift) |
| 0x09 | `BNO085_SENSOR_GEOMAGNETIC_ROTATION_VECTOR` | Quaternion | Low power, stationary devices |
| 0x28 | `BNO085_SENSOR_AR_VR_STABILIZED_RV` | Quaternion | High-quality AR/VR |
| 0x2A | `BNO085_SENSOR_GYRO_INTEGRATED_RV` | Quaternion | Head tracking (1kHz) |

### Raw IMU Data

| Sensor ID | Constant | Output | Range |
|-----------|----------|--------|-------|
| 0x01 | `BNO085_SENSOR_ACCELEROMETER` | x, y, z (m/s²) | ±8g |
| 0x02 | `BNO085_SENSOR_GYROSCOPE_CALIBRATED` | x, y, z (rad/s) | Calibrated |
| 0x03 | `BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED` | x, y, z (µT) | Calibrated |
| 0x04 | `BNO085_SENSOR_LINEAR_ACCELERATION` | x, y, z (m/s²) | Gravity removed |
| 0x06 | `BNO085_SENSOR_GRAVITY` | x, y, z (m/s²) | Gravity vector only |
| 0x07 | `BNO085_SENSOR_GYROSCOPE_UNCALIBRATED` | x, y, z + bias | Raw with bias |
| 0x0F | `BNO085_SENSOR_MAGNETIC_FIELD_UNCALIBRATED` | x, y, z + bias | Raw with bias |
| 0x14 | `BNO085_SENSOR_RAW_ACCELEROMETER` | x, y, z | ADC counts |
| 0x15 | `BNO085_SENSOR_RAW_GYROSCOPE` | x, y, z | ADC counts |
| 0x16 | `BNO085_SENSOR_RAW_MAGNETOMETER` | x, y, z | ADC counts |

### Motion Detection & Activity

| Sensor ID | Constant | Description |
|-----------|----------|-------------|
| 0x10 | `BNO085_SENSOR_TAP_DETECTOR` | Single/double tap detection |
| 0x11 | `BNO085_SENSOR_STEP_COUNTER` | Step count since reset |
| 0x12 | `BNO085_SENSOR_SIGNIFICANT_MOTION` | Significant motion event |
| 0x13 | `BNO085_SENSOR_STABILITY_CLASSIFIER` | Device stability state |
| 0x18 | `BNO085_SENSOR_STEP_DETECTOR` | Step event trigger |
| 0x19 | `BNO085_SENSOR_SHAKE_DETECTOR` | Shake/vibration detection |
| 0x1A | `BNO085_SENSOR_FLIP_DETECTOR` | Device flip motion |
| 0x1B | `BNO085_SENSOR_PICKUP_DETECTOR` | Device pickup detection |
| 0x1E | `BNO085_SENSOR_PERSONAL_ACTIVITY_CLASSIFIER` | Activity type (walk, run, vehicle, etc.) |
| 0x1F | `BNO085_SENSOR_SLEEP_DETECTOR` | Sleep state detection |
| 0x20 | `BNO085_SENSOR_TILT_DETECTOR` | Tilt change detection |
| 0x21 | `BNO085_SENSOR_POCKET_DETECTOR` | In-pocket detection |
| 0x22 | `BNO085_SENSOR_CIRCLE_DETECTOR` | Circular motion detection |

### Environmental Sensors

| Sensor ID | Constant | Output | Units |
|-----------|----------|--------|-------|
| 0x0A | `BNO085_SENSOR_PRESSURE` | Atmospheric pressure | Pa |
| 0x0B | `BNO085_SENSOR_AMBIENT_LIGHT` | Light intensity | lux |
| 0x0C | `BNO085_SENSOR_HUMIDITY` | Relative humidity | % |
| 0x0E | `BNO085_SENSOR_TEMPERATURE` | Ambient temperature | °C |

## Usage Examples

### Get Orientation Only

```c
void sensor_callback(bno085_handle_t handle,
                    const bno085_sensor_value_t *value,
                    void *user_context)
{
    if (value->sensor_id == BNO085_SENSOR_ROTATION_VECTOR) {
        printf("Q: i=%f, j=%f, k=%f, real=%f, acc=%f rad\n",
               value->data.rotation_vector.i,
               value->data.rotation_vector.j,
               value->data.rotation_vector.k,
               value->data.rotation_vector.real,
               value->data.rotation_vector.accuracy_rad);
    }
}

bno085_register_sensor_callback(bno085, sensor_callback, NULL);
bno085_enable_sensor(bno085, BNO085_SENSOR_ROTATION_VECTOR, 100000);
```

### Get All IMU Data

```c
bno085_register_sensor_callback(bno085, sensor_callback, NULL);
bno085_enable_sensor(bno085, BNO085_SENSOR_ACCELEROMETER, 50000);
bno085_enable_sensor(bno085, BNO085_SENSOR_GYROSCOPE_CALIBRATED, 50000);
bno085_enable_sensor(bno085, BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED, 50000);
bno085_enable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION, 50000);
```

### Activity Detection (Fitness Tracker)

```c
bno085_enable_sensor(bno085, BNO085_SENSOR_STEP_COUNTER, 500000);
bno085_enable_sensor(bno085, BNO085_SENSOR_PERSONAL_ACTIVITY_CLASSIFIER, 1000000);
bno085_enable_sensor(bno085, BNO085_SENSOR_SIGNIFICANT_MOTION, 1000000);
```

### Motion + Orientation (Robot/Drone)

```c
bno085_enable_sensor(bno085, BNO085_SENSOR_ROTATION_VECTOR, 50000);  // 20Hz
bno085_enable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION, 50000);
bno085_enable_sensor(bno085, BNO085_SENSOR_GYROSCOPE_CALIBRATED, 50000);
```

### Game Controller

```c
bno085_register_sensor_callback(bno085, sensor_callback, NULL);
bno085_enable_sensor(bno085, BNO085_SENSOR_GAME_ROTATION_VECTOR, 10000);  // 100Hz
```

## Sensor Data Structure

The callback receives `bno085_sensor_value_t` with:
- `sensor_id`: Which sensor sent this data
- `status`: Accuracy/reliability (0=unreliable, 1=low, 2=medium, 3=high)
- `timestamp_us`: Timestamp in microseconds
- `data`: Union with decoded sensor data

Access the appropriate data field based on sensor_id:

```c
if (value->sensor_id == BNO085_SENSOR_ACCELEROMETER) {
    float x = value->data.accelerometer.x;
    float y = value->data.accelerometer.y;
    float z = value->data.accelerometer.z;
}

if (value->sensor_id == BNO085_SENSOR_ROTATION_VECTOR) {
    float i = value->data.rotation_vector.i;
    float j = value->data.rotation_vector.j;
    float k = value->data.rotation_vector.k;
    float real = value->data.rotation_vector.real;
    float accuracy = value->data.rotation_vector.accuracy_rad;
}

if (value->sensor_id == BNO085_SENSOR_STEP_COUNTER) {
    uint32_t steps = value->data.step_counter.count;
}
```

## Limitations

- Up to `CONFIG_BNO085_MAX_INSTANCES` concurrent sensors (default 2, configurable to 1-8)
- Only one active BNO085 per I2C bus without a multiplexer
- The underlying SH2 library is not thread-safe for init/deinit; call from a single task

## Example Project

See `examples/basic_read/` for a complete working example.

## License

Apache-2.0. Includes CEVA SH2 library (also Apache-2.0). See `LICENSE` for details.
