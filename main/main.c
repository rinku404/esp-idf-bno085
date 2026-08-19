#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "bno085.h"
#include "sh2.h"
#include "sh2_SensorValue.h"

#define BNO085_I2C_PORT     I2C_NUM_0
#define BNO085_SDA_GPIO     GPIO_NUM_6     // Heltec Lora V3 SDA
#define BNO085_SCL_GPIO     GPIO_NUM_7     // Heltec Lora V3 SCL
#define BNO085_INT_GPIO     GPIO_NUM_5     // H_INTN pin
#define BNO085_RST_GPIO     GPIO_NUM_4     // RSTN pin

#define BNO085_I2C_ADDR     0x4A
#define BNO085_I2C_FREQ_HZ  400000

static const char *TAG = "app_main";

// Example sensor callback to print rotation vector data
static void sensor_callback(sh2_SensorEvent_t *event)
{
    if (!event) return;

    // Decode the sensor event into human-readable format
    sh2_SensorValue_t sensorValue;
    memset(&sensorValue, 0, sizeof(sensorValue));

    int rc = sh2_decodeSensorEvent(&sensorValue, event);
    if (rc != SH2_OK) {
        return;
    }

    // Print data based on sensor type
    switch (sensorValue.sensorId) {
        case SH2_ROTATION_VECTOR: {
            // Rotation Vector: quaternion (most accurate orientation)
            ESP_LOGI(TAG, "Rotation Vector: i=%.4f, j=%.4f, k=%.4f, real=%.4f, accuracy=%.1f°",
                     sensorValue.un.rotationVector.i,
                     sensorValue.un.rotationVector.j,
                     sensorValue.un.rotationVector.k,
                     sensorValue.un.rotationVector.real,
                     sensorValue.un.rotationVector.accuracy * 57.2958f);  // rad to degrees
            break;
        }
        case SH2_LINEAR_ACCELERATION: {
            // Linear acceleration (gravity removed)
            ESP_LOGI(TAG, "Linear Accel: x=%.2f, y=%.2f, z=%.2f m/s²",
                     sensorValue.un.linearAcceleration.x,
                     sensorValue.un.linearAcceleration.y,
                     sensorValue.un.linearAcceleration.z);
            break;
        }
        case SH2_GYROSCOPE_CALIBRATED: {
            // Calibrated gyroscope in rad/s
            ESP_LOGI(TAG, "Gyro: x=%.4f, y=%.4f, z=%.4f rad/s",
                     sensorValue.un.gyroscope.x,
                     sensorValue.un.gyroscope.y,
                     sensorValue.un.gyroscope.z);
            break;
        }
        case SH2_ACCELEROMETER: {
            // Accelerometer with gravity
            ESP_LOGI(TAG, "Accel: x=%.2f, y=%.2f, z=%.2f m/s²",
                     sensorValue.un.acceleration.x,
                     sensorValue.un.acceleration.y,
                     sensorValue.un.acceleration.z);
            break;
        }
        case SH2_MAGNETIC_FIELD_CALIBRATED: {
            // Magnetometer
            ESP_LOGI(TAG, "Mag: x=%.1f, y=%.1f, z=%.1f µT",
                     sensorValue.un.magneticField.x,
                     sensorValue.un.magneticField.y,
                     sensorValue.un.magneticField.z);
            break;
        }
        default:
            ESP_LOGD(TAG, "Sensor 0x%02x: timestamp=%lld µs", sensorValue.sensorId, event->timestamp_uS);
            break;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing BNO085 sensor driver...");

    i2c_master_bus_config_t bus_config = {
        .i2c_port = BNO085_I2C_PORT,
        .sda_io_num = BNO085_SDA_GPIO,
        .scl_io_num = BNO085_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_LOGI(TAG, "I2C bus initialized");

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO085_I2C_ADDR,
        .scl_speed_hz = BNO085_I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
    ESP_LOGI(TAG, "BNO085 device added to bus at address 0x%02X", BNO085_I2C_ADDR);

    ESP_ERROR_CHECK(bno085_init(dev_handle, BNO085_INT_GPIO, BNO085_RST_GPIO));
    ESP_LOGI(TAG, "BNO085 driver initialized");

    // Register sensor callback to receive data
    ESP_ERROR_CHECK(bno085_register_sensor_callback(sensor_callback, NULL));

    // Enable sensors with reporting intervals
    // Rotation Vector: 100ms interval (10 Hz)
    ESP_ERROR_CHECK(bno085_enable_sensor(SH2_ROTATION_VECTOR, 100000));

    // Optional: enable other sensors
    // Linear Acceleration: 100ms interval
    // ESP_ERROR_CHECK(bno085_enable_sensor(SH2_LINEAR_ACCELERATION, 100000));

    // Calibrated Gyroscope: 100ms interval
    // ESP_ERROR_CHECK(bno085_enable_sensor(SH2_GYROSCOPE_CALIBRATED, 100000));

    // Accelerometer: 100ms interval
    // ESP_ERROR_CHECK(bno085_enable_sensor(SH2_ACCELEROMETER, 100000));

    // Magnetometer: 100ms interval
    // ESP_ERROR_CHECK(bno085_enable_sensor(SH2_MAGNETIC_FIELD_CALIBRATED, 100000));

    ESP_LOGI(TAG, "Sensors enabled, starting service loop...");

    while (1) {
        bno085_service();
        vTaskDelay(pdMS_TO_TICKS(10));  // Service at 100Hz
    }
}
