#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "bno085.h"

#define BNO085_I2C_PORT     I2C_NUM_0
#define BNO085_SDA_GPIO     GPIO_NUM_6     // Heltec Lora V3 SDA
#define BNO085_SCL_GPIO     GPIO_NUM_7     // Heltec Lora V3 SCL
#define BNO085_INT_GPIO     GPIO_NUM_5     // H_INTN pin
#define BNO085_RST_GPIO     GPIO_NUM_4     // RSTN pin

#define BNO085_I2C_ADDR     0x4A
#define BNO085_I2C_FREQ_HZ  400000

static const char *TAG = "app_main";

static void sensor_callback(bno085_handle_t handle, const bno085_sensor_value_t *value, void *user_context)
{
    (void) handle;
    (void) user_context;

    if (!value) return;

    switch (value->sensor_id) {
        case BNO085_SENSOR_ROTATION_VECTOR: {
            ESP_LOGI(TAG, "Rotation Vector: i=%.4f, j=%.4f, k=%.4f, real=%.4f, accuracy=%.1f°",
                     value->data.rotation_vector.i,
                     value->data.rotation_vector.j,
                     value->data.rotation_vector.k,
                     value->data.rotation_vector.real,
                     value->data.rotation_vector.accuracy_rad * 57.2958f);
            break;
        }
        case BNO085_SENSOR_LINEAR_ACCELERATION: {
            ESP_LOGI(TAG, "Linear Accel: x=%.2f, y=%.2f, z=%.2f m/s²",
                     value->data.linear_acceleration.x,
                     value->data.linear_acceleration.y,
                     value->data.linear_acceleration.z);
            break;
        }
        case BNO085_SENSOR_GYROSCOPE_CALIBRATED: {
            ESP_LOGI(TAG, "Gyro: x=%.4f, y=%.4f, z=%.4f rad/s",
                     value->data.gyroscope.x,
                     value->data.gyroscope.y,
                     value->data.gyroscope.z);
            break;
        }
        case BNO085_SENSOR_ACCELEROMETER: {
            ESP_LOGI(TAG, "Accel: x=%.2f, y=%.2f, z=%.2f m/s²",
                     value->data.accelerometer.x,
                     value->data.accelerometer.y,
                     value->data.accelerometer.z);
            break;
        }
        case BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED: {
            ESP_LOGI(TAG, "Mag: x=%.1f, y=%.1f, z=%.1f µT",
                     value->data.magnetic_field.x,
                     value->data.magnetic_field.y,
                     value->data.magnetic_field.z);
            break;
        }
        default:
            ESP_LOGD(TAG, "Sensor 0x%02x: status=%u, timestamp=%llu µs",
                     value->sensor_id, value->status, value->timestamp_us);
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

    /* Initialize BNO085 with default configuration */
    bno085_config_t config;
    bno085_config_default(&config);

    bno085_handle_t bno085_handle;
    ESP_ERROR_CHECK(bno085_init(&config, dev_handle, BNO085_INT_GPIO, BNO085_RST_GPIO, &bno085_handle));
    ESP_LOGI(TAG, "BNO085 driver initialized");

    /* Register sensor callback to receive data */
    ESP_ERROR_CHECK(bno085_register_sensor_callback(bno085_handle, sensor_callback, NULL));

    /* Enable sensors with reporting intervals */
    // Rotation Vector: 100ms interval (10 Hz)
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085_handle, BNO085_SENSOR_ROTATION_VECTOR, 100000));

    /* Optional: enable other sensors */
    // Linear Acceleration: 100ms interval
    // ESP_ERROR_CHECK(bno085_enable_sensor(bno085_handle, BNO085_SENSOR_LINEAR_ACCELERATION, 100000));

    // Calibrated Gyroscope: 100ms interval
    // ESP_ERROR_CHECK(bno085_enable_sensor(bno085_handle, BNO085_SENSOR_GYROSCOPE_CALIBRATED, 100000));

    // Accelerometer: 100ms interval
    // ESP_ERROR_CHECK(bno085_enable_sensor(bno085_handle, BNO085_SENSOR_ACCELEROMETER, 100000));

    // Magnetometer: 100ms interval
    // ESP_ERROR_CHECK(bno085_enable_sensor(bno085_handle, BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED, 100000));

    ESP_LOGI(TAG, "Sensors enabled, starting service loop...");

    while (1) {
        bno085_service(bno085_handle);
        vTaskDelay(pdMS_TO_TICKS(10));  // Service at 100Hz
    }
}
