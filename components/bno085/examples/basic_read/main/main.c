/*
 * BNO085 Basic Read Example
 *
 * This example demonstrates how to:
 * 1. Initialize the BNO085 sensor
 * 2. Register a callback to receive sensor data
 * 3. Enable sensors based on configuration
 * 4. Print data in verbose or Edge Impulse CSV format
 *
 * Configuration via menuconfig:
 * - Enable/disable individual sensors
 * - Choose output format (verbose or CSV for Edge Impulse)
 * - Set sampling rate
 *
 * Hardware Setup:
 * - BNO085 SDA connected to GPIO 6
 * - BNO085 SCL connected to GPIO 7
 * - BNO085 INT (H_INTN) connected to GPIO 5
 * - BNO085 RST (NRST) connected to GPIO 4
 * - BNO085 AD0 tied to GND (I2C address 0x4A)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "bno085.h"

/* Application configuration from menuconfig */
#include "sdkconfig.h"

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     GPIO_NUM_6
#define I2C_SCL_PIN     GPIO_NUM_7
#define I2C_FREQ_HZ     400000
#define I2C_ADDR        0x4A

#define BNO085_INT_PIN  GPIO_NUM_5
#define BNO085_RST_PIN  GPIO_NUM_4

static const char *TAG = "bno085_example";

/**
 * Callback invoked whenever the BNO085 sends sensor data.
 * Outputs data in either verbose or CSV format based on configuration.
 */
static void sensor_callback(bno085_handle_t handle,
                           const bno085_sensor_value_t *value,
                           void *user_context)
{
    (void) handle;
    (void) user_context;

    if (!value) {
        return;
    }

#ifdef CONFIG_APP_BNO085_OUTPUT_CSV
    /* CSV format for Edge Impulse */
    static bool first_sample = true;

    if (first_sample && CONFIG_APP_BNO085_CSV_PRINT_HEADER) {
        printf("timestamp_ms");
#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
        printf(",ax,ay,az");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
        printf(",gx,gy,gz");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_LINEAR_ACCELERATION
        printf(",lax,lay,laz");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
        printf(",mx,my,mz");
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
        printf(",qx,qy,qz,qw");
#endif
        printf("\n");
        first_sample = false;
    }

    printf("%llu", value->timestamp_us / 1000);

#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
    if (value->sensor_id == BNO085_SENSOR_ACCELEROMETER) {
        printf(",%.4f,%.4f,%.4f",
               value->data.accelerometer.x,
               value->data.accelerometer.y,
               value->data.accelerometer.z);
    }
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
    if (value->sensor_id == BNO085_SENSOR_GYROSCOPE_CALIBRATED) {
        printf(",%.4f,%.4f,%.4f",
               value->data.gyroscope.x,
               value->data.gyroscope.y,
               value->data.gyroscope.z);
    }
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_LINEAR_ACCELERATION
    if (value->sensor_id == BNO085_SENSOR_LINEAR_ACCELERATION) {
        printf(",%.4f,%.4f,%.4f",
               value->data.linear_acceleration.x,
               value->data.linear_acceleration.y,
               value->data.linear_acceleration.z);
    }
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
    if (value->sensor_id == BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED) {
        printf(",%.4f,%.4f,%.4f",
               value->data.magnetic_field.x,
               value->data.magnetic_field.y,
               value->data.magnetic_field.z);
    }
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
    if (value->sensor_id == BNO085_SENSOR_ROTATION_VECTOR) {
        printf(",%.4f,%.4f,%.4f,%.4f",
               value->data.rotation_vector.i,
               value->data.rotation_vector.j,
               value->data.rotation_vector.k,
               value->data.rotation_vector.real);
    }
#endif

    printf("\n");

#else
    /* Verbose format (human-readable) */
    switch (value->sensor_id) {
        case BNO085_SENSOR_ROTATION_VECTOR:
#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
            ESP_LOGI(TAG, "Quaternion: i=%.4f, j=%.4f, k=%.4f, real=%.4f",
                     value->data.rotation_vector.i,
                     value->data.rotation_vector.j,
                     value->data.rotation_vector.k,
                     value->data.rotation_vector.real);
#endif
            break;

        case BNO085_SENSOR_ACCELEROMETER:
#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
            ESP_LOGI(TAG, "Accel: x=%.2f, y=%.2f, z=%.2f m/s²",
                     value->data.accelerometer.x,
                     value->data.accelerometer.y,
                     value->data.accelerometer.z);
#endif
            break;

        case BNO085_SENSOR_GYROSCOPE_CALIBRATED:
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
            ESP_LOGI(TAG, "Gyro: x=%.4f, y=%.4f, z=%.4f rad/s",
                     value->data.gyroscope.x,
                     value->data.gyroscope.y,
                     value->data.gyroscope.z);
#endif
            break;

        case BNO085_SENSOR_LINEAR_ACCELERATION:
#ifdef CONFIG_APP_BNO085_ENABLE_LINEAR_ACCELERATION
            ESP_LOGI(TAG, "Linear Accel: x=%.2f, y=%.2f, z=%.2f m/s²",
                     value->data.linear_acceleration.x,
                     value->data.linear_acceleration.y,
                     value->data.linear_acceleration.z);
#endif
            break;

        case BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED:
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
            ESP_LOGI(TAG, "Mag: x=%.1f, y=%.1f, z=%.1f µT",
                     value->data.magnetic_field.x,
                     value->data.magnetic_field.y,
                     value->data.magnetic_field.z);
#endif
            break;

        default:
            break;
    }
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "BNO085 Basic Read Example");

    /* Initialize I2C bus */
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_LOGI(TAG, "I2C bus initialized");

    /* Add BNO085 device to the bus */
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
    ESP_LOGI(TAG, "BNO085 device added to I2C bus");

    /* Initialize BNO085 */
    bno085_config_t config;
    bno085_config_default(&config);

    bno085_handle_t bno085;
    ESP_ERROR_CHECK(bno085_init(&config, dev_handle, BNO085_INT_PIN, BNO085_RST_PIN, &bno085));
    ESP_LOGI(TAG, "BNO085 initialized");

    /* Register callback to receive sensor data */
    ESP_ERROR_CHECK(bno085_register_sensor_callback(bno085, sensor_callback, NULL));
    ESP_LOGI(TAG, "Sensor callback registered");

    /* Enable sensors based on Kconfig settings */
    uint32_t report_interval_us = (1000000 / CONFIG_APP_BNO085_SAMPLING_RATE_HZ);

#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085, BNO085_SENSOR_ROTATION_VECTOR, report_interval_us));
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085, BNO085_SENSOR_ACCELEROMETER, report_interval_us));
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085, BNO085_SENSOR_GYROSCOPE_CALIBRATED, report_interval_us));
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_LINEAR_ACCELERATION
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION, report_interval_us));
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085, BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED, report_interval_us));
#endif

#ifdef CONFIG_APP_BNO085_OUTPUT_CSV
    ESP_LOGI(TAG, "Output format: CSV (Edge Impulse), sampling rate: %d Hz", CONFIG_APP_BNO085_SAMPLING_RATE_HZ);
#else
    ESP_LOGI(TAG, "Output format: Verbose, sampling rate: %d Hz", CONFIG_APP_BNO085_SAMPLING_RATE_HZ);
#endif

    ESP_LOGI(TAG, "Starting sensor read loop");

    /* Main service loop */
    while (1) {
        bno085_service(bno085);
        vTaskDelay(pdMS_TO_TICKS(10));  /* Service at 100 Hz */
    }
}
