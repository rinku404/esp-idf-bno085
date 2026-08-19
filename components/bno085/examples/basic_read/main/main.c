/*
 * BNO085 Basic Read Example
 *
 * This example demonstrates how to:
 * 1. Initialize the BNO085 sensor
 * 2. Register a callback to receive sensor data
 * 3. Enable the rotation vector sensor
 * 4. Print quaternion data in the main loop
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
 * Prints the quaternion (rotation vector) to the console.
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

    switch (value->sensor_id) {
        case BNO085_SENSOR_ROTATION_VECTOR:
            /* Print quaternion (most common orientation representation) */
            ESP_LOGI(TAG, "Quaternion: i=%.4f, j=%.4f, k=%.4f, real=%.4f, accuracy=%.2f°",
                     value->data.rotation_vector.i,
                     value->data.rotation_vector.j,
                     value->data.rotation_vector.k,
                     value->data.rotation_vector.real,
                     value->data.rotation_vector.accuracy_rad * 57.2958f);
            break;

        default:
            /* Ignore other sensor types in this example */
            break;
    }
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

    /* Enable the rotation vector sensor at 10 Hz (100 ms interval) */
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085, BNO085_SENSOR_ROTATION_VECTOR, 100000));
    ESP_LOGI(TAG, "Rotation Vector sensor enabled at 10 Hz");

    ESP_LOGI(TAG, "Starting sensor read loop (press Ctrl-C to stop)");

    /* Main service loop */
    while (1) {
        bno085_service(bno085);
        vTaskDelay(pdMS_TO_TICKS(10));  /* Service at 100 Hz */
    }
}
