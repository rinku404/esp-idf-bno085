#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "bno085.h"

/* Application configuration from menuconfig */
#include "sdkconfig.h"

#define BNO085_I2C_PORT     I2C_NUM_0
#define BNO085_SDA_GPIO     ((gpio_num_t)CONFIG_APP_BNO085_I2C_SDA_GPIO)
#define BNO085_SCL_GPIO     ((gpio_num_t)CONFIG_APP_BNO085_I2C_SCL_GPIO)
#define BNO085_INT_GPIO     ((gpio_num_t)CONFIG_APP_BNO085_INT_GPIO)
#define BNO085_RST_GPIO     ((gpio_num_t)CONFIG_APP_BNO085_RST_GPIO)

#define BNO085_I2C_ADDR     CONFIG_APP_BNO085_I2C_ADDR
#define BNO085_I2C_FREQ_HZ  CONFIG_APP_BNO085_I2C_FREQ_HZ

static const char *TAG = "app_main";

/* CSV data buffer for accumulating sensor values */
#ifdef CONFIG_APP_BNO085_OUTPUT_CSV
typedef struct {
    uint64_t timestamp_us;
    bool has_accel;
    float accel[3];
    bool has_gyro;
    float gyro[3];
    bool has_linear_accel;
    float linear_accel[3];
    bool has_mag;
    float mag[3];
    bool has_quat;
    float quat[4];
} csv_buffer_t;

static csv_buffer_t csv_buffer = {0};
static csv_buffer_t csv_last_values = {0};
static bool csv_header_printed = false;
#endif

static void sensor_callback(bno085_handle_t handle, const bno085_sensor_value_t *value, void *user_context)
{
    (void) handle;
    (void) user_context;

    if (!value) return;

#ifdef CONFIG_APP_BNO085_OUTPUT_CSV
    /* CSV format for CSV - buffer values by timestamp */

    /* Print header on first sample */
    if (!csv_header_printed && CONFIG_APP_BNO085_CSV_PRINT_HEADER) {
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
        csv_header_printed = true;
    }

    /* Check if this is a new sample (timestamp differs by more than tolerance) */
    uint64_t time_diff_ms = 0;
    if (csv_buffer.timestamp_us != 0) {
        time_diff_ms = (value->timestamp_us - csv_buffer.timestamp_us) / 1000;
    }

    if (csv_buffer.timestamp_us != 0 && time_diff_ms > CONFIG_APP_BNO085_CSV_TIMESTAMP_TOLERANCE_MS) {
        /* Print the previous sample, using last known values if sensor didn't report this cycle */
        printf("%llu", csv_buffer.timestamp_us / 1000);
#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
        if (csv_buffer.has_accel) {
            printf(",%.4f,%.4f,%.4f", csv_buffer.accel[0], csv_buffer.accel[1], csv_buffer.accel[2]);
            csv_last_values.has_accel = true;
            memcpy(csv_last_values.accel, csv_buffer.accel, sizeof(csv_buffer.accel));
        } else if (csv_last_values.has_accel) {
            printf(",%.4f,%.4f,%.4f", csv_last_values.accel[0], csv_last_values.accel[1], csv_last_values.accel[2]);
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
        if (csv_buffer.has_gyro) {
            printf(",%.4f,%.4f,%.4f", csv_buffer.gyro[0], csv_buffer.gyro[1], csv_buffer.gyro[2]);
            csv_last_values.has_gyro = true;
            memcpy(csv_last_values.gyro, csv_buffer.gyro, sizeof(csv_buffer.gyro));
        } else if (csv_last_values.has_gyro) {
            printf(",%.4f,%.4f,%.4f", csv_last_values.gyro[0], csv_last_values.gyro[1], csv_last_values.gyro[2]);
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_LINEAR_ACCELERATION
        if (csv_buffer.has_linear_accel) {
            printf(",%.4f,%.4f,%.4f", csv_buffer.linear_accel[0], csv_buffer.linear_accel[1], csv_buffer.linear_accel[2]);
            csv_last_values.has_linear_accel = true;
            memcpy(csv_last_values.linear_accel, csv_buffer.linear_accel, sizeof(csv_buffer.linear_accel));
        } else if (csv_last_values.has_linear_accel) {
            printf(",%.4f,%.4f,%.4f", csv_last_values.linear_accel[0], csv_last_values.linear_accel[1], csv_last_values.linear_accel[2]);
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
        if (csv_buffer.has_mag) {
            printf(",%.4f,%.4f,%.4f", csv_buffer.mag[0], csv_buffer.mag[1], csv_buffer.mag[2]);
            csv_last_values.has_mag = true;
            memcpy(csv_last_values.mag, csv_buffer.mag, sizeof(csv_buffer.mag));
        } else if (csv_last_values.has_mag) {
            printf(",%.4f,%.4f,%.4f", csv_last_values.mag[0], csv_last_values.mag[1], csv_last_values.mag[2]);
        }
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
        if (csv_buffer.has_quat) {
            printf(",%.4f,%.4f,%.4f,%.4f", csv_buffer.quat[0], csv_buffer.quat[1], csv_buffer.quat[2], csv_buffer.quat[3]);
            csv_last_values.has_quat = true;
            memcpy(csv_last_values.quat, csv_buffer.quat, sizeof(csv_buffer.quat));
        } else if (csv_last_values.has_quat) {
            printf(",%.4f,%.4f,%.4f,%.4f", csv_last_values.quat[0], csv_last_values.quat[1], csv_last_values.quat[2], csv_last_values.quat[3]);
        }
#endif
        printf("\n");
        fflush(stdout);

        /* Reset buffer for new sample (will be filled with current values if they arrive) */
        memset(&csv_buffer, 0, sizeof(csv_buffer));
    }

    /* Update buffer with current sensor value */
    csv_buffer.timestamp_us = value->timestamp_us;

    switch (value->sensor_id) {
#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
        case BNO085_SENSOR_ACCELEROMETER:
            csv_buffer.has_accel = true;
            csv_buffer.accel[0] = value->data.accelerometer.x;
            csv_buffer.accel[1] = value->data.accelerometer.y;
            csv_buffer.accel[2] = value->data.accelerometer.z;
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
        case BNO085_SENSOR_GYROSCOPE_CALIBRATED:
            csv_buffer.has_gyro = true;
            csv_buffer.gyro[0] = value->data.gyroscope.x;
            csv_buffer.gyro[1] = value->data.gyroscope.y;
            csv_buffer.gyro[2] = value->data.gyroscope.z;
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_LINEAR_ACCELERATION
        case BNO085_SENSOR_LINEAR_ACCELERATION:
            csv_buffer.has_linear_accel = true;
            csv_buffer.linear_accel[0] = value->data.linear_acceleration.x;
            csv_buffer.linear_accel[1] = value->data.linear_acceleration.y;
            csv_buffer.linear_accel[2] = value->data.linear_acceleration.z;
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
        case BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED:
            csv_buffer.has_mag = true;
            csv_buffer.mag[0] = value->data.magnetic_field.x;
            csv_buffer.mag[1] = value->data.magnetic_field.y;
            csv_buffer.mag[2] = value->data.magnetic_field.z;
            break;
#endif
#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
        case BNO085_SENSOR_ROTATION_VECTOR:
            csv_buffer.has_quat = true;
            csv_buffer.quat[0] = value->data.rotation_vector.i;
            csv_buffer.quat[1] = value->data.rotation_vector.j;
            csv_buffer.quat[2] = value->data.rotation_vector.k;
            csv_buffer.quat[3] = value->data.rotation_vector.real;
            break;
#endif
        default:
            break;
    }

#else
    /* Verbose format (human-readable) */
    switch (value->sensor_id) {
        case BNO085_SENSOR_ROTATION_VECTOR:
#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
            ESP_LOGI(TAG, "Rotation Vector: i=%.4f, j=%.4f, k=%.4f, real=%.4f, accuracy=%.1f°",
                     value->data.rotation_vector.i,
                     value->data.rotation_vector.j,
                     value->data.rotation_vector.k,
                     value->data.rotation_vector.real,
                     value->data.rotation_vector.accuracy_rad * 57.2958f);
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

        case BNO085_SENSOR_GYROSCOPE_CALIBRATED:
#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
            ESP_LOGI(TAG, "Gyro: x=%.4f, y=%.4f, z=%.4f rad/s",
                     value->data.gyroscope.x,
                     value->data.gyroscope.y,
                     value->data.gyroscope.z);
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

    /* Initialize BNO085 with config from menuconfig */
    bno085_config_t config;
    bno085_config_default(&config);

    bno085_handle_t bno085_handle;
    ESP_ERROR_CHECK(bno085_init(&config, dev_handle, BNO085_INT_GPIO, BNO085_RST_GPIO, &bno085_handle));
    ESP_LOGI(TAG, "BNO085 driver initialized");

    /* Register sensor callback to receive data */
    ESP_ERROR_CHECK(bno085_register_sensor_callback(bno085_handle, sensor_callback, NULL));

    /* Enable sensors based on Kconfig settings */
    uint32_t report_interval_us = (1000000 / CONFIG_APP_BNO085_SAMPLING_RATE_HZ);

#ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085_handle, BNO085_SENSOR_ROTATION_VECTOR, report_interval_us));
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_LINEAR_ACCELERATION
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085_handle, BNO085_SENSOR_LINEAR_ACCELERATION, report_interval_us));
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085_handle, BNO085_SENSOR_GYROSCOPE_CALIBRATED, report_interval_us));
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085_handle, BNO085_SENSOR_ACCELEROMETER, report_interval_us));
#endif

#ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
    ESP_ERROR_CHECK(bno085_enable_sensor(bno085_handle, BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED, report_interval_us));
#endif

#ifdef CONFIG_APP_BNO085_OUTPUT_CSV
    ESP_LOGI(TAG, "Output format: CSV (CSV), sampling rate: %d Hz", CONFIG_APP_BNO085_SAMPLING_RATE_HZ);
#else
    ESP_LOGI(TAG, "Output format: Verbose, sampling rate: %d Hz", CONFIG_APP_BNO085_SAMPLING_RATE_HZ);
#endif
    
    ESP_LOGI(TAG, "Sensors enabled, starting service loop...");

    while (1) {
        bno085_service(bno085_handle);
        vTaskDelay(pdMS_TO_TICKS(10));  // Service at 100Hz
    }
}
