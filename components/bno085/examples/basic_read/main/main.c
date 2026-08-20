#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "bno085.h"
#include "sdkconfig.h"
#include "sensor.h"
#include "output.h"

#define BNO085_I2C_PORT     I2C_NUM_0
#define BNO085_SDA_GPIO     ((gpio_num_t)CONFIG_APP_BNO085_I2C_SDA_GPIO)
#define BNO085_SCL_GPIO     ((gpio_num_t)CONFIG_APP_BNO085_I2C_SCL_GPIO)
#define BNO085_RST_GPIO     ((gpio_num_t)CONFIG_APP_BNO085_RST_GPIO)
#define BNO085_I2C_ADDR     CONFIG_APP_BNO085_I2C_ADDR
#define BNO085_I2C_FREQ_HZ  CONFIG_APP_BNO085_I2C_FREQ_HZ

static const char *TAG = "app_main";

static bno085_handle_t global_bno085_handle = NULL;

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing BNO085 sensor driver...");

    sensor_init();
    output_init();

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

    bno085_config_t config;
    bno085_config_default(&config);

    ESP_ERROR_CHECK(bno085_init(&config, dev_handle, CONFIG_APP_BNO085_INT_GPIO, BNO085_RST_GPIO, &global_bno085_handle));
    ESP_LOGI(TAG, "BNO085 driver initialized");

    sensor_install_isr();
    sensor_register_callback(global_bno085_handle);

    uint32_t report_interval_us = CONFIG_APP_BNO085_SENSOR_UPDATE_PERIOD_MS * 1000;

    #ifdef CONFIG_APP_BNO085_ENABLE_ROTATION_VECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_ROTATION_VECTOR, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_ACCELEROMETER
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_ACCELEROMETER, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_LINEAR_ACCELERATION
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_LINEAR_ACCELERATION, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_GRAVITY
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_GRAVITY, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_GYROSCOPE_CALIBRATED, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_GYROSCOPE_UNCALIBRATED
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_GYROSCOPE_UNCALIBRATED, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_MAGNETIC_FIELD_UNCALIBRATED
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GAME_ROTATION_VECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_GAME_ROTATION_VECTOR, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_GEOMAGNETIC_ROTATION_VECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_PRINT_AR_VR_STABILIZED_RV
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_AR_VR_STABILIZED_RV, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_GYRO_INTEGRATED_RV
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_GYRO_INTEGRATED_RV, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_RAW_ACCELEROMETER
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_RAW_ACCELEROMETER, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_RAW_GYROSCOPE
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_RAW_GYROSCOPE, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_RAW_MAGNETOMETER
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_RAW_MAGNETOMETER, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_PRESSURE
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_PRESSURE, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_HUMIDITY
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_HUMIDITY, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_TEMPERATURE
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_TEMPERATURE, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_AMBIENT_LIGHT
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_AMBIENT_LIGHT, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_STEP_COUNTER
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_STEP_COUNTER, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_STEP_DETECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_STEP_DETECTOR, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_STABILITY_CLASSIFIER
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_STABILITY_CLASSIFIER, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_PERSONAL_ACTIVITY_CLASSIFIER
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_PERSONAL_ACTIVITY_CLASSIFIER, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_SIGNIFICANT_MOTION
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_SIGNIFICANT_MOTION, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_SHAKE_DETECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_SHAKE_DETECTOR, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_TAP_DETECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_TAP_DETECTOR, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_FLIP_DETECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_FLIP_DETECTOR, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_PICKUP_DETECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_PICKUP_DETECTOR, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_TILT_DETECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_TILT_DETECTOR, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_POCKET_DETECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_POCKET_DETECTOR, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_CIRCLE_DETECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_CIRCLE_DETECTOR, report_interval_us);
    #endif
    #ifdef CONFIG_APP_BNO085_ENABLE_SLEEP_DETECTOR
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_SLEEP_DETECTOR, report_interval_us);
    #endif

    #if defined(CONFIG_APP_BNO085_ACCEL_CALC_NEEDED) && !defined(CONFIG_APP_BNO085_ENABLE_ACCELEROMETER)
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_ACCELEROMETER, report_interval_us);
    #endif

    #if defined(CONFIG_APP_BNO085_GYRO_CALC_NEEDED) && !defined(CONFIG_APP_BNO085_ENABLE_GYROSCOPE)
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_GYROSCOPE_CALIBRATED, report_interval_us);
    #endif

    #if defined(CONFIG_APP_BNO085_MAG_CALC_NEEDED) && !defined(CONFIG_APP_BNO085_ENABLE_MAGNETIC_FIELD)
    bno085_enable_sensor(global_bno085_handle, BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED, report_interval_us);
    #endif

    ESP_LOGI(TAG, "Starting service and output tasks...");

    sensor_start_task(global_bno085_handle);

    #ifdef CONFIG_APP_BNO085_OUTPUT_CSV
    output_start_task();
    output_start_timer();
    #endif

    ESP_LOGI(TAG, "Tasks started");

    vTaskDelete(NULL);
}
