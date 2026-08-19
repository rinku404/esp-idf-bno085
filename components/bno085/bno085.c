#include "bno085.h"
#include "sh2.h"
#include "sh2_hal.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define BNO085_SHTP_HDR_LEN   (4)

static const char *TAG = "bno085";

/* Internal device structure (opaque to user) */
struct bno085_dev_t {
    sh2_Hal_t hal_base;
    i2c_master_dev_handle_t i2c_dev;
    gpio_num_t int_pin;
    gpio_num_t reset_pin;
    bno085_config_t config;
    bno085_sensor_callback_t user_callback;
    void *user_cookie;
    bool initialized;
};

/* Global: tracks the active instance (SH2 library supports only one) */
static bno085_handle_t s_active_instance = NULL;

/* ============================================================================
 * SH2 HAL Callbacks
 * ========================================================================== */

static int bno085_hal_open(sh2_Hal_t *self)
{
    struct bno085_dev_t *dev = (struct bno085_dev_t *)self;

    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    bool success = false;

    for (uint8_t attempts = 0; attempts < dev->config.reset_retry_count; attempts++) {
        esp_err_t err = i2c_master_transmit(dev->i2c_dev, softreset_pkt, sizeof(softreset_pkt),
                                            pdMS_TO_TICKS(dev->config.i2c_timeout_ms));
        if (err == ESP_OK) {
            success = true;
            ESP_LOGI(TAG, "Soft-reset I2C packet sent successfully");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(dev->config.reset_retry_delay_ms));
    }

    if (!success) {
        ESP_LOGE(TAG, "Failed to send soft-reset I2C packet after %u attempts",
                 dev->config.reset_retry_count);
        return SH2_ERR_IO;
    }

    vTaskDelay(pdMS_TO_TICKS(dev->config.reset_settle_delay_ms));
    return SH2_OK;
}

static void bno085_hal_close(sh2_Hal_t *self)
{
    struct bno085_dev_t *dev = (struct bno085_dev_t *)self;
    gpio_set_level(dev->reset_pin, 0);
}

static int bno085_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    struct bno085_dev_t *dev = (struct bno085_dev_t *)self;

    int pin_level = gpio_get_level(dev->int_pin);
    if (pin_level != 0) {
        /* Debug: only log first few times to avoid spam */
        static uint32_t no_data_count = 0;
        if (no_data_count++ % 1000 == 0) {
            ESP_LOGD(TAG, "INT pin is HIGH (no data), count=%lu", no_data_count);
        }
        return 0;
    }

    ESP_LOGD(TAG, "INT pin is LOW, reading I2C data");
    *t_us = (uint32_t) esp_timer_get_time();

    uint8_t header[BNO085_SHTP_HDR_LEN];
    esp_err_t err = i2c_master_receive(dev->i2c_dev, header, sizeof(header),
                                       pdMS_TO_TICKS(dev->config.i2c_timeout_ms));
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "I2C header read failed: %s", esp_err_to_name(err));
        return 0;
    }
    ESP_LOGD(TAG, "I2C header read OK: 0x%02x 0x%02x", header[0], header[1]);

    uint16_t packet_len = (uint16_t)(header[0] | (header[1] << 8)) & 0x7FFF;
    if (packet_len < BNO085_SHTP_HDR_LEN) {
        return 0;
    }

    if (packet_len > len) {
        packet_len = (uint16_t)len;
    }
    if (packet_len > SH2_HAL_MAX_TRANSFER_IN) {
        packet_len = SH2_HAL_MAX_TRANSFER_IN;
    }

    err = i2c_master_receive(dev->i2c_dev, pBuffer, packet_len,
                             pdMS_TO_TICKS(dev->config.i2c_timeout_ms));
    if (err != ESP_OK) {
        return 0;
    }

    return (int) packet_len;
}

static int bno085_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    struct bno085_dev_t *dev = (struct bno085_dev_t *)self;

    esp_err_t err = i2c_master_transmit(dev->i2c_dev, pBuffer, len,
                                        pdMS_TO_TICKS(dev->config.i2c_timeout_ms));
    if (err != ESP_OK) {
        return 0;
    }
    return (int) len;
}

static uint32_t bno085_hal_getTimeUs(sh2_Hal_t *self)
{
    (void) self;
    return (uint32_t) esp_timer_get_time();
}

/* ============================================================================
 * Event and Sensor Callbacks (Internal)
 * ========================================================================== */

static void bno085_sensor_cb(void *cookie, sh2_SensorEvent_t *pEvent)
{
    struct bno085_dev_t *dev = (struct bno085_dev_t *)cookie;
    if (!dev || !dev->user_callback || !pEvent) {
        return;
    }

    sh2_SensorValue_t sh2_value;
    memset(&sh2_value, 0, sizeof(sh2_value));

    int rc = sh2_decodeSensorEvent(&sh2_value, pEvent);
    if (rc != SH2_OK) {
        ESP_LOGD(TAG, "sh2_decodeSensorEvent failed, rc=%d", rc);
        return;
    }

    /* Map SH2 sensor value to the public bno085_sensor_value_t type */
    bno085_sensor_value_t value;
    memset(&value, 0, sizeof(value));

    value.sensor_id = sh2_value.sensorId;
    value.status = sh2_value.status & 0x03;  /* Accuracy is in bits 1-0 */
    value.timestamp_us = sh2_value.timestamp;

    /* Decode the five known sensor types */
    switch (sh2_value.sensorId) {
        case BNO085_SENSOR_ROTATION_VECTOR:
            value.data.rotation_vector.i = sh2_value.un.rotationVector.i;
            value.data.rotation_vector.j = sh2_value.un.rotationVector.j;
            value.data.rotation_vector.k = sh2_value.un.rotationVector.k;
            value.data.rotation_vector.real = sh2_value.un.rotationVector.real;
            value.data.rotation_vector.accuracy_rad = sh2_value.un.rotationVector.accuracy;
            break;

        case BNO085_SENSOR_ACCELEROMETER:
            value.data.accelerometer.x = sh2_value.un.accelerometer.x;
            value.data.accelerometer.y = sh2_value.un.accelerometer.y;
            value.data.accelerometer.z = sh2_value.un.accelerometer.z;
            break;

        case BNO085_SENSOR_LINEAR_ACCELERATION:
            value.data.linear_acceleration.x = sh2_value.un.linearAcceleration.x;
            value.data.linear_acceleration.y = sh2_value.un.linearAcceleration.y;
            value.data.linear_acceleration.z = sh2_value.un.linearAcceleration.z;
            break;

        case BNO085_SENSOR_GYROSCOPE_CALIBRATED:
            value.data.gyroscope.x = sh2_value.un.gyroscope.x;
            value.data.gyroscope.y = sh2_value.un.gyroscope.y;
            value.data.gyroscope.z = sh2_value.un.gyroscope.z;
            break;

        case BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED:
            value.data.magnetic_field.x = sh2_value.un.magneticField.x;
            value.data.magnetic_field.y = sh2_value.un.magneticField.y;
            value.data.magnetic_field.z = sh2_value.un.magneticField.z;
            break;

        default:
            /* Unknown sensor type: data union remains zeroed, metadata populated */
            ESP_LOGD(TAG, "Unmapped sensor ID 0x%02x, timestamp=%llu", sh2_value.sensorId, value.timestamp_us);
            break;
    }

    /* Invoke the user callback with the decoded value */
    dev->user_callback((bno085_handle_t)dev, &value, dev->user_cookie);
}

static void bno085_event_cb(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    (void) cookie;
    if (!pEvent) {
        return;
    }

    switch (pEvent->eventId) {
        case SH2_RESET:
            ESP_LOGI(TAG, "BNO085 sensor hub reset event");
            break;
        case SH2_SHTP_EVENT:
            ESP_LOGD(TAG, "SHTP event: %d", pEvent->shtpEvent);
            break;
        case SH2_GET_FEATURE_RESP:
            ESP_LOGD(TAG, "Get-feature response received");
            break;
        default:
            ESP_LOGD(TAG, "Unhandled sh2 async event id=%lu", (unsigned long) pEvent->eventId);
            break;
    }
}

/* ============================================================================
 * Public API
 * ========================================================================== */

void bno085_config_default(bno085_config_t *config)
{
    if (!config) {
        return;
    }
    config->i2c_timeout_ms = 100;
    config->reset_retry_count = 5;
    config->reset_retry_delay_ms = 30;
    config->reset_settle_delay_ms = 300;
}

esp_err_t bno085_init(const bno085_config_t *config,
                      i2c_master_dev_handle_t i2c_dev,
                      gpio_num_t int_pin,
                      gpio_num_t reset_pin,
                      bno085_handle_t *out_handle)
{
    if (!i2c_dev || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_active_instance != NULL) {
        ESP_LOGE(TAG, "BNO085 already initialized; only one instance supported at a time");
        return ESP_ERR_INVALID_STATE;
    }

    /* Allocate device structure */
    struct bno085_dev_t *dev = heap_caps_calloc(1, sizeof(struct bno085_dev_t), MALLOC_CAP_DEFAULT);
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate device structure");
        return ESP_ERR_NO_MEM;
    }

    /* Store configuration (or use defaults if not provided) */
    if (config) {
        dev->config = *config;
    } else {
        bno085_config_default(&dev->config);
    }

    dev->i2c_dev = i2c_dev;
    dev->int_pin = int_pin;
    dev->reset_pin = reset_pin;

    /* Configure GPIO pins */
    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << int_pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&int_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INT pin: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << reset_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&rst_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure RST pin: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }
    gpio_set_level(reset_pin, 1);

    /* Perform hardware reset pulse */
    ESP_LOGI(TAG, "Asserting NRST (reset pulse)...");
    gpio_set_level(reset_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(reset_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Set up HAL callback table */
    dev->hal_base.open      = bno085_hal_open;
    dev->hal_base.close     = bno085_hal_close;
    dev->hal_base.read      = bno085_hal_read;
    dev->hal_base.write     = bno085_hal_write;
    dev->hal_base.getTimeUs = bno085_hal_getTimeUs;

    /* Open SH2 session */
    ESP_LOGI(TAG, "Calling sh2_open()...");
    int rc = sh2_open(&dev->hal_base, bno085_event_cb, dev);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "sh2_open() failed, rc=%d", rc);
        free(dev);
        return ESP_FAIL;
    }

    dev->initialized = true;
    s_active_instance = (bno085_handle_t)dev;
    *out_handle = (bno085_handle_t)dev;

    /* Debug: Check GPIO states after init */
    ESP_LOGI(TAG, "BNO085 initialized successfully (int_pin=%d, reset_pin=%d)", int_pin, reset_pin);
    ESP_LOGI(TAG, "GPIO states: INT=%d, RST=%d", gpio_get_level(int_pin), gpio_get_level(reset_pin));
    return ESP_OK;
}

void bno085_service(bno085_handle_t handle)
{
    if (!handle) {
        return;
    }

    struct bno085_dev_t *dev = (struct bno085_dev_t *)handle;
    if (!dev->initialized) {
        return;
    }

    /* Set the user callback for this session before calling sh2_service */
    if (dev->user_callback) {
        sh2_setSensorCallback(bno085_sensor_cb, dev);
    }

    sh2_service();
}

void bno085_deinit(bno085_handle_t handle)
{
    if (!handle) {
        return;
    }

    struct bno085_dev_t *dev = (struct bno085_dev_t *)handle;
    if (!dev->initialized) {
        return;
    }

    sh2_close();
    gpio_set_level(dev->reset_pin, 0);

    dev->initialized = false;
    s_active_instance = NULL;
    free(dev);
}

esp_err_t bno085_register_sensor_callback(bno085_handle_t handle,
                                          bno085_sensor_callback_t callback,
                                          void *user_context)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    struct bno085_dev_t *dev = (struct bno085_dev_t *)handle;
    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    dev->user_callback = callback;
    dev->user_cookie = user_context;

    if (callback) {
        int rc = sh2_setSensorCallback(bno085_sensor_cb, dev);
        if (rc != SH2_OK) {
            ESP_LOGE(TAG, "sh2_setSensorCallback() failed, rc=%d", rc);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Sensor callback registered");
    } else {
        sh2_setSensorCallback(NULL, NULL);
        ESP_LOGI(TAG, "Sensor callback unregistered");
    }

    return ESP_OK;
}

esp_err_t bno085_enable_sensor(bno085_handle_t handle,
                               uint8_t sensor_id,
                               uint32_t report_interval_us)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    struct bno085_dev_t *dev = (struct bno085_dev_t *)handle;
    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Validate sensor ID is in valid SH2 range */
    if (sensor_id == 0 || sensor_id > 0x2A) {
        ESP_LOGE(TAG, "Invalid sensor ID 0x%02x", sensor_id);
        return ESP_ERR_INVALID_ARG;
    }

    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.alwaysOnEnabled = false;
    config.sniffEnabled = false;
    config.reportInterval_us = report_interval_us;

    int rc = sh2_setSensorConfig(sensor_id, &config);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "Failed to enable sensor 0x%02x, rc=%d", sensor_id, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor 0x%02x enabled with interval %u us", sensor_id, report_interval_us);
    return ESP_OK;
}

esp_err_t bno085_disable_sensor(bno085_handle_t handle,
                                uint8_t sensor_id)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    struct bno085_dev_t *dev = (struct bno085_dev_t *)handle;
    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));
    config.reportInterval_us = 0;  /* 0 = disable */

    int rc = sh2_setSensorConfig(sensor_id, &config);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "Failed to disable sensor 0x%02x, rc=%d", sensor_id, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor 0x%02x disabled", sensor_id);
    return ESP_OK;
}
