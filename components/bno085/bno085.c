#include "bno085.h"
#include "sh2.h"
#include "sh2_hal.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "sh2_multi.h"
#include "euler.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
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
    const sh2_instance_slot_t *slot;
    bool initialized;
};

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
    (void) self;
    /* Don't drive RST during normal operation — only in bno085_deinit() on shutdown */
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

    /* Decode all supported sensor types */
    switch (sh2_value.sensorId) {
        case BNO085_SENSOR_ACCELEROMETER:
            value.data.accelerometer.x = sh2_value.un.accelerometer.x;
            value.data.accelerometer.y = sh2_value.un.accelerometer.y;
            value.data.accelerometer.z = sh2_value.un.accelerometer.z;
            break;

        case BNO085_SENSOR_GYROSCOPE_CALIBRATED:
            value.data.gyroscope_calibrated.x = sh2_value.un.gyroscope.x;
            value.data.gyroscope_calibrated.y = sh2_value.un.gyroscope.y;
            value.data.gyroscope_calibrated.z = sh2_value.un.gyroscope.z;
            break;

        case BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED:
            value.data.magnetic_field_calibrated.x = sh2_value.un.magneticField.x;
            value.data.magnetic_field_calibrated.y = sh2_value.un.magneticField.y;
            value.data.magnetic_field_calibrated.z = sh2_value.un.magneticField.z;
            break;

        case BNO085_SENSOR_LINEAR_ACCELERATION:
            value.data.linear_acceleration.x = sh2_value.un.linearAcceleration.x;
            value.data.linear_acceleration.y = sh2_value.un.linearAcceleration.y;
            value.data.linear_acceleration.z = sh2_value.un.linearAcceleration.z;
            break;

        case BNO085_SENSOR_ROTATION_VECTOR:
            value.data.quaternion.i = sh2_value.un.rotationVector.i;
            value.data.quaternion.j = sh2_value.un.rotationVector.j;
            value.data.quaternion.k = sh2_value.un.rotationVector.k;
            value.data.quaternion.real = sh2_value.un.rotationVector.real;
            value.data.quaternion.accuracy_rad = sh2_value.un.rotationVector.accuracy;
            break;

        case BNO085_SENSOR_GRAVITY:
            value.data.gravity.x = sh2_value.un.gravity.x;
            value.data.gravity.y = sh2_value.un.gravity.y;
            value.data.gravity.z = sh2_value.un.gravity.z;
            break;

        case BNO085_SENSOR_GYROSCOPE_UNCALIBRATED:
            value.data.gyroscope_uncalibrated.x = sh2_value.un.gyroscopeUncal.x;
            value.data.gyroscope_uncalibrated.y = sh2_value.un.gyroscopeUncal.y;
            value.data.gyroscope_uncalibrated.z = sh2_value.un.gyroscopeUncal.z;
            value.data.gyroscope_uncalibrated.bias_x = sh2_value.un.gyroscopeUncal.biasX;
            value.data.gyroscope_uncalibrated.bias_y = sh2_value.un.gyroscopeUncal.biasY;
            value.data.gyroscope_uncalibrated.bias_z = sh2_value.un.gyroscopeUncal.biasZ;
            break;

        case BNO085_SENSOR_GAME_ROTATION_VECTOR:
            value.data.game_rotation_vector.i = sh2_value.un.gameRotationVector.i;
            value.data.game_rotation_vector.j = sh2_value.un.gameRotationVector.j;
            value.data.game_rotation_vector.k = sh2_value.un.gameRotationVector.k;
            value.data.game_rotation_vector.real = sh2_value.un.gameRotationVector.real;
            value.data.game_rotation_vector.accuracy_rad = 0.0f;
            break;

        case BNO085_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
            value.data.geomagnetic_rotation_vector.i = sh2_value.un.geoMagRotationVector.i;
            value.data.geomagnetic_rotation_vector.j = sh2_value.un.geoMagRotationVector.j;
            value.data.geomagnetic_rotation_vector.k = sh2_value.un.geoMagRotationVector.k;
            value.data.geomagnetic_rotation_vector.real = sh2_value.un.geoMagRotationVector.real;
            value.data.geomagnetic_rotation_vector.accuracy_rad = 0.0f;
            break;

        case BNO085_SENSOR_PRESSURE:
            value.data.pressure.value = sh2_value.un.pressure.value;
            break;

        case BNO085_SENSOR_AMBIENT_LIGHT:
            value.data.ambient_light.value = sh2_value.un.ambientLight.value;
            break;

        case BNO085_SENSOR_HUMIDITY:
            value.data.humidity.value = sh2_value.un.humidity.value;
            break;

        case BNO085_SENSOR_TEMPERATURE:
            value.data.temperature.value = sh2_value.un.temperature.value;
            break;

        case BNO085_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
            value.data.magnetic_field_uncalibrated.x = sh2_value.un.magneticFieldUncal.x;
            value.data.magnetic_field_uncalibrated.y = sh2_value.un.magneticFieldUncal.y;
            value.data.magnetic_field_uncalibrated.z = sh2_value.un.magneticFieldUncal.z;
            value.data.magnetic_field_uncalibrated.bias_x = sh2_value.un.magneticFieldUncal.biasX;
            value.data.magnetic_field_uncalibrated.bias_y = sh2_value.un.magneticFieldUncal.biasY;
            value.data.magnetic_field_uncalibrated.bias_z = sh2_value.un.magneticFieldUncal.biasZ;
            break;

        case BNO085_SENSOR_TAP_DETECTOR:
            value.data.tap_detector.tap_type = sh2_value.un.tapDetector.flags >> 6;  /* bit 6 = double tap */
            value.data.tap_detector.direction = sh2_value.un.tapDetector.flags & 0x3F;  /* bits 0-5 = direction */
            break;

        case BNO085_SENSOR_STEP_COUNTER:
            value.data.step_counter.count = sh2_value.un.stepCounter.steps;
            break;

        case BNO085_SENSOR_SIGNIFICANT_MOTION:
            value.data.significant_motion.event = sh2_value.un.sigMotion.motion;
            break;

        case BNO085_SENSOR_STABILITY_CLASSIFIER:
            value.data.stability_classifier.activity = sh2_value.un.stabilityClassifier.classification;
            break;

        case BNO085_SENSOR_RAW_ACCELEROMETER:
            value.data.raw_accelerometer.x = sh2_value.un.rawAccelerometer.x;
            value.data.raw_accelerometer.y = sh2_value.un.rawAccelerometer.y;
            value.data.raw_accelerometer.z = sh2_value.un.rawAccelerometer.z;
            break;

        case BNO085_SENSOR_RAW_GYROSCOPE:
            value.data.raw_gyroscope.x = sh2_value.un.rawGyroscope.x;
            value.data.raw_gyroscope.y = sh2_value.un.rawGyroscope.y;
            value.data.raw_gyroscope.z = sh2_value.un.rawGyroscope.z;
            break;

        case BNO085_SENSOR_RAW_MAGNETOMETER:
            value.data.raw_magnetometer.x = sh2_value.un.rawMagnetometer.x;
            value.data.raw_magnetometer.y = sh2_value.un.rawMagnetometer.y;
            value.data.raw_magnetometer.z = sh2_value.un.rawMagnetometer.z;
            break;

        case BNO085_SENSOR_STEP_DETECTOR:
            value.data.step_detector.event = 1;  /* Step detected */
            break;

        case BNO085_SENSOR_SHAKE_DETECTOR:
            value.data.shake_detector.event = sh2_value.un.shakeDetector.shake > 0 ? 1 : 0;
            break;

        case BNO085_SENSOR_FLIP_DETECTOR:
            value.data.flip_detector.event = sh2_value.un.flipDetector.flip > 0 ? 1 : 0;
            break;

        case BNO085_SENSOR_PICKUP_DETECTOR:
            value.data.pickup_detector.event = sh2_value.un.pickupDetector.pickup > 0 ? 1 : 0;
            break;

        case BNO085_SENSOR_PERSONAL_ACTIVITY_CLASSIFIER:
            value.data.personal_activity_classifier.activity = sh2_value.un.personalActivityClassifier.mostLikelyState;
            break;

        case BNO085_SENSOR_SLEEP_DETECTOR:
            value.data.sleep_detector.event = sh2_value.un.sleepDetector.sleepState;
            break;

        case BNO085_SENSOR_TILT_DETECTOR:
            value.data.tilt_detector.event = sh2_value.un.tiltDetector.tilt > 0 ? 1 : 0;
            break;

        case BNO085_SENSOR_POCKET_DETECTOR:
            value.data.pocket_detector.event = sh2_value.un.pocketDetector.pocket > 0 ? 1 : 0;
            break;

        case BNO085_SENSOR_CIRCLE_DETECTOR:
            value.data.circle_detector.event = sh2_value.un.circleDetector.circle > 0 ? 1 : 0;
            break;

        case BNO085_SENSOR_AR_VR_STABILIZED_RV:
            value.data.ar_vr_stabilized_rv.i = sh2_value.un.arvrStabilizedRV.i;
            value.data.ar_vr_stabilized_rv.j = sh2_value.un.arvrStabilizedRV.j;
            value.data.ar_vr_stabilized_rv.k = sh2_value.un.arvrStabilizedRV.k;
            value.data.ar_vr_stabilized_rv.real = sh2_value.un.arvrStabilizedRV.real;
            value.data.ar_vr_stabilized_rv.accuracy_rad = sh2_value.un.arvrStabilizedRV.accuracy;
            break;

        case BNO085_SENSOR_GYRO_INTEGRATED_RV:
            value.data.gyro_integrated_rv.i = sh2_value.un.gyroIntegratedRV.i;
            value.data.gyro_integrated_rv.j = sh2_value.un.gyroIntegratedRV.j;
            value.data.gyro_integrated_rv.k = sh2_value.un.gyroIntegratedRV.k;
            value.data.gyro_integrated_rv.real = sh2_value.un.gyroIntegratedRV.real;
            value.data.gyro_integrated_rv.accuracy_rad = 0.0f;
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
    config->i2c_timeout_ms = CONFIG_BNO085_I2C_TIMEOUT_MS;
    config->reset_retry_count = CONFIG_BNO085_RESET_RETRY_COUNT;
    config->reset_retry_delay_ms = CONFIG_BNO085_RESET_RETRY_DELAY_MS;
    config->reset_settle_delay_ms = CONFIG_BNO085_RESET_SETTLE_DELAY_MS;
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

    /* Acquire a free SH2 instance slot */
    const sh2_instance_slot_t *slot = sh2_multi_acquire_slot();
    if (!slot) {
        ESP_LOGE(TAG, "No free BNO085 slot — increase CONFIG_BNO085_MAX_INSTANCES");
        return ESP_ERR_NO_MEM;
    }

    /* Allocate device structure */
    struct bno085_dev_t *dev = heap_caps_calloc(1, sizeof(struct bno085_dev_t), MALLOC_CAP_DEFAULT);
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate device structure");
        sh2_multi_release_slot(slot);
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
    dev->slot = slot;

    ESP_LOGI(TAG, "GPIO pins: INT=%d, RST=%d", int_pin, reset_pin);

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
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&rst_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure RST pin: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }
    /* Perform hardware reset: RST is active-low */
    ESP_LOGI(TAG, "Performing hardware reset on BNO085...");
    gpio_set_level(reset_pin, 0);  /* RST LOW */
    ESP_LOGI(TAG, "RST driven LOW");
    vTaskDelay(pdMS_TO_TICKS(100));  /* Hold reset for 100ms */

    gpio_set_level(reset_pin, 1);  /* RST HIGH (release reset) */
    ESP_LOGI(TAG, "RST driven HIGH, waiting for device stabilization...");
    vTaskDelay(pdMS_TO_TICKS(300));  /* Wait for device to stabilize */

    /* Set up HAL callback table */
    dev->hal_base.open      = bno085_hal_open;
    dev->hal_base.close     = bno085_hal_close;
    dev->hal_base.read      = bno085_hal_read;
    dev->hal_base.write     = bno085_hal_write;
    dev->hal_base.getTimeUs = bno085_hal_getTimeUs;

    /* Open SH2 session via vtable */
    ESP_LOGI(TAG, "Before sh2_open(): RST=%d", gpio_get_level(reset_pin));
    ESP_LOGI(TAG, "Calling sh2_open()...");
    int rc = slot->vtable->open(&dev->hal_base, bno085_event_cb, dev);
    ESP_LOGI(TAG, "After sh2_open(): RST=%d", gpio_get_level(reset_pin));
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "sh2_open() failed, rc=%d", rc);
        sh2_multi_release_slot(slot);
        free(dev);
        return ESP_FAIL;
    }

    dev->initialized = true;
    *out_handle = (bno085_handle_t)dev;

    /* Debug: Check GPIO states after init */
    ESP_LOGI(TAG, "BNO085 initialized successfully (int_pin=%d, reset_pin=%d)", int_pin, reset_pin);
    ESP_LOGI(TAG, "GPIO states final: INT=%d, RST=%d", gpio_get_level(int_pin), gpio_get_level(reset_pin));
    return ESP_OK;
}

void bno085_service(bno085_handle_t handle)
{
    if (!handle) {
        return;
    }

    struct bno085_dev_t *dev = (struct bno085_dev_t *)handle;
    if (!dev->initialized || !dev->slot) {
        return;
    }

    dev->slot->vtable->service();
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

    if (dev->slot) {
        dev->slot->vtable->close();
        sh2_multi_release_slot(dev->slot);
    }
    gpio_set_level(dev->reset_pin, 0);

    dev->initialized = false;
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
        int rc = dev->slot->vtable->setSensorCallback(bno085_sensor_cb, dev);
        if (rc != SH2_OK) {
            ESP_LOGE(TAG, "sh2_setSensorCallback() failed, rc=%d", rc);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Sensor callback registered");
    } else {
        dev->slot->vtable->setSensorCallback(NULL, NULL);
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

    int rc = dev->slot->vtable->setSensorConfig(sensor_id, &config);
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

    int rc = dev->slot->vtable->setSensorConfig(sensor_id, &config);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "Failed to disable sensor 0x%02x, rc=%d", sensor_id, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor 0x%02x disabled", sensor_id);
    return ESP_OK;
}
